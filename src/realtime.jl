# Deterministic periodic execution on (PREEMPT_RT) Linux.

const MCL_CURRENT     = Cint(1)
const MCL_FUTURE      = Cint(2)
const CLOCK_MONOTONIC = Cint(1)
const TIMER_ABSTIME   = Cint(1)
const SCHED_OTHER     = Cint(0)
const SCHED_FIFO      = Cint(1)
const EINTR           = Cint(4)
const NS_PER_S        = 1_000_000_000

struct Timespec
    tv_sec::Clong
    tv_nsec::Clong
end
@inline Timespec(ns::Int64) = Timespec(Clong(div(ns, NS_PER_S)), Clong(rem(ns, NS_PER_S)))

const CpuSet = NTuple{16,UInt64}   # glibc cpu_set_t: 1024 bits

"""
    monotonic_ns() -> UInt64

`CLOCK_MONOTONIC` in nanoseconds (same clock `clock_nanosleep` sleeps on).
"""
@inline monotonic_ns() = time_ns()

# Hot-path variant: direct libc call into a preallocated buffer. (Equivalent to
# `time_ns()` on Linux, but avoids the `jl_hrtime` runtime call so AllocCheck can
# prove the loop allocation-free.)
@inline function _now_ns(ts::Base.RefValue{Timespec})
    @static if Sys.islinux()
        ccall(:clock_gettime, Cint, (Cint, Ptr{Timespec}), CLOCK_MONOTONIC, ts)
        t = ts[]
        return Int64(t.tv_sec) * NS_PER_S + Int64(t.tv_nsec)
    else
        return monotonic_ns() % Int64
    end
end

"""
    RTConfig(; period_ns, iterations, priority=80, cpu=-1, lock_memory=true,
               disable_gc=true, strict=false, max_consecutive_overruns=0,
               max_consecutive_errors=3, hist_bucket_ns=1_000, hist_buckets=2_000)

Real-time loop settings.

- `priority`: `SCHED_FIFO` priority 1–99; `0` keeps the default scheduler.
- `cpu`: pin the loop thread to this CPU (isolate it with `isolcpus=`/`nohz_full=`); `-1` = no pinning.
- `lock_memory`: `mlockall(MCL_CURRENT | MCL_FUTURE)` to prevent page faults.
- `disable_gc`: run `GC.gc()` once, then pause the GC for the duration of the loop.
- `strict`: throw if any of the above cannot be applied instead of continuing in degraded (soft) mode.
- `max_consecutive_overruns` / `max_consecutive_errors`: watchdog. The loop stops
  (and drives the safe command, if one was given) after this many overruns / non-OK
  steps in a row. `0` disables the check.
- `hist_bucket_ns`, `hist_buckets`: resolution and range of the latency / jitter
  histograms used by [`latency_percentile`](@ref). The last bucket collects everything above the range.

`SCHED_FIFO` and `mlockall` need root or `CAP_SYS_NICE` / `CAP_IPC_LOCK` plus a
suitable `ulimit -r` / `ulimit -l`.
"""
Base.@kwdef struct RTConfig
    period_ns::Int64
    iterations::Int64
    priority::Int = 80
    cpu::Int = -1
    lock_memory::Bool = true
    disable_gc::Bool = true
    strict::Bool = false
    max_consecutive_overruns::Int = 0
    max_consecutive_errors::Int = 3
    hist_bucket_ns::Int64 = 1_000
    hist_buckets::Int = 2_000
end

"""
    LoopStats

Timing statistics filled by [`run_realtime_loop!`](@ref). `wake_late` is the
scheduling jitter (actual wake-up minus deadline), `latency` the compute time of
`control_step!`, and an overrun is a step that finished after the next deadline.
`abort_reason` is `:none` for a complete run, or `:overruns`, `:errors` or
`:exception` when the loop stopped early. Use [`latency_percentile`](@ref) for
percentiles from the preallocated histograms.
"""
mutable struct LoopStats
    iterations::Int64
    overruns::Int64
    errors::Int64
    max_wake_late_ns::Int64
    sum_wake_late_ns::Int64
    max_latency_ns::Int64
    sum_latency_ns::Int64
    consecutive_overruns::Int64
    consecutive_errors::Int64
    abort_reason::Symbol
    safe_command_sent::Bool
    memory_locked::Bool
    fifo_scheduled::Bool
    cpu_pinned::Bool
    gc_disabled::Bool
    const hist_bucket_ns::Int64
    const latency_hist::Vector{Int64}
    const wake_hist::Vector{Int64}
end

LoopStats(; hist_bucket_ns::Integer=1_000, hist_buckets::Integer=2_000) =
    LoopStats(0, 0, 0, 0, 0, 0, 0, 0, 0, :none, false, false, false, false, false,
              Int64(hist_bucket_ns), zeros(Int64, hist_buckets), zeros(Int64, hist_buckets))
LoopStats(cfg::RTConfig) = LoopStats(; cfg.hist_bucket_ns, cfg.hist_buckets)

@inline function _hist_add!(h::Vector{Int64}, bucket_ns::Int64, v::Int64)
    i = clamp(div(max(v, 0), bucket_ns) + 1, 1, length(h))
    @inbounds h[i] += 1
    return nothing
end

"""
    latency_percentile(stats, p; kind=:latency) -> Int64

Upper bound (ns) of the histogram bucket holding the `p`-quantile (`0 < p ≤ 1`)
of step latency (`kind=:latency`) or wake-up jitter (`kind=:wake`). Returns
`typemax(Int64)` if the quantile falls in the overflow bucket.
"""
function latency_percentile(s::LoopStats, p::Real; kind::Symbol=:latency)
    0 < p <= 1 || throw(ArgumentError("p must be in (0, 1]"))
    h = kind === :latency ? s.latency_hist : kind === :wake ? s.wake_hist :
        throw(ArgumentError("kind must be :latency or :wake"))
    total = sum(h)
    total == 0 && return Int64(0)
    target = ceil(Int64, p * total)
    acc = 0
    for (i, c) in enumerate(h)
        acc += c
        if acc >= target
            return i == length(h) ? typemax(Int64) : Int64(i) * s.hist_bucket_ns
        end
    end
    return typemax(Int64)
end

function Base.show(io::IO, ::MIME"text/plain", s::LoopStats)
    n = max(s.iterations, 1)
    us(x) = x == typemax(Int64) ? "overflow" : string(round(x / 1e3; digits=2), " µs")
    println(io, "LoopStats: ", s.iterations, " iterations, ", s.overruns, " overruns, ", s.errors,
            " errors", s.abort_reason === :none ? "" : string(" — ABORTED (", s.abort_reason, ")"))
    for (label, kind, sum_, max_) in (("wake-up jitter", :wake, s.sum_wake_late_ns, s.max_wake_late_ns),
                                      ("step latency:  ", :latency, s.sum_latency_ns, s.max_latency_ns))
        println(io, "  ", label, " mean ", us(sum_ / n), ", p99 ≤ ", us(latency_percentile(s, 0.99; kind)),
                ", p99.9 ≤ ", us(latency_percentile(s, 0.999; kind)), ", max ", us(max_))
    end
    print(io, "  mlockall=", s.memory_locked, " SCHED_FIFO=", s.fifo_scheduled,
          " pinned=", s.cpu_pinned, " gc_off=", s.gc_disabled, " safe_cmd_sent=", s.safe_command_sent)
end

# --- absolute-time sleep ----------------------------------------------------

@inline function sleep_until!(ts::Base.RefValue{Timespec}, deadline_ns::Int64)
    @static if Sys.islinux()
        ts[] = Timespec(deadline_ns)
        while true
            rc = ccall(:clock_nanosleep, Cint, (Cint, Cint, Ptr{Timespec}, Ptr{Cvoid}),
                       CLOCK_MONOTONIC, TIMER_ABSTIME, ts, C_NULL)
            rc == EINTR || return rc
        end
    else
        # No clock_nanosleep (e.g. macOS): spin. Soft real-time only.
        while _now_ns(ts) < deadline_ns
        end
        return Cint(0)
    end
end

# --- OS setup / teardown ----------------------------------------------------

function _rt_fail(cfg::RTConfig, what)
    msg = "ArtyHill: could not apply $what (errno $(Libc.errno()): $(Libc.strerror()))"
    cfg.strict ? error(msg) : @warn msg * " — continuing in soft real-time mode"
end

struct _SavedOSState
    policy::Cint
    priority::Cint
    affinity::CpuSet
    have_affinity::Bool
    gc_was_enabled::Bool
end

function _rt_setup!(stats::LoopStats, cfg::RTConfig)
    policy = Cint(0); prio = Ref(Cint(0)); aff = Ref(ntuple(_ -> UInt64(0), Val(16)))
    have_aff = false
    if Sys.islinux()
        policy = ccall(:sched_getscheduler, Cint, (Cint,), 0)
        ccall(:sched_getparam, Cint, (Cint, Ptr{Cint}), 0, prio)
        have_aff = ccall(:sched_getaffinity, Cint, (Cint, Csize_t, Ptr{CpuSet}), 0, sizeof(CpuSet), aff) == 0

        if cfg.lock_memory
            stats.memory_locked = ccall(:mlockall, Cint, (Cint,), MCL_CURRENT | MCL_FUTURE) == 0
            stats.memory_locked || _rt_fail(cfg, "mlockall")
        end
        if cfg.cpu >= 0
            0 <= cfg.cpu < 1024 || throw(ArgumentError("cpu must be in 0:1023"))
            words = ntuple(i -> (i - 1) == cfg.cpu ÷ 64 ? UInt64(1) << (cfg.cpu % 64) : UInt64(0), Val(16))
            stats.cpu_pinned = ccall(:sched_setaffinity, Cint, (Cint, Csize_t, Ptr{CpuSet}),
                                     0, sizeof(CpuSet), Ref(words)) == 0
            stats.cpu_pinned || _rt_fail(cfg, "CPU affinity $(cfg.cpu)")
        end
        if cfg.priority > 0
            # pid 0 = the calling OS thread only (Linux threads are tasks).
            stats.fifo_scheduled = ccall(:sched_setscheduler, Cint, (Cint, Cint, Ptr{Cint}),
                                         0, SCHED_FIFO, Ref(Cint(cfg.priority))) == 0
            stats.fifo_scheduled || _rt_fail(cfg, "SCHED_FIFO priority $(cfg.priority)")
        end
    elseif cfg.strict
        error("ArtyHill: hard real-time setup is only supported on Linux")
    end
    gc_was = GC.enable(true)
    if cfg.disable_gc
        GC.gc()
        GC.enable(false)
        stats.gc_disabled = true
    end
    return _SavedOSState(policy, prio[], aff[], have_aff, gc_was)
end

function _rt_teardown!(stats::LoopStats, saved::_SavedOSState)
    GC.enable(saved.gc_was_enabled)
    if Sys.islinux()
        stats.fifo_scheduled &&
            ccall(:sched_setscheduler, Cint, (Cint, Cint, Ptr{Cint}), 0, saved.policy, Ref(saved.priority))
        stats.cpu_pinned && saved.have_affinity &&
            ccall(:sched_setaffinity, Cint, (Cint, Csize_t, Ptr{CpuSet}), 0, sizeof(CpuSet), Ref(saved.affinity))
        stats.memory_locked && ccall(:munlockall, Cint, ())
    end
    return nothing
end

# --- the hot loop (function barrier: fully specialized, allocation-free) ----

@inline _log!(::Nothing, ws, t, late, lat, st, ovr) = nothing
@inline _log!(ring::SPSCRing, ws, t, late, lat, st, ovr) =
    (try_push!(ring, TelemetrySample(ws, t, late, lat, st, ovr)); nothing)

@inline _stop_requested(::Nothing) = false
@inline _stop_requested(flag::Threads.Atomic{Bool}) = flag[]

function _rt_inner!(stats::LoopStats, ws::ControlWorkspace, controller!::F, backend::B,
                    period_ns::Int64, iterations::Int64, telemetry::R,
                    ts::Base.RefValue{Timespec}, max_ovr::Int, max_err::Int, stop::S) where {F,B,R,S}
    next_ns = _now_ns(ts)
    bucket = stats.hist_bucket_ns
    for i in 1:iterations
        next_ns += period_ns
        sleep_until!(ts, next_ns)
        t_wake = _now_ns(ts)
        st = control_step!(ws, controller!, backend)
        hb = heartbeat!(backend, i)
        st == HIL_OK && (st = hb)
        t_done = _now_ns(ts)

        late = t_wake - next_ns
        lat = t_done - t_wake
        ovr = t_done > next_ns + period_ns
        stats.iterations += 1
        stats.sum_wake_late_ns += late
        stats.sum_latency_ns += lat
        late > stats.max_wake_late_ns && (stats.max_wake_late_ns = late)
        lat > stats.max_latency_ns && (stats.max_latency_ns = lat)
        _hist_add!(stats.latency_hist, bucket, lat)
        _hist_add!(stats.wake_hist, bucket, late)
        if ovr
            stats.overruns += 1
            stats.consecutive_overruns += 1
        else
            stats.consecutive_overruns = 0
        end
        if st != HIL_OK
            stats.errors += 1
            stats.consecutive_errors += 1
        else
            stats.consecutive_errors = 0
        end
        _log!(telemetry, ws, t_wake, late, lat, st, ovr)
        next_ns += cycle_correction_ns(backend, period_ns)   # e.g. DC phase lock

        if _stop_requested(stop)
            stats.abort_reason = :stopped
            break
        elseif max_ovr > 0 && stats.consecutive_overruns >= max_ovr
            stats.abort_reason = :overruns
            break
        elseif max_err > 0 && stats.consecutive_errors >= max_err
            stats.abort_reason = :errors
            break
        end
    end
    return stats
end

"""
    send_safe_command!(ws, backend, safe_command) -> Int32

Copy `safe_command` into `ws.u_cmd` and write it to the backend, bypassing the
controller. Used by [`run_realtime_loop!`](@ref) on watchdog trips, exceptions
and normal exit; call it yourself from emergency-stop handlers.
"""
function send_safe_command!(ws::ControlWorkspace, backend::AbstractHILBackend, safe_command)
    length(safe_command) == length(ws.u_cmd) ||
        throw(DimensionMismatch("safe_command has length $(length(safe_command)), expected $(length(ws.u_cmd))"))
    @inbounds for i in eachindex(ws.u_cmd)
        ws.u_cmd[i] = safe_command[i]
    end
    return write_actuators!(ws.u_cmd, backend)
end

"""
    run_realtime_loop!(ws, controller!, backend, cfg::RTConfig;
                       telemetry=nothing, safe_command=nothing, stop=nothing) -> LoopStats
    run_realtime_loop!(ws, controller!, backend, period_ns, iterations; kwargs...)

Run `control_step!(ws, controller!, backend)` every `period_ns` nanoseconds for
`iterations` cycles on the calling OS thread:

1. `mlockall` memory, pin the thread to `cfg.cpu`, switch it to `SCHED_FIFO`;
2. collect garbage once, then pause the GC;
3. sleep with `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` against absolute
   deadlines, so there is no cumulative drift;
4. after each step call [`heartbeat!`](@ref)`(backend, i)`, shift the next
   deadline by [`cycle_correction_ns`](@ref) (DC phase lock), and check the
   watchdog limits (`cfg.max_consecutive_overruns`, `cfg.max_consecutive_errors`);
5. on exit — normal, watchdog trip or exception — write `safe_command`
   (e.g. zero torque) if one was given, then restore scheduler, affinity, memory
   locking and GC state. Exceptions are rethrown after that.

`telemetry` may be an [`SPSCRing`](@ref) built with `SPSCRing(ws, capacity)`; one
`TelemetrySample` is pushed per cycle (dropped, never blocking, when full).
`stop` may be a `Threads.Atomic{Bool}`: setting it ends the loop after the
current cycle (`abort_reason = :stopped`, safe command still sent). To run the
loop on its own thread, see [`spawn_realtime_loop`](@ref).

The backend is *not* initialized or closed here — call `init_backend!` /
`close_backend!` around it. Run a warm-up (e.g. a few `control_step!` calls) first
so everything is compiled before the deadlines start.

Note: with Julia's GC paused, *other* Julia threads must not allocate heavily
during the loop. Run the loop on a dedicated thread (e.g. start Julia with
`--threads=2` and keep the consumer side light).
"""
function run_realtime_loop!(ws::ControlWorkspace, controller!, backend::AbstractHILBackend,
                            cfg::RTConfig; telemetry::Union{Nothing,SPSCRing}=nothing,
                            safe_command=nothing, stop::Union{Nothing,Threads.Atomic{Bool}}=nothing)
    cfg.period_ns > 0 || throw(ArgumentError("period_ns must be positive"))
    safe_command === nothing || length(safe_command) == length(ws.u_cmd) ||
        throw(DimensionMismatch("safe_command must have length $(length(ws.u_cmd))"))
    stats = LoopStats(cfg)
    ts = Ref(Timespec(0, 0))
    # Compile everything before entering the deadline-driven section.
    precompile(_rt_inner!, (typeof(stats), typeof(ws), typeof(controller!), typeof(backend),
                            Int64, Int64, typeof(telemetry), typeof(ts), Int, Int, typeof(stop)))
    saved = _rt_setup!(stats, cfg)
    try
        _rt_inner!(stats, ws, controller!, backend, cfg.period_ns, cfg.iterations, telemetry, ts,
                   cfg.max_consecutive_overruns, cfg.max_consecutive_errors, stop)
    catch
        stats.abort_reason = :exception
        rethrow()
    finally
        if safe_command !== nothing
            try
                stats.safe_command_sent = send_safe_command!(ws, backend, safe_command) == HIL_OK
            catch err
                @error "ArtyHill: failed to send safe command" exception = err
            end
        end
        _rt_teardown!(stats, saved)
    end
    return stats
end

run_realtime_loop!(ws::ControlWorkspace, controller!, backend::AbstractHILBackend,
                   period_ns::Integer, iterations::Integer; telemetry=nothing,
                   safe_command=nothing, stop=nothing, kwargs...) =
    run_realtime_loop!(ws, controller!, backend,
                       RTConfig(; period_ns=Int64(period_ns), iterations=Int64(iterations), kwargs...);
                       telemetry, safe_command, stop)

# --- running the loop on a dedicated thread ------------------------------------

"""
    RealtimeLoop

Handle returned by [`spawn_realtime_loop`](@ref). `stop_loop!(h)` asks the loop
to finish its current cycle and exit; `fetch(h)` waits and returns the
[`LoopStats`](@ref) (rethrowing any error from the loop); `istaskdone(h)` polls.
"""
struct RealtimeLoop
    task::Task
    stop::Threads.Atomic{Bool}
    thread::Int
end

"Ask a spawned real-time loop to stop after its current cycle. Returns the handle."
stop_loop!(h::RealtimeLoop) = (h.stop[] = true; h)
Base.fetch(h::RealtimeLoop) = fetch(h.task)
Base.wait(h::RealtimeLoop) = (wait(h.task); h)
Base.istaskdone(h::RealtimeLoop) = istaskdone(h.task)

"Julia worker threads that can run tasks (excludes foreign/adopted thread slots)."
_worker_threads() = [t for t in 1:Threads.maxthreadid() if Threads.threadpool(t) in (:default, :interactive)]

"Default thread for [`spawn_realtime_loop`](@ref): the last thread of the `:default` pool."
default_rt_thread() = last(t for t in _worker_threads() if Threads.threadpool(t) === :default)

"""
    spawn_realtime_loop(ws, controller!, backend, cfg::RTConfig;
                        thread=default_rt_thread(), telemetry=nothing, safe_command=nothing) -> RealtimeLoop
    spawn_realtime_loop(ws, controller!, backend, period_ns, iterations; thread, kwargs...)

Run [`run_realtime_loop!`](@ref) on Julia thread `thread` (default: the last
thread of the `:default` pool), leaving the calling thread free for telemetry,
UI or a REPL. The task is pinned to that thread and never yields while the loop
runs, so the `SCHED_FIFO`/CPU-affinity settings apply to exactly that OS thread.
Start Julia with at least two threads (`julia --threads=2`). Use
`iterations=typemax(Int64)` with [`stop_loop!`](@ref) for an open-ended run.
"""
function spawn_realtime_loop(ws::ControlWorkspace, controller!, backend::AbstractHILBackend, cfg::RTConfig;
                             thread::Integer=default_rt_thread(), telemetry=nothing, safe_command=nothing)
    workers = _worker_threads()
    thread in workers || throw(ArgumentError("thread must be one of the Julia worker threads $workers"))
    if thread == Threads.threadid()
        @warn "ArtyHill: real-time loop spawned on the calling thread $thread; start Julia with " *
              "--threads=2 (or more) to give it a thread of its own"
    end
    stop = Threads.Atomic{Bool}(false)
    task = Task(() -> run_realtime_loop!(ws, controller!, backend, cfg; telemetry, safe_command, stop))
    task.sticky = true
    ccall(:jl_set_task_tid, Cint, (Any, Cint), task, thread - 1) == 1 ||
        throw(ArgumentError("could not place the loop on thread $thread"))
    schedule(task)
    return RealtimeLoop(task, stop, Int(thread))
end

spawn_realtime_loop(ws::ControlWorkspace, controller!, backend::AbstractHILBackend,
                    period_ns::Integer, iterations::Integer; thread::Integer=default_rt_thread(),
                    telemetry=nothing, safe_command=nothing, kwargs...) =
    spawn_realtime_loop(ws, controller!, backend,
                        RTConfig(; period_ns=Int64(period_ns), iterations=Int64(iterations), kwargs...);
                        thread, telemetry, safe_command)

export RealtimeLoop, spawn_realtime_loop, stop_loop!
