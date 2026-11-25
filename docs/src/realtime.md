# Real-time loop

[`run_realtime_loop!`](@ref) runs `control_step!` periodically on the calling OS thread:

1. `mlockall(MCL_CURRENT | MCL_FUTURE)`, CPU pinning, `SCHED_FIFO`;
2. one `GC.gc()`, then the GC is paused;
3. `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` against absolute deadlines (no drift);
4. per-cycle statistics, histograms and an optional telemetry sample;
5. on exit, the safe command is sent and every OS setting is restored.

If a setup step fails without `strict=true`, the loop warns and continues in
soft real-time mode. [`LoopStats`](@ref) records which steps were actually applied.

## Running on a dedicated thread

[`spawn_realtime_loop`](@ref) runs the same loop on a Julia worker thread of its
own (by default the last thread of the `:default` pool) and returns at once.
The calling thread stays free for telemetry, plotting or a REPL:

```julia
# julia --threads=2
h = spawn_realtime_loop(ws, ctrl, backend, 1_000_000, typemax(Int64);
                        telemetry=ring, priority=80, cpu=3, safe_command=[0.0])
# ... drain `ring`, watch, tweak setpoints ...
stop_loop!(h)              # finishes the current cycle, sends the safe command
stats = fetch(h)           # LoopStats; abort_reason === :stopped
```

The task is pinned to its thread and never yields during the loop, so
`SCHED_FIFO` and CPU affinity apply to exactly that OS thread. Any
`run_realtime_loop!` also accepts `stop=Threads.Atomic{Bool}(false)` directly.

## Checklist

- PREEMPT_RT kernel; isolate a core (`isolcpus=3 nohz_full=3 rcu_nocbs=3`) and pass `cpu=3`.
- Root, or `CAP_SYS_NICE` + `CAP_IPC_LOCK` with `ulimit -r 99` and `ulimit -l unlimited`.
  Julia reserves a lot of virtual memory, so `mlockall` fails under the default memlock limit.
- Start Julia with `--threads=2` and keep other threads allocation-light: the GC is paused process-wide.
- Warm up with a few `control_step!` calls so compilation never lands on a deadline.

## Measuring

```julia
stats = run_realtime_loop!(ws, ctrl, backend, cfg)
latency_percentile(stats, 0.999)               # step compute time, ns
latency_percentile(stats, 0.999; kind=:wake)   # scheduling jitter, ns
```

`bench/compare_cyclictest.sh` runs `cyclictest` and ArtyHill with identical
period, priority and CPU, so you can see how much the Julia loop adds on top
of the kernel's own latency. [Tutorial: PREEMPT_RT latency](@ref) walks through
setting up a machine and running it.

## Telemetry

Push one [`TelemetrySample`](@ref) per cycle into an [`SPSCRing`](@ref). It never
blocks; when the ring is full, samples are dropped and counted. Drain it from
another thread and save with [`write_telemetry_csv`](@ref).
