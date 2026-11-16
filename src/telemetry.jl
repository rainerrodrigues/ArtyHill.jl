"""
    SPSCRing{T}(capacity)

Lock-free single-producer / single-consumer ring buffer for `isbits` `T`.
The real-time thread calls [`try_push!`](@ref) (never blocks, never allocates;
returns `false` and counts a drop when full). A background thread calls
[`try_pop!`](@ref) / [`drain!`](@ref) to move samples to disk or the network.
`capacity` is rounded up to a power of two.
"""
mutable struct SPSCRing{T}
    const buf::Vector{T}
    const mask::Int
    @atomic head::Int                 # next slot to write (owned by producer)
    _pad1::NTuple{7,Int}              # keep head/tail on separate cache lines
    @atomic tail::Int                 # next slot to read  (owned by consumer)
    _pad2::NTuple{7,Int}
    @atomic dropped::Int
end

function SPSCRing{T}(capacity::Integer) where {T}
    isbitstype(T) || throw(ArgumentError("SPSCRing element type must be isbits, got $T"))
    cap = nextpow(2, max(2, Int(capacity)))
    z = ntuple(_ -> 0, Val(7))
    return SPSCRing{T}(Vector{T}(undef, cap), cap - 1, 0, z, 0, z, 0)
end

Base.length(r::SPSCRing) = (@atomic :acquire r.head) - (@atomic :acquire r.tail)
capacity(r::SPSCRing) = r.mask + 1
Base.isempty(r::SPSCRing) = length(r) == 0

"Producer side. Returns `true` if `x` was stored, `false` (and counts a drop) if full."
@inline function try_push!(r::SPSCRing{T}, x::T) where {T}
    h = @atomic :monotonic r.head
    t = @atomic :acquire r.tail
    if h - t > r.mask
        @atomic :monotonic r.dropped += 1
        return false
    end
    @inbounds r.buf[(h & r.mask) + 1] = x
    @atomic :release r.head = h + 1
    return true
end

"Consumer side. Returns the oldest element, or `nothing` if empty."
@inline function try_pop!(r::SPSCRing{T}) where {T}
    t = @atomic :monotonic r.tail
    h = @atomic :acquire r.head
    h == t && return nothing
    x = @inbounds r.buf[(t & r.mask) + 1]
    @atomic :release r.tail = t + 1
    return x
end

"""
    drain!(f, r::SPSCRing) -> Int

Pop every currently available element and call `f(x)` on it. Returns the count.
"""
function drain!(f, r::SPSCRing)
    n = 0
    while true
        x = try_pop!(r)
        x === nothing && return n
        f(x)
        n += 1
    end
end

"""
    TelemetrySample{P,M}

One logged control cycle: wake-up timestamp (`CLOCK_MONOTONIC` ns), wake-up
lateness and compute latency (ns), step status, whether the step overran its
deadline, measurements and commands.
"""
struct TelemetrySample{P,M}
    t_ns::Int64
    wake_late_ns::Int64
    latency_ns::Int64
    status::Int32
    overrun::Bool
    y::SVector{P,Float64}
    u::SVector{M,Float64}
end

TelemetrySample(ws::ControlWorkspace{N,M,P}, t, late, lat, st, ovr::Bool=false) where {N,M,P} =
    TelemetrySample{P,M}(t, late, lat, st, ovr, SVector{P,Float64}(ws.y_meas), SVector{M,Float64}(ws.u_cmd))

"Element type of the telemetry ring matching workspace `ws`."
telemetry_type(::ControlWorkspace{N,M,P}) where {N,M,P} = TelemetrySample{P,M}

"""
    SPSCRing(ws::ControlWorkspace, capacity)

Telemetry ring sized for the samples produced by `ws`.
"""
SPSCRing(ws::ControlWorkspace, capacity::Integer) = SPSCRing{telemetry_type(ws)}(capacity)

"""
    write_telemetry_csv(io, samples)

Dump collected `TelemetrySample`s as CSV (header + one row per cycle) for
post-processing / system identification outside the real-time process.
"""
function write_telemetry_csv(io::IO, samples::AbstractVector{TelemetrySample{P,M}}) where {P,M}
    print(io, "t_ns,wake_late_ns,latency_ns,status,overrun")
    for i in 1:P; print(io, ",y", i); end
    for i in 1:M; print(io, ",u", i); end
    println(io)
    for s in samples
        print(io, s.t_ns, ',', s.wake_late_ns, ',', s.latency_ns, ',', s.status, ',', Int(s.overrun))
        for v in s.y; print(io, ',', v); end
        for v in s.u; print(io, ',', v); end
        println(io)
    end
end

export capacity, telemetry_type, write_telemetry_csv
