"""
    AbstractHILBackend

Supertype of all hardware backends. A concrete backend `B` must implement

    init_backend!(b::B)::Int32
    read_sensors!(y::AbstractVector, b::B)::Int32     # fill y in place
    write_actuators!(u::AbstractVector, b::B)::Int32  # consume u
    close_backend!(b::B)::Int32

`read_sensors!` and `write_actuators!` are called from the real-time thread and
must not allocate on the Julia heap (verify with [`check_step_allocs`](@ref)).
All functions return `HIL_OK` (0) on success and a negative code on failure.
"""
abstract type AbstractHILBackend end

# Fallbacks fail loudly at setup time instead of silently returning `nothing`
# (which would fail the ::Int32 conversion in an opaque way).
for f in (:init_backend!, :close_backend!)
    @eval $f(b::AbstractHILBackend) =
        throw(ArgumentError(string($(string(f)), " is not implemented for ", typeof(b))))
end
read_sensors!(y, b::AbstractHILBackend) =
    throw(ArgumentError("read_sensors! is not implemented for $(typeof(b))"))
write_actuators!(u, b::AbstractHILBackend) =
    throw(ArgumentError("write_actuators! is not implemented for $(typeof(b))"))

"""
    fixed_cstring(s, Val(N)) -> NTuple{N,UInt8}

Pack `s` into a NUL-terminated fixed-size buffer (for interface names etc.).
"""
function fixed_cstring(s::AbstractString, ::Val{N}) where {N}
    bytes = codeunits(s)
    length(bytes) < N || throw(ArgumentError("\"$s\" does not fit in $N bytes (incl. NUL)"))
    return ntuple(i -> i <= length(bytes) ? UInt8(bytes[i]) : 0x00, Val(N))
end

cstring_to_string(t::NTuple{N,UInt8}) where {N} =
    String(UInt8[c for c in Iterators.takewhile(!iszero, t)])
