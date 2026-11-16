module ArtyHillVeriJulExt

using ArtyHill
using ArtyHill: HIL_OK, HIL_ERR_NOT_INIT
import VeriJul
using VeriJul: DUT

# Raw access to a ≤64-bit port of a Verilated model (pointers resolved once).
struct RawPort
    get_ptr::Ptr{Cvoid}
    set_ptr::Ptr{Cvoid}
    width::Int
end

"""
VeriJul co-simulation backend; build with `verijul_backend(dut; ...)`.
"""
mutable struct VeriJulBackend{NI,NO} <: AbstractHILBackend
    const dut::DUT                       # keeps the model (and its library) alive
    const inputs::NTuple{NI,RawPort}
    const outputs::NTuple{NO,RawPort}
    const clock::RawPort
    const cycles_per_step::Int
    const input_scale::NTuple{NI,Float64}
    const output_scale::NTuple{NO,Float64}
    const signed_outputs::Bool
    const eval_ptr::Ptr{Cvoid}
    model_ptr::Ptr{Cvoid}
    time::UInt64
end

function _rawport(dut::DUT, name::Symbol, want_input::Bool)
    h = VeriJul.Device.handle(dut, name)
    h.port.width <= 64 || throw(ArgumentError("port `$name` is $(h.port.width) bits; at most 64 supported"))
    want_input && h.port.direction != :in && throw(ArgumentError("port `$name` is not an input"))
    return RawPort(h.get_ptr, h.set_ptr, h.port.width)
end

_expand(v, n) = length(v) == 1 ? ntuple(_ -> Float64(first(v)), n) :
    (length(v) == n ? Tuple(Float64.(v)) : throw(DimensionMismatch("expected $n scale factors")))

function ArtyHill.verijul_backend(dut::DUT; inputs, outputs, clock::Symbol=:clk,
                                  cycles_per_step::Integer=1, input_scale=1.0, output_scale=1.0,
                                  signed_outputs::Bool=true)
    cycles_per_step >= 1 || throw(ArgumentError("cycles_per_step must be ≥ 1"))
    ins = Tuple(_rawport(dut, Symbol(n), true) for n in inputs)
    outs = Tuple(_rawport(dut, Symbol(n), false) for n in outputs)
    model = VeriJul.Device.model(dut)
    return VeriJulBackend{length(ins),length(outs)}(dut, ins, outs, _rawport(dut, clock, true),
                                                    Int(cycles_per_step), _expand(input_scale, length(ins)),
                                                    _expand(output_scale, length(outs)), signed_outputs,
                                                    model.eval_ptr, C_NULL, UInt64(0))
end

@inline function _set!(m::Ptr{Cvoid}, p::RawPort, v::UInt64)
    w = p.width
    v &= w >= 64 ? typemax(UInt64) : (UInt64(1) << w) - 1
    if w <= 8
        ccall(p.set_ptr, Cvoid, (Ptr{Cvoid}, UInt8), m, v % UInt8)
    elseif w <= 16
        ccall(p.set_ptr, Cvoid, (Ptr{Cvoid}, UInt16), m, v % UInt16)
    elseif w <= 32
        ccall(p.set_ptr, Cvoid, (Ptr{Cvoid}, UInt32), m, v % UInt32)
    else
        ccall(p.set_ptr, Cvoid, (Ptr{Cvoid}, UInt64), m, v)
    end
end

@inline function _get(m::Ptr{Cvoid}, p::RawPort)::UInt64
    w = p.width
    w <= 8  && return UInt64(ccall(p.get_ptr, UInt8, (Ptr{Cvoid},), m))
    w <= 16 && return UInt64(ccall(p.get_ptr, UInt16, (Ptr{Cvoid},), m))
    w <= 32 && return UInt64(ccall(p.get_ptr, UInt32, (Ptr{Cvoid},), m))
    return ccall(p.get_ptr, UInt64, (Ptr{Cvoid},), m)
end

@inline _signext(v::UInt64, w::Int) = w >= 64 ? reinterpret(Int64, v) : (reinterpret(Int64, v << (64 - w)) >> (64 - w))

@inline function _eval!(b::VeriJulBackend)
    ccall(b.eval_ptr, Cvoid, (Ptr{Cvoid}, UInt64), b.model_ptr, b.time)
    b.time += 1
end

function ArtyHill.init_backend!(b::VeriJulBackend)
    m = VeriJul.Interop.check_alive(VeriJul.Device.model(b.dut)).ptr
    b.model_ptr = m
    b.time = 0
    _set!(m, b.clock, UInt64(0))
    _eval!(b)
    return HIL_OK
end

ArtyHill.close_backend!(b::VeriJulBackend) = (b.model_ptr = C_NULL; HIL_OK)

@inline function ArtyHill.write_actuators!(u, b::VeriJulBackend{NI}) where {NI}
    m = b.model_ptr
    m == C_NULL && return HIL_ERR_NOT_INIT
    @inbounds for k in 1:NI
        x = Float64(u[firstindex(u) + k - 1]) * b.input_scale[k]
        raw = isfinite(x) ? unsafe_trunc(Int64, clamp(round(x), -9.2e18, 9.2e18)) : Int64(0)
        _set!(m, b.inputs[k], reinterpret(UInt64, raw))
    end
    for _ in 1:b.cycles_per_step
        _set!(m, b.clock, UInt64(1)); _eval!(b)
        _set!(m, b.clock, UInt64(0)); _eval!(b)
    end
    return HIL_OK
end

@inline function ArtyHill.read_sensors!(y, b::VeriJulBackend{NI,NO}) where {NI,NO}
    m = b.model_ptr
    m == C_NULL && return HIL_ERR_NOT_INIT
    @inbounds for k in 1:NO
        p = b.outputs[k]
        v = _get(m, p)
        val = b.signed_outputs ? Float64(_signext(v, p.width)) : Float64(v)
        y[firstindex(y) + k - 1] = val / b.output_scale[k]
    end
    return HIL_OK
end

end
