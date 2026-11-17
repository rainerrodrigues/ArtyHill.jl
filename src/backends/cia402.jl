# CiA 402 (IEC 61800-7) drive profile codec for EtherCAT process images.

const CIA402_MODE = (csp=Int8(8), csv=Int8(9), cst=Int8(10))

"""
    CiA402Layout(; out_stride, in_stride, controlword=0, target=2, mode_of_operation=-1,
                   statusword=0, position_actual=2, velocity_actual=6, torque_actual=-1)

Byte offsets of the CiA 402 objects inside **one axis's** slice of the output
(RxPDO) and input (TxPDO) process image. Axis `k` starts at `(k-1) * stride`.
Use `-1` for an object that is not mapped. The defaults match a common minimal
mapping: outputs `0x6040 controlword (u16), 0x607A/0x60FF/0x6071 target`, inputs
`0x6041 statusword (u16), 0x6064 position (i32), 0x606C velocity (i32)`.
Check your drive's ESI / PDO assignment and adjust.
"""
Base.@kwdef struct CiA402Layout
    out_stride::Int
    in_stride::Int
    controlword::Int = 0
    target::Int = 2
    mode_of_operation::Int = -1
    statusword::Int = 0
    position_actual::Int = 2
    velocity_actual::Int = 6
    torque_actual::Int = -1
end

"""
    CiA402Codec{N}(layout; mode=:csp, position_scale=1.0, velocity_scale=1.0,
                   torque_scale=1.0, auto_fault_reset=false)

Process-image codec for `N` CiA 402 axes, for use with
`EtherCATBackend(...; codec=CiA402Codec{N}(...))`.

- **Commands** `u[k]`: target of axis `k` in user units — position (`:csp`),
  velocity (`:csv`) or torque (`:cst`). Raw value = `u[k] * scale`
  (e.g. `position_scale` = encoder counts per radian; torque in ‰ of rated torque).
- **Sensors** `y`: `[pos₁, vel₁, torque₁, pos₂, …]` (3 per axis) in user units;
  unmapped objects read as 0.
- **State machine**: on every cycle the statusword is decoded and the
  controlword advances the drive towards *Operation enabled* — but only while
  [`cia402_enable!`](@ref)`(codec, true)` is set. Disabled, the drive is held in
  *Ready to switch on* (no torque). In `:csp`, the target follows the actual
  position until the axis is enabled, so enabling never causes a jump.
- **Faults**: a fault on *any* axis clears the enable flag for all axes, so after
  a reset nothing re-energizes until you call `cia402_enable!(codec, true)` again.
  Request a reset with [`cia402_fault_reset!`](@ref), or pass
  `auto_fault_reset=true` to reset automatically (enable stays cleared either way).
  The reset is sent as rising edges on controlword bit 7 until the drive leaves *Fault*.
"""
struct CiA402Codec{N}
    layout::CiA402Layout
    mode::Int8
    position_scale::Float64
    velocity_scale::Float64
    torque_scale::Float64
    auto_fault_reset::Bool
    enable::Base.RefValue{Bool}
    statusword::MVector{N,UInt16}
    controlword::MVector{N,UInt16}
    position_raw::MVector{N,Int32}
    reset_request::MVector{N,Bool}
end

function CiA402Codec{N}(layout::CiA402Layout; mode::Symbol=:csp, position_scale::Real=1.0,
                        velocity_scale::Real=1.0, torque_scale::Real=1.0,
                        auto_fault_reset::Bool=false) where {N}
    haskey(CIA402_MODE, mode) || throw(ArgumentError("mode must be :csp, :csv or :cst"))
    return CiA402Codec{N}(layout, CIA402_MODE[mode], position_scale, velocity_scale, torque_scale,
                          auto_fault_reset, Ref(false), zeros(MVector{N,UInt16}),
                          zeros(MVector{N,UInt16}), zeros(MVector{N,Int32}), zeros(MVector{N,Bool}))
end

"Allow (`true`) or forbid (`false`) the state machine to enable the drives."
cia402_enable!(c::CiA402Codec, on::Bool) = (c.enable[] = on; c)

"""
    cia402_fault_reset!(codec, axis=:all)

Request a fault reset for one axis (or all). The codec then pulses controlword
bit 7 until the drive leaves *Fault*. The drive ends up in *Switch on disabled*;
re-enable explicitly with [`cia402_enable!`](@ref).
"""
function cia402_fault_reset!(c::CiA402Codec{N}, axis=:all) where {N}
    axes = axis === :all ? (1:N) : (axis:axis)
    for k in axes
        c.reset_request[k] = true
    end
    return c
end

"""
    cia402_state(statusword) -> Symbol
    cia402_state(codec, axis) -> Symbol

Decode a CiA 402 statusword into `:not_ready`, `:switch_on_disabled`,
`:ready_to_switch_on`, `:switched_on`, `:operation_enabled`, `:quick_stop`,
`:fault_reaction` or `:fault`.
"""
@inline function cia402_state(sw::UInt16)
    s4f = sw & 0x004f
    s6f = sw & 0x006f
    s4f == 0x0000 && return :not_ready
    s4f == 0x0040 && return :switch_on_disabled
    s6f == 0x0021 && return :ready_to_switch_on
    s6f == 0x0023 && return :switched_on
    s6f == 0x0027 && return :operation_enabled
    s6f == 0x0007 && return :quick_stop
    s4f == 0x000f && return :fault_reaction
    s4f == 0x0008 && return :fault
    return :not_ready
end
cia402_state(c::CiA402Codec, axis::Integer) = cia402_state(c.statusword[axis])

# Next controlword for a given state (CiA 402 transitions 2, 3, 4, 15).
# `reset` requests a fault reset: bit 7 is toggled so the drive sees rising edges.
@inline function _cia402_next_cw(state::Symbol, enable::Bool, reset::Bool, prev_cw::UInt16)
    state === :fault && return (reset && (prev_cw & 0x0080) == 0) ? 0x0080 : 0x0000
    state === :fault_reaction && return 0x0000
    state === :quick_stop && return 0x0000                             # → switch on disabled
    enable || return 0x0006                                            # shutdown: hold "ready to switch on"
    state === :switch_on_disabled && return 0x0006                     # transition 2
    state === :ready_to_switch_on && return 0x0007                     # transition 3
    state === :switched_on && return 0x000f                            # transition 4
    state === :operation_enabled && return 0x000f
    return 0x0000
end

@inline _ld(::Type{T}, p::Ptr{UInt8}, off::Int) where {T} = off < 0 ? zero(T) : unsafe_load(Ptr{T}(p + off))
@inline _st!(p::Ptr{UInt8}, off::Int, v) = (off >= 0 && unsafe_store!(Ptr{typeof(v)}(p + off), v); nothing)

@inline function decode_inputs!(y, c::CiA402Codec{N}, p::Ptr{UInt8}) where {N}
    L = c.layout
    # Pass 1: latch statuswords; any fault drops the shared enable flag.
    @inbounds for k in 1:N
        q = p + (k - 1) * L.in_stride
        sw = _ld(UInt16, q, L.statusword)
        c.statusword[k] = sw
        st = cia402_state(sw)
        (st === :fault || st === :fault_reaction) && (c.enable[] = false)
    end
    @inbounds for k in 1:N
        q = p + (k - 1) * L.in_stride
        pos = _ld(Int32, q, L.position_actual)
        c.position_raw[k] = pos
        st = cia402_state(c.statusword[k])
        st === :fault || (c.reset_request[k] = false)
        reset = c.auto_fault_reset || c.reset_request[k]
        c.controlword[k] = _cia402_next_cw(st, c.enable[], reset, c.controlword[k])
        j = firstindex(y) + 3 * (k - 1)
        if j + 2 <= lastindex(y)
            y[j]     = pos / c.position_scale
            y[j + 1] = _ld(Int32, q, L.velocity_actual) / c.velocity_scale
            y[j + 2] = _ld(Int16, q, L.torque_actual) / c.torque_scale
        end
    end
    return nothing
end

@inline function encode_outputs!(p::Ptr{UInt8}, c::CiA402Codec{N}, u) where {N}
    L = c.layout
    @inbounds for k in 1:N
        q = p + (k - 1) * L.out_stride
        cw = c.controlword[k]
        _st!(q, L.controlword, cw)
        _st!(q, L.mode_of_operation, c.mode)
        enabled = cia402_state(c.statusword[k]) === :operation_enabled && c.enable[]
        cmd = k <= length(u) ? Float64(u[firstindex(u) + k - 1]) : 0.0
        if c.mode == CIA402_MODE.csp
            raw = enabled ? _sat(Int32, cmd * c.position_scale) : c.position_raw[k]
            _st!(q, L.target, raw)
        elseif c.mode == CIA402_MODE.csv
            _st!(q, L.target, enabled ? _sat(Int32, cmd * c.velocity_scale) : Int32(0))
        else
            _st!(q, L.target, enabled ? _sat(Int16, cmd * c.torque_scale) : Int16(0))
        end
    end
    return nothing
end

# Saturating, NaN-safe float → integer conversion (never throws InexactError).
@inline function _sat(::Type{T}, x::Float64) where {T<:Integer}
    isnan(x) && return zero(T)
    return unsafe_trunc(T, clamp(round(x), Float64(typemin(T)), Float64(typemax(T))))
end

export CiA402Layout, CiA402Codec, cia402_enable!, cia402_state, cia402_fault_reset!
