# EtherCAT backend on top of SOEM (Simple Open EtherCAT Master, legacy `ec_*` API
# of SOEM 1.4). SOEM is resolved lazily on first use, so ArtyHill has no hard
# binary dependency and loads fine on machines without it.

const EC_STATE_INIT        = UInt16(0x01)
const EC_STATE_PRE_OP      = UInt16(0x02)
const EC_STATE_SAFE_OP     = UInt16(0x04)
const EC_STATE_OPERATIONAL = UInt16(0x08)
const EC_TIMEOUTRET        = Cint(2_000)       # µs
const EC_TIMEOUTSTATE      = Cint(2_000_000)   # µs
const EC_TIMEOUTRXM        = Cint(700_000)     # µs, SOEM's default mailbox timeout
const EC_ERR_TYPE_SDO_ERROR = Cint(0)          # ec_err_type in ethercattype.h

"""
    PackedFloat64Codec()

Default process-image codec: outputs are packed `Float64`s starting at the
output offset, inputs are packed `Float64`s starting at the input offset.
Real drives (CiA 402 etc.) need a custom codec implementing

    encode_outputs!(iomap::Ptr{UInt8}, codec, u)
    decode_inputs!(y, codec, iomap::Ptr{UInt8})

where `iomap` already points at the start of the output / input region.
"""
struct PackedFloat64Codec end

@inline function encode_outputs!(p::Ptr{UInt8}, ::PackedFloat64Codec, u)
    q = Ptr{Float64}(p)
    @inbounds for (k, i) in enumerate(eachindex(u))
        unsafe_store!(q, Float64(u[i]), k)
    end
end

@inline function decode_inputs!(y, ::PackedFloat64Codec, p::Ptr{UInt8})
    q = Ptr{Float64}(p)
    @inbounds for (k, i) in enumerate(eachindex(y))
        y[i] = unsafe_load(q, k)
    end
end

"""
    LIBSOEM

Library used for SOEM calls (default `"libsoem"`, found via the system loader
path). It is a compile-time constant so hot-path `ccall`s bind statically (no
indirect function-pointer call, which also lets AllocCheck prove the step
allocation-free). Change it with [`set_libsoem!`](@ref), which stores a
Preferences.jl setting and takes effect after restarting Julia.

Build SOEM 1.4 as a shared library: `cmake -DBUILD_SHARED_LIBS=ON .. && make && sudo make install`.
"""
const LIBSOEM = @load_preference("libsoem", "libsoem")

"""
    set_libsoem!(path)

Point ArtyHill at a specific SOEM shared library (absolute path or soname).
Saved in `LocalPreferences.toml` of the active project; restart Julia to apply.
"""
function set_libsoem!(path::AbstractString)
    @set_preferences!("libsoem" => String(path))
    @info "ArtyHill: SOEM library set to $(repr(path)); restart Julia for it to take effect."
    return path
end

export set_libsoem!

"""
    EtherCATBackend(iface; output_bytes, iomap_size=4096, codec=PackedFloat64Codec(),
                    preop_config=nothing, dc_cycle_ns=0, dc_shift_ns=0,
                    dc_frame_phase_ns=dc_cycle_ns ÷ 2, dc_align=dc_cycle_ns > 0)

EtherCAT master using SOEM ([`LIBSOEM`](@ref)). `output_bytes` is the size of
the output part of the process image (SOEM maps outputs first, then inputs).
One control cycle is: `read_sensors!` receives the frame sent in the previous
cycle, `write_actuators!` encodes commands and sends the next frame — the
standard one-cycle pipeline. The expected working counter is taken from the
last exchange during the transition to OPERATIONAL.

**Drive configuration (CoE).** `preop_config(backend, slave)` is called for every
slave while the network is in PRE-OP, *before* the process image is mapped. Use
[`sdo_write!`](@ref) / [`configure_pdo!`](@ref) there to set parameters and PDO
mappings; SOEM then reads the resulting mapping over CoE when it builds the
process image. An exception aborts `init_backend!` (which then returns `HIL_ERR_SYS`).

**Distributed clocks.** With `dc_cycle_ns > 0`, every slave's Sync0 is started
with that cycle time (shifted by `dc_shift_ns`), which most drives require for
cyclic synchronous modes. With `dc_align` (default on), [`run_realtime_loop!`](@ref)
also steers its own wake-up times so that frames reach the reference clock at
`dc_frame_phase_ns` within each DC cycle — use the same `period_ns` as `dc_cycle_ns`.
See [`dc_phase_error_ns`](@ref).

Raw-socket access requires root or `CAP_NET_RAW` (`sudo setcap cap_net_raw+ep julia`).
"""
mutable struct EtherCATBackend{C} <: AbstractHILBackend
    const iface::NTuple{16,UInt8}
    const iomap::Vector{UInt8}
    const output_bytes::Int
    const codec::C
    const preop_config::Any
    const dc_cycle_ns::Int64
    const dc_shift_ns::Int64
    const dc_frame_phase_ns::Int64
    const dc_align::Bool
    ec_slave::Ptr{UInt16}       # &ec_slave[0]; `state` is the struct's first field
    dc_time::Ptr{Int64}         # &ec_DCtime, updated by every process-data exchange
    initialized::Bool
    slave_count::Int32
    expected_wkc::Int32
    last_wkc::Int32
    dc_integral::Int64
    dc_last_error::Int64
end

function EtherCATBackend(iface::AbstractString; output_bytes::Integer, iomap_size::Integer=4096,
                         codec=PackedFloat64Codec(), preop_config=nothing, dc_cycle_ns::Integer=0,
                         dc_shift_ns::Integer=0, dc_frame_phase_ns::Integer=dc_cycle_ns ÷ 2,
                         dc_align::Bool=dc_cycle_ns > 0)
    dc_cycle_ns >= 0 || throw(ArgumentError("dc_cycle_ns must be ≥ 0"))
    dc_align && dc_cycle_ns == 0 && throw(ArgumentError("dc_align requires dc_cycle_ns > 0"))
    return EtherCATBackend(fixed_cstring(iface, Val(16)), zeros(UInt8, iomap_size),
                           Int(output_bytes), codec, preop_config, Int64(dc_cycle_ns),
                           Int64(dc_shift_ns), Int64(dc_frame_phase_ns), dc_align,
                           Ptr{UInt16}(C_NULL), Ptr{Int64}(C_NULL), false,
                           Int32(0), Int32(0), Int32(0), Int64(0), Int64(0))
end

@inline _ec_send() = ccall((:ec_send_processdata, LIBSOEM), Cint, ())
@inline _ec_receive(timeout) = ccall((:ec_receive_processdata, LIBSOEM), Cint, (Cint,), timeout)
_ec_statecheck(state, timeout; slave=0) =
    ccall((:ec_statecheck, LIBSOEM), UInt16, (UInt16, UInt16, Cint), slave, state, timeout)
_ec_writestate() = ccall((:ec_writestate, LIBSOEM), Cint, (UInt16,), 0)

function init_backend!(b::EtherCATBackend)
    b.initialized && return HIL_OK
    h = Libdl.dlopen(LIBSOEM; throw_error=false)
    h === nothing && return HIL_ERR_SYS
    slave_sym = Libdl.dlsym(h, :ec_slave; throw_error=false)
    dctime_sym = Libdl.dlsym(h, :ec_DCtime; throw_error=false)
    (slave_sym === nothing || dctime_sym === nothing) && return HIL_ERR_SYS
    b.ec_slave = Ptr{UInt16}(slave_sym)
    b.dc_time = Ptr{Int64}(dctime_sym)
    b.dc_integral = 0
    b.dc_last_error = 0

    name = Ref(b.iface)
    ccall((:ec_init, LIBSOEM), Cint, (Ptr{NTuple{16,UInt8}},), name) > 0 || return HIL_ERR_SYS
    b.initialized = true
    b.slave_count = ccall((:ec_config_init, LIBSOEM), Cint, (UInt8,), 0x00)
    b.slave_count > 0 || (close_backend!(b); return HIL_ERR_IO)

    if b.preop_config !== nothing
        _ec_statecheck(EC_STATE_PRE_OP, EC_TIMEOUTSTATE) == EC_STATE_PRE_OP ||
            (close_backend!(b); return HIL_ERR_IO)
        try
            for s in 1:b.slave_count
                b.preop_config(b, Int(s))
            end
        catch err
            @error "ArtyHill: PRE-OP configuration failed" exception = (err, catch_backtrace())
            close_backend!(b)
            return HIL_ERR_SYS
        end
    end

    used = ccall((:ec_config_map, LIBSOEM), Cint, (Ptr{UInt8},), b.iomap)
    (used <= length(b.iomap) && b.output_bytes <= used) || (close_backend!(b); return HIL_ERR_SYS)
    has_dc = ccall((:ec_configdc, LIBSOEM), UInt8, ()) != 0
    if b.dc_cycle_ns > 0
        has_dc || (@error "ArtyHill: dc_cycle_ns set but no slave supports distributed clocks";
                   close_backend!(b); return HIL_ERR_SYS)
        for s in 1:b.slave_count
            ccall((:ec_dcsync0, LIBSOEM), Cvoid, (UInt16, UInt8, UInt32, Int32),
                  s, 0x01, UInt32(b.dc_cycle_ns), Int32(b.dc_shift_ns))
        end
    end
    _ec_statecheck(EC_STATE_SAFE_OP, 4 * EC_TIMEOUTSTATE)

    # Request OPERATIONAL for all slaves while keeping process data flowing.
    unsafe_store!(b.ec_slave, EC_STATE_OPERATIONAL)
    _ec_send(); _ec_receive(EC_TIMEOUTRET)
    _ec_writestate()
    for _ in 1:200
        _ec_send()
        b.last_wkc = _ec_receive(EC_TIMEOUTRET)
        _ec_statecheck(EC_STATE_OPERATIONAL, Cint(50_000))
        unsafe_load(b.ec_slave) == EC_STATE_OPERATIONAL && break
    end
    unsafe_load(b.ec_slave) == EC_STATE_OPERATIONAL || (close_backend!(b); return HIL_ERR_IO)
    b.expected_wkc = b.last_wkc
    _ec_send()   # prime the pipeline for the first read_sensors!
    return HIL_OK
end

function close_backend!(b::EtherCATBackend)
    b.initialized || return HIL_OK
    unsafe_store!(b.ec_slave, EC_STATE_INIT)
    _ec_writestate()
    ccall((:ec_close, LIBSOEM), Cvoid, ())
    b.initialized = false
    return HIL_OK
end

@inline function read_sensors!(y, b::EtherCATBackend)
    b.initialized || return HIL_ERR_NOT_INIT
    b.last_wkc = _ec_receive(EC_TIMEOUTRET)
    decode_inputs!(y, b.codec, pointer(b.iomap) + b.output_bytes)
    return b.last_wkc >= b.expected_wkc ? HIL_OK : HIL_ERR_IO
end

@inline function write_actuators!(u, b::EtherCATBackend)
    b.initialized || return HIL_ERR_NOT_INIT
    encode_outputs!(pointer(b.iomap), b.codec, u)
    return _ec_send() > 0 ? HIL_OK : HIL_ERR_IO
end

# --- distributed-clock alignment ---------------------------------------------

"""
    dc_time(backend) -> Int64

System time (ns since 2000-01-01) of the DC reference clock, as captured by the
most recent process-data exchange. `0` before the first exchange or without DC.
"""
dc_time(b::EtherCATBackend) = b.dc_time == C_NULL ? Int64(0) : unsafe_load(b.dc_time)

"""
    dc_phase_error_ns(backend) -> Int64

Last measured deviation of the frame arrival phase (DC time modulo the DC cycle)
from `dc_frame_phase_ns`, in ns, wrapped to `(-cycle/2, cycle/2]`.
"""
dc_phase_error_ns(b::EtherCATBackend) = b.dc_last_error

@inline function _dc_wrap(x::Int64, cycle::Int64)
    d = mod(x, cycle)
    return d > cycle ÷ 2 ? d - cycle : d
end

# PI controller on the phase between the loop and the DC reference clock
# (the same law as SOEM's `ec_sync` example), clamped to a quarter period.
@inline function cycle_correction_ns(b::EtherCATBackend, period_ns::Int64)
    (b.dc_align && b.initialized) || return Int64(0)
    t = unsafe_load(b.dc_time)
    t == 0 && return Int64(0)
    delta = _dc_wrap(t - b.dc_frame_phase_ns, b.dc_cycle_ns)
    b.dc_last_error = delta
    delta > 0 && (b.dc_integral += 1)
    delta < 0 && (b.dc_integral -= 1)
    corr = -(delta ÷ 100) - (b.dc_integral ÷ 20)
    lim = period_ns ÷ 4
    return clamp(corr, -lim, lim)
end

# --- CoE / SDO -----------------------------------------------------------------

"Layout of SOEM's `ec_errort` (not packed); checked against the C headers in the EtherCAT tests."
struct ECError
    time_sec::UInt32
    time_usec::UInt32
    signal::UInt8
    slave::UInt16
    index::UInt16
    subindex::UInt8
    etype::Cint
    abort_code::Int32
    w1::UInt16
    w2::UInt16
end
ECError() = ECError(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

const SDO_ABORT_MESSAGES = Dict{UInt32,String}(
    0x05030000 => "toggle bit not changed",
    0x05040000 => "SDO protocol timeout",
    0x05040001 => "client/server command specifier not valid",
    0x06010000 => "unsupported access to an object",
    0x06010001 => "attempt to read a write-only object",
    0x06010002 => "attempt to write a read-only object",
    0x06020000 => "object does not exist in the object dictionary",
    0x06040041 => "object cannot be mapped to the PDO",
    0x06040042 => "number and length of objects would exceed PDO length",
    0x06070010 => "data type does not match, length of service parameter does not match",
    0x06090011 => "subindex does not exist",
    0x06090030 => "value range of parameter exceeded",
    0x08000000 => "general error",
    0x08000020 => "data cannot be transferred or stored",
    0x08000022 => "data cannot be transferred or stored because of the present device state",
)

"""
    SDOError(slave, index, subindex, abort_code)

A failed SDO transfer. `abort_code` is the CoE abort code from the slave, or `0`
if the transfer failed without one (timeout, no mailbox).
"""
struct SDOError <: Exception
    slave::Int
    index::UInt16
    subindex::UInt8
    abort_code::UInt32
end

function Base.showerror(io::IO, e::SDOError)
    print(io, "SDOError: slave ", e.slave, " object 0x", string(e.index; base=16, pad=4), ":",
          string(e.subindex; base=16, pad=2))
    if e.abort_code == 0
        print(io, " — no response (timeout or slave without CoE mailbox)")
    else
        print(io, " — abort 0x", string(e.abort_code; base=16, pad=8), " (",
              get(SDO_ABORT_MESSAGES, e.abort_code, "unknown abort code"), ")")
    end
end

function _sdo_abort_code(slave, index, subindex)
    code = UInt32(0)
    err = Ref(ECError())
    while ccall((:ec_poperror, LIBSOEM), UInt8, (Ptr{ECError},), err) != 0
        e = err[]
        if e.etype == EC_ERR_TYPE_SDO_ERROR && e.slave == slave && e.index == index && e.subindex == subindex
            code = reinterpret(UInt32, e.abort_code)
        end
    end
    return code
end

_check_mailbox(b::EtherCATBackend) =
    b.initialized || throw(ArgumentError("EtherCAT backend is not initialized"))

"""
    sdo_write!(backend, slave, index, subindex, value; timeout_us=700_000)

Write `value` (an `Integer` of the object's exact width, e.g. `UInt16(…)`, or a
`Float32`) to object `index:subindex` of `slave` (1-based) over CoE. Throws an
[`SDOError`](@ref) with the slave's abort code on failure. Not real-time: use it
in `preop_config` or between runs, not inside the control loop.
"""
function sdo_write!(b::EtherCATBackend, slave::Integer, index::Integer, subindex::Integer,
                    value::Union{Integer,Float32}; timeout_us::Integer=EC_TIMEOUTRXM)
    _check_mailbox(b)
    isbitstype(typeof(value)) || throw(ArgumentError("value must be a fixed-width number"))
    ref = Ref(value)
    wkc = ccall((:ec_SDOwrite, LIBSOEM), Cint, (UInt16, UInt16, UInt8, UInt8, Cint, Ptr{Cvoid}, Cint),
                slave, index, subindex, 0x00, sizeof(value), ref, timeout_us)
    wkc > 0 || throw(SDOError(slave, index, subindex, _sdo_abort_code(slave, index, subindex)))
    return value
end

"""
    sdo_read(backend, slave, index, subindex, T; timeout_us=700_000) -> T

Read object `index:subindex` of `slave` as type `T` over CoE. Throws an
[`SDOError`](@ref) on failure.
"""
function sdo_read(b::EtherCATBackend, slave::Integer, index::Integer, subindex::Integer,
                  ::Type{T}; timeout_us::Integer=EC_TIMEOUTRXM) where {T<:Union{Integer,Float32}}
    _check_mailbox(b)
    ref = Ref(zero(T))
    size = Ref(Cint(sizeof(T)))
    wkc = ccall((:ec_SDOread, LIBSOEM), Cint, (UInt16, UInt16, UInt8, UInt8, Ptr{Cint}, Ptr{Cvoid}, Cint),
                slave, index, subindex, 0x00, size, ref, timeout_us)
    wkc > 0 || throw(SDOError(slave, index, subindex, _sdo_abort_code(slave, index, subindex)))
    return ref[]
end

"""
    PDOEntry(index, subindex, bits)

One object mapped into a PDO, e.g. `PDOEntry(0x6040, 0, 16)` for the controlword.
Use `index = 0` for padding.
"""
struct PDOEntry
    index::UInt16
    subindex::UInt8
    bits::UInt8
end
PDOEntry(index::Integer, subindex::Integer, bits::Integer) = PDOEntry(UInt16(index), UInt8(subindex), UInt8(bits))
_mapping_word(e::PDOEntry) = (UInt32(e.index) << 16) | (UInt32(e.subindex) << 8) | UInt32(e.bits)

"""
    configure_pdo!(backend, slave, assign_index, pdo_index, entries)

Program a PDO mapping over CoE with the standard sequence: clear the assignment
`assign_index` (`0x1C12` outputs / `0x1C13` inputs), clear `pdo_index`
(`0x1600…` / `0x1A00…`), write the `entries`, set their count, then assign the
PDO. Call it from `preop_config`. Returns the PDO size in bytes.
"""
function configure_pdo!(b::EtherCATBackend, slave::Integer, assign_index::Integer, pdo_index::Integer,
                        entries::AbstractVector{PDOEntry})
    length(entries) <= 254 || throw(ArgumentError("too many PDO entries"))
    sdo_write!(b, slave, assign_index, 0, UInt8(0))
    sdo_write!(b, slave, pdo_index, 0, UInt8(0))
    for (k, e) in enumerate(entries)
        sdo_write!(b, slave, pdo_index, k, _mapping_word(e))
    end
    sdo_write!(b, slave, pdo_index, 0, UInt8(length(entries)))
    sdo_write!(b, slave, assign_index, 1, UInt16(pdo_index))
    sdo_write!(b, slave, assign_index, 0, UInt8(1))
    return sum(e -> Int(e.bits), entries; init=0) ÷ 8
end

export dc_time, dc_phase_error_ns, SDOError, sdo_write!, sdo_read, PDOEntry, configure_pdo!
