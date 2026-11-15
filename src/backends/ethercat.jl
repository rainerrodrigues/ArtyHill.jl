# EtherCAT backend on top of SOEM (Simple Open EtherCAT Master, legacy `ec_*` API
# of SOEM 1.4). SOEM is resolved lazily on first use, so ArtyHill has no hard
# binary dependency and loads fine on machines without it.

const EC_STATE_INIT        = UInt16(0x01)
const EC_STATE_SAFE_OP     = UInt16(0x04)
const EC_STATE_OPERATIONAL = UInt16(0x08)
const EC_TIMEOUTRET        = Cint(2_000)       # µs
const EC_TIMEOUTSTATE      = Cint(2_000_000)   # µs

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

Library name used for SOEM calls. It is a compile-time constant so that hot-path
`ccall`s bind statically (no indirect function-pointer call, which also lets
AllocCheck prove the step allocation-free). Put SOEM 1.4 built as a shared
library (`-DBUILD_SHARED_LIBS=ON`) on the loader path as `libsoem.so`
(`ldconfig` or `LD_LIBRARY_PATH`).
"""
const LIBSOEM = "libsoem"

"""
    EtherCATBackend(iface; output_bytes, iomap_size=4096, codec=PackedFloat64Codec())

EtherCAT master using SOEM ([`LIBSOEM`](@ref)). `output_bytes` is the size of
the output part of the process image (SOEM maps outputs first, then inputs).
One control cycle is: `read_sensors!` receives the frame sent in the previous
cycle, `write_actuators!` encodes commands and sends the next frame — the
standard one-cycle pipeline. The expected working counter is taken from the
last exchange during the transition to OPERATIONAL.

Raw-socket access requires root or `CAP_NET_RAW` (`sudo setcap cap_net_raw+ep julia`).
"""
mutable struct EtherCATBackend{C} <: AbstractHILBackend
    const iface::NTuple{16,UInt8}
    const iomap::Vector{UInt8}
    const output_bytes::Int
    const codec::C
    ec_slave::Ptr{UInt16}       # &ec_slave[0]; `state` is the struct's first field
    initialized::Bool
    slave_count::Int32
    expected_wkc::Int32
    last_wkc::Int32
end

function EtherCATBackend(iface::AbstractString; output_bytes::Integer, iomap_size::Integer=4096,
                         codec=PackedFloat64Codec())
    return EtherCATBackend(fixed_cstring(iface, Val(16)), zeros(UInt8, iomap_size),
                           Int(output_bytes), codec, Ptr{UInt16}(C_NULL), false,
                           Int32(0), Int32(0), Int32(0))
end

@inline _ec_send() = ccall((:ec_send_processdata, LIBSOEM), Cint, ())
@inline _ec_receive(timeout) = ccall((:ec_receive_processdata, LIBSOEM), Cint, (Cint,), timeout)
_ec_statecheck(state, timeout) =
    ccall((:ec_statecheck, LIBSOEM), UInt16, (UInt16, UInt16, Cint), 0, state, timeout)
_ec_writestate() = ccall((:ec_writestate, LIBSOEM), Cint, (UInt16,), 0)

function init_backend!(b::EtherCATBackend)
    b.initialized && return HIL_OK
    h = Libdl.dlopen(LIBSOEM; throw_error=false)
    h === nothing && return HIL_ERR_SYS
    slave_sym = Libdl.dlsym(h, :ec_slave; throw_error=false)
    slave_sym === nothing && return HIL_ERR_SYS
    b.ec_slave = Ptr{UInt16}(slave_sym)

    name = Ref(b.iface)
    ccall((:ec_init, LIBSOEM), Cint, (Ptr{NTuple{16,UInt8}},), name) > 0 || return HIL_ERR_SYS
    b.initialized = true
    b.slave_count = ccall((:ec_config_init, LIBSOEM), Cint, (UInt8,), 0x00)
    b.slave_count > 0 || (close_backend!(b); return HIL_ERR_IO)

    used = ccall((:ec_config_map, LIBSOEM), Cint, (Ptr{UInt8},), b.iomap)
    (used <= length(b.iomap) && b.output_bytes <= used) || (close_backend!(b); return HIL_ERR_SYS)
    ccall((:ec_configdc, LIBSOEM), UInt8, ())
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
