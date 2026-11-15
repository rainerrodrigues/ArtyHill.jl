# Linux SocketCAN backend (raw CAN 2.0 frames) via direct libc ccalls.

const AF_CAN        = Cint(29)
const SOCK_RAW      = Cint(3)
const SOCK_NONBLOCK = Cint(0o4000)
const CAN_RAW       = Cint(1)
const CAN_EFF_FLAG  = 0x80000000
const CAN_EFF_MASK  = 0x1fffffff

"Linux `struct can_frame` (16 bytes)."
struct CanFrame
    can_id::UInt32
    can_dlc::UInt8
    __pad::UInt8
    __res0::UInt8
    __res1::UInt8
    data::NTuple{8,UInt8}
end
CanFrame() = CanFrame(0, 0, 0, 0, 0, ntuple(_ -> 0x00, Val(8)))

"Linux `struct sockaddr_can` (24 bytes on current kernels)."
struct SockaddrCan
    can_family::UInt16
    __pad::UInt16
    can_ifindex::Cint
    can_addr::NTuple{16,UInt8}
end

"""
    Float64Codec()

Default CAN payload codec: one native-endian `Float64` per 8-byte frame.
Define your own codec type with `encode_can(codec, x)::NTuple{8,UInt8}` and
`decode_can(codec, data::NTuple{8,UInt8}, dlc::UInt8)` for real drive protocols.
"""
struct Float64Codec end
@inline encode_can(::Float64Codec, x) = reinterpret(NTuple{8,UInt8}, Float64(x))
@inline decode_can(::Float64Codec, data::NTuple{8,UInt8}, ::UInt8) = reinterpret(Float64, data)

"""
    SocketCANBackend(iface; tx_base_id, rx_base_id, codec=Float64Codec(), extended=false)

Raw SocketCAN backend. Actuator `u[i]` is sent as one frame with id
`tx_base_id + i - 1`; sensor `y[i]` is updated from any received frame with id
`rx_base_id + i - 1` (the last value is held when no new frame arrived).

The socket is non-blocking: `read_sensors!` drains at most `4 * length(y)` frames
per cycle so its worst-case run time stays bounded.

Bring the interface up first, e.g. `sudo ip link set can0 up type can bitrate 1000000`
or, for testing, `sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up`.
"""
mutable struct SocketCANBackend{C} <: AbstractHILBackend
    const iface::NTuple{16,UInt8}
    const tx_base_id::UInt32
    const rx_base_id::UInt32
    const extended::Bool
    const codec::C
    const frame::Base.RefValue{CanFrame}   # preallocated I/O buffer → zero allocs
    socket_fd::Cint
end

function SocketCANBackend(iface::AbstractString; tx_base_id::Integer, rx_base_id::Integer,
                          codec=Float64Codec(), extended::Bool=false)
    return SocketCANBackend(fixed_cstring(iface, Val(16)), UInt32(tx_base_id), UInt32(rx_base_id),
                            extended, codec, Ref(CanFrame()), Cint(-1))
end

function init_backend!(b::SocketCANBackend)
    Sys.islinux() || return HIL_ERR_SYS
    fd = ccall(:socket, Cint, (Cint, Cint, Cint), AF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)
    fd < 0 && return HIL_ERR_SYS
    name = Ref(b.iface)
    ifindex = ccall(:if_nametoindex, Cuint, (Ptr{NTuple{16,UInt8}},), name)
    if ifindex == 0
        ccall(:close, Cint, (Cint,), fd)
        return HIL_ERR_SYS
    end
    addr = Ref(SockaddrCan(UInt16(AF_CAN), 0, Cint(ifindex), ntuple(_ -> 0x00, Val(16))))
    if ccall(:bind, Cint, (Cint, Ptr{SockaddrCan}, Cuint), fd, addr, sizeof(SockaddrCan)) != 0
        ccall(:close, Cint, (Cint,), fd)
        return HIL_ERR_SYS
    end
    b.socket_fd = fd
    return HIL_OK
end

function close_backend!(b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_OK
    rc = ccall(:close, Cint, (Cint,), b.socket_fd)
    b.socket_fd = Cint(-1)
    return rc == 0 ? HIL_OK : HIL_ERR_SYS
end

@inline function _can_id(b::SocketCANBackend, base::UInt32, i)
    id = base + UInt32(i - 1)
    return b.extended ? (id & CAN_EFF_MASK) | CAN_EFF_FLAG : id & 0x7ff
end

@inline function write_actuators!(u, b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_ERR_NOT_INIT
    status = HIL_OK
    @inbounds for i in eachindex(u)
        b.frame[] = CanFrame(_can_id(b, b.tx_base_id, i), 0x08, 0, 0, 0, encode_can(b.codec, u[i]))
        n = ccall(:write, Cssize_t, (Cint, Ptr{CanFrame}, Csize_t),
                  b.socket_fd, b.frame, sizeof(CanFrame))
        n == sizeof(CanFrame) || (status = HIL_ERR_IO)
    end
    return status
end

@inline function read_sensors!(y, b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_ERR_NOT_INIT
    n_y = length(y)
    id_mask = b.extended ? CAN_EFF_MASK : UInt32(0x7ff)
    for _ in 1:(4 * n_y)
        n = ccall(:read, Cssize_t, (Cint, Ptr{CanFrame}, Csize_t),
                  b.socket_fd, b.frame, sizeof(CanFrame))
        n == sizeof(CanFrame) || break          # EAGAIN: queue drained
        f = b.frame[]
        idx = Int((f.can_id & id_mask) - b.rx_base_id) + 1
        if 1 <= idx <= n_y
            @inbounds y[firstindex(y) + idx - 1] = decode_can(b.codec, f.data, f.can_dlc)
        end
    end
    return HIL_OK
end
