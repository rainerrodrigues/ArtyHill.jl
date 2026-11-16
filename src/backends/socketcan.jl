# Linux SocketCAN backend (raw CAN 2.0 and CAN FD frames) via direct libc ccalls.

const AF_CAN             = Cint(29)
const SOCK_RAW           = Cint(3)
const SOCK_NONBLOCK      = Cint(0o4000)
const CAN_RAW            = Cint(1)
const SOL_CAN_RAW        = Cint(101)
const CAN_RAW_FILTER     = Cint(1)
const CAN_RAW_FD_FRAMES  = Cint(5)
const CAN_EFF_FLAG       = 0x80000000
const CAN_RTR_FLAG       = 0x40000000
const CAN_SFF_MASK       = 0x000007ff
const CAN_EFF_MASK       = 0x1fffffff
const CANFD_MAX_DLEN     = 64

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

"Linux `struct canfd_frame` (72 bytes)."
struct CanFDFrame
    can_id::UInt32
    len::UInt8
    flags::UInt8
    __res0::UInt8
    __res1::UInt8
    data::NTuple{64,UInt8}
end
CanFDFrame() = CanFDFrame(0, 0, 0, 0, 0, ntuple(_ -> 0x00, Val(64)))

"Linux `struct can_filter`."
struct CanFilter
    can_id::UInt32
    can_mask::UInt32
end

"Linux `struct sockaddr_can` (24 bytes on current kernels)."
struct SockaddrCan
    can_family::UInt16
    __pad::UInt16
    can_ifindex::Cint
    can_addr::NTuple{16,UInt8}
end

"""
    Float64Codec()

Default CAN payload codec: one native-endian `Float64` per 8-byte slot.
Define your own codec type with `encode_can(codec, x)::NTuple{8,UInt8}` and
`decode_can(codec, data::NTuple{8,UInt8}, dlc::UInt8)` for real drive protocols.
"""
struct Float64Codec end
@inline encode_can(::Float64Codec, x) = reinterpret(NTuple{8,UInt8}, Float64(x))
@inline decode_can(::Float64Codec, data::NTuple{8,UInt8}, ::UInt8) = reinterpret(Float64, data)

"""
    SocketCANBackend(iface; tx_base_id, rx_base_id, codec=Float64Codec(), extended=false,
                     rx_count=0, canfd=false, heartbeat_id=0, heartbeat_every=100)

Raw SocketCAN backend.

**Classic CAN** (default): actuator `u[i]` is sent as one frame with id
`tx_base_id + i - 1`; sensor `y[i]` is updated from any received frame with id
`rx_base_id + i - 1`.

**CAN FD** (`canfd=true`, interface must be FD-capable): all commands go out in
*one* 64-byte frame with id `tx_base_id` (8-byte slot per value, so up to 8
values, updated atomically), and sensors are read from FD frames with id
`rx_base_id` the same way.

In both modes the last value is held when no new frame arrived, and the socket
is non-blocking: `read_sensors!` drains at most `4 * length(y)` frames per
cycle so its worst-case run time stays bounded.

- `rx_count > 0` installs kernel receive filters for exactly the sensor ids, so
  unrelated bus traffic never reaches Julia (recommended on busy buses).
- `heartbeat_id != 0` sends a 1-byte CANopen-style heartbeat (`0x05`,
  operational) with that id every `heartbeat_every` cycles from
  [`heartbeat!`](@ref), for drives that cut power when the master goes silent.

Bring the interface up first, e.g. `sudo ip link set can0 up type can bitrate 1000000`
(add `dbitrate 5000000 fd on` for CAN FD) or, for testing,
`sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up`.
"""
mutable struct SocketCANBackend{C} <: AbstractHILBackend
    const iface::NTuple{16,UInt8}
    const tx_base_id::UInt32
    const rx_base_id::UInt32
    const extended::Bool
    const canfd::Bool
    const rx_count::Int
    const heartbeat_id::UInt32
    const heartbeat_every::Int
    const codec::C
    const frame::Base.RefValue{CanFrame}       # preallocated I/O buffers → zero allocs
    const fdframe::Base.RefValue{CanFDFrame}
    socket_fd::Cint
end

function SocketCANBackend(iface::AbstractString; tx_base_id::Integer, rx_base_id::Integer,
                          codec=Float64Codec(), extended::Bool=false, rx_count::Integer=0,
                          canfd::Bool=false, heartbeat_id::Integer=0, heartbeat_every::Integer=100)
    heartbeat_every > 0 || throw(ArgumentError("heartbeat_every must be positive"))
    return SocketCANBackend(fixed_cstring(iface, Val(16)), UInt32(tx_base_id), UInt32(rx_base_id),
                            extended, canfd, Int(rx_count), UInt32(heartbeat_id), Int(heartbeat_every),
                            codec, Ref(CanFrame()), Ref(CanFDFrame()), Cint(-1))
end

@inline _id_mask(b::SocketCANBackend) = b.extended ? CAN_EFF_MASK : CAN_SFF_MASK

@inline function _can_id(b::SocketCANBackend, base::UInt32, i)
    id = base + UInt32(i - 1)
    return b.extended ? (id & CAN_EFF_MASK) | CAN_EFF_FLAG : id & CAN_SFF_MASK
end

"Kernel filters matching exactly the sensor ids of `b` (empty when `rx_count == 0`)."
function can_filters(b::SocketCANBackend)
    n = b.canfd ? min(b.rx_count, 1) : b.rx_count
    mask = b.extended ? (CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG) : (CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG)
    return [CanFilter(_can_id(b, b.rx_base_id, i), mask) for i in 1:n]
end

function _setsockopt(fd, level, opt, ref, len)
    return ccall(:setsockopt, Cint, (Cint, Cint, Cint, Ptr{Cvoid}, Cuint), fd, level, opt, ref, len) == 0
end

function init_backend!(b::SocketCANBackend)
    Sys.islinux() || return HIL_ERR_SYS
    b.socket_fd >= 0 && return HIL_OK
    fd = ccall(:socket, Cint, (Cint, Cint, Cint), AF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)
    fd < 0 && return HIL_ERR_SYS
    fail() = (ccall(:close, Cint, (Cint,), fd); HIL_ERR_SYS)

    if b.canfd
        _setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, Ref(Cint(1)), sizeof(Cint)) || return fail()
    end
    filters = can_filters(b)
    if !isempty(filters)
        _setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) || return fail()
    end
    name = Ref(b.iface)
    ifindex = ccall(:if_nametoindex, Cuint, (Ptr{NTuple{16,UInt8}},), name)
    ifindex == 0 && return fail()
    addr = Ref(SockaddrCan(UInt16(AF_CAN), 0, Cint(ifindex), ntuple(_ -> 0x00, Val(16))))
    ccall(:bind, Cint, (Cint, Ptr{SockaddrCan}, Cuint), fd, addr, sizeof(SockaddrCan)) == 0 || return fail()
    b.socket_fd = fd
    return HIL_OK
end

function close_backend!(b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_OK
    rc = ccall(:close, Cint, (Cint,), b.socket_fd)
    b.socket_fd = Cint(-1)
    return rc == 0 ? HIL_OK : HIL_ERR_SYS
end

@inline function _write_classic!(b::SocketCANBackend, id::UInt32, dlc::UInt8, data::NTuple{8,UInt8})
    b.frame[] = CanFrame(id, dlc, 0, 0, 0, data)
    n = ccall(:write, Cssize_t, (Cint, Ptr{CanFrame}, Csize_t), b.socket_fd, b.frame, sizeof(CanFrame))
    return n == sizeof(CanFrame) ? HIL_OK : HIL_ERR_IO
end

@inline function write_actuators!(u, b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_ERR_NOT_INIT
    if b.canfd
        nslots = min(length(u), CANFD_MAX_DLEN ÷ 8)
        ref = b.fdframe
        GC.@preserve ref begin
            p = Base.unsafe_convert(Ptr{CanFDFrame}, ref)
            unsafe_store!(Ptr{UInt32}(p), _can_id(b, b.tx_base_id, 1))
            unsafe_store!(Ptr{UInt8}(p) + 4, UInt8(8 * nslots))
            unsafe_store!(Ptr{UInt8}(p) + 5, 0x00)
            data = Ptr{NTuple{8,UInt8}}(p + 8)
            @inbounds for k in 1:nslots
                unsafe_store!(data, encode_can(b.codec, u[firstindex(u) + k - 1]), k)
            end
        end
        n = ccall(:write, Cssize_t, (Cint, Ptr{CanFDFrame}, Csize_t), b.socket_fd, ref, sizeof(CanFDFrame))
        return n == sizeof(CanFDFrame) ? HIL_OK : HIL_ERR_IO
    end
    status = HIL_OK
    @inbounds for i in eachindex(u)
        st = _write_classic!(b, _can_id(b, b.tx_base_id, i - firstindex(u) + 1), 0x08, encode_can(b.codec, u[i]))
        st == HIL_OK || (status = st)
    end
    return status
end

@inline function read_sensors!(y, b::SocketCANBackend)
    b.socket_fd < 0 && return HIL_ERR_NOT_INIT
    n_y = length(y)
    id_mask = _id_mask(b)
    if b.canfd
        ref = b.fdframe
        for _ in 1:(4 * n_y)
            n = ccall(:read, Cssize_t, (Cint, Ptr{CanFDFrame}, Csize_t), b.socket_fd, ref, sizeof(CanFDFrame))
            n > 0 || break                                   # EAGAIN: queue drained
            f = ref[]
            (f.can_id & id_mask) == (b.rx_base_id & id_mask) || continue
            len = n == sizeof(CanFDFrame) ? Int(f.len) : Int(min(f.len, 0x08))  # classic frames: dlc ≤ 8
            nslots = min(n_y, len ÷ 8)
            GC.@preserve ref begin
                data = Ptr{NTuple{8,UInt8}}(Base.unsafe_convert(Ptr{CanFDFrame}, ref) + 8)
                @inbounds for k in 1:nslots
                    y[firstindex(y) + k - 1] = decode_can(b.codec, unsafe_load(data, k), 0x08)
                end
            end
        end
        return HIL_OK
    end
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

@inline function heartbeat!(b::SocketCANBackend, i::Integer)
    (b.heartbeat_id == 0 || i % b.heartbeat_every != 0) && return HIL_OK
    b.socket_fd < 0 && return HIL_ERR_NOT_INIT
    id = b.extended ? (b.heartbeat_id & CAN_EFF_MASK) | CAN_EFF_FLAG : b.heartbeat_id & CAN_SFF_MASK
    return _write_classic!(b, id, 0x01, (0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))
end
