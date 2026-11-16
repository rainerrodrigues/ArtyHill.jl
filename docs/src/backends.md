# Backends

A backend implements four functions, each returning `HIL_OK` (0) or a negative
`HIL_ERR_*` code. Exceptions allocate, so keep them out of the hot path:

```julia
struct MyBackend <: AbstractHILBackend ... end
ArtyHill.init_backend!(b::MyBackend)
ArtyHill.read_sensors!(y, b::MyBackend)      # fill y in place, no allocations
ArtyHill.write_actuators!(u, b::MyBackend)   # no allocations
ArtyHill.close_backend!(b::MyBackend)
ArtyHill.heartbeat!(b::MyBackend, i)         # optional, called once per cycle
```

Verify the hot path with `check_step_allocs(ws, controller, backend)` (needs `using AllocCheck`).

## Simulation

[`SimulationBackend`](@ref) wraps an [`AbstractPlant`](@ref):
[`DiscreteLinearPlant`](@ref) for LTI models, or [`NonlinearPlant`](@ref) from your own
`f(x, u)` / `h(x)`, which is handy for gearbox backlash or torque ripple.

## SocketCAN

[`SocketCANBackend`](@ref) talks to Linux raw CAN sockets directly through libc.

- Classic CAN: one frame per value, consecutive ids from `tx_base_id` / `rx_base_id`.
- CAN FD (`canfd=true`): all commands in one 64-byte frame, updated atomically.
- `rx_count` installs kernel receive filters; `heartbeat_id` emits a liveness frame.
- Payloads go through a codec (`Float64Codec` by default). Implement
  `encode_can` / `decode_can` for your drive's protocol.

## EtherCAT

[`EtherCATBackend`](@ref) drives SOEM 1.4's `ec_*` API. Point it at your SOEM
build with [`set_libsoem!`](@ref). The default codec packs `Float64`s.
[`CiA402Codec`](@ref) speaks the CiA 402 servo profile: it decodes the statusword,
steps the drive's state machine towards *Operation enabled* (only after
[`cia402_enable!`](@ref)), scales targets and feedback, and in CSP mode keeps the
target on the actual position until the drive is enabled.

## Integration tests on a virtual bus

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
ARTYHILL_VCAN=vcan0 julia --project -e 'using Pkg; Pkg.test()'
```
