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

A fault on any axis latches the enable flag off. Clear it with
[`cia402_fault_reset!`](@ref), which pulses controlword bit 7 until the drive
leaves *Fault*, then re-enable explicitly. Nothing re-energizes on its own.

### Configuring drives over CoE

Most servo drives expect their PDO mapping and parameters to be written over
the mailbox (SDO) in PRE-OP. Pass a `preop_config(backend, slave)` function; it
runs for every slave before the process image is mapped, and SOEM then reads
the mapping you programmed:

```julia
rx = [PDOEntry(0x6040, 0, 16), PDOEntry(0x607A, 0, 32)]                 # controlword, target position
tx = [PDOEntry(0x6041, 0, 16), PDOEntry(0x6064, 0, 32), PDOEntry(0x606C, 0, 32)]

function preop!(b, slave)
    sdo_write!(b, slave, 0x6060, 0, Int8(8))            # cyclic synchronous position
    sdo_write!(b, slave, 0x6065, 0, UInt32(300_000))    # following-error window
    configure_pdo!(b, slave, 0x1C12, 0x1600, rx)
    configure_pdo!(b, slave, 0x1C13, 0x1A00, tx)
end

layout = CiA402Layout(rx, tx)                           # offsets derived from the same mapping
codec = CiA402Codec{2}(layout; position_scale=10_000 / 2π)
b = EtherCATBackend("eth0"; output_bytes=2layout.out_stride, codec, preop_config=preop!,
                    dc_cycle_ns=1_000_000)
```

[`sdo_read`](@ref) / [`sdo_write!`](@ref) throw an [`SDOError`](@ref) carrying the
drive's CoE abort code and its meaning (for example "attempt to write a
read-only object"). They work in OP too, but not inside the control loop.

### Distributed clocks

With `dc_cycle_ns > 0`, `init_backend!` starts Sync0 on every slave, which most
drives require before they accept OP in a cyclic synchronous mode. With
`dc_align` (default on), [`run_realtime_loop!`](@ref) phase-locks its wake-ups to
the DC reference clock through [`cycle_correction_ns`](@ref), using a PI law on the
frame phase (the same as SOEM's `ec_sync` example). Frames then reach the drives
at `dc_frame_phase_ns` within each cycle. Use the same `period_ns` as `dc_cycle_ns`,
and watch [`dc_phase_error_ns`](@ref) and [`dc_time`](@ref).

### Testing without hardware

`test/ethercat/run.sh` builds SOEM 1.4 and runs the unmodified master against
`test/ethercat/ecat_emu.c`, a frame-level emulator of a line of CiA 402 drives,
over a veth pair in an unprivileged network namespace. It covers:
- the slave scan (SII EEPROM) and FMMU/SM configuration;
- CoE SDO reads, writes and aborts, and PDO remapping;
- DC Sync0, including drives that refuse OP without it, and phase locking;
- the transition to OP, CSP motion, faults, and a simulated cable pull.

## Integration tests on a virtual bus

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
ARTYHILL_VCAN=vcan0 julia --project -e 'using Pkg; Pkg.test()'
```
