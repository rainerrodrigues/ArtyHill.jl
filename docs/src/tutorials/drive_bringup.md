# Tutorial: bring up a real servo drive

This tutorial takes a CiA 402 servo drive on EtherCAT from "cable plugged in"
to "moving under ArtyHill's real-time loop", one stage at a time. It uses
`bench/drive_bench.jl`, which stops and asks before it enables the drive and
again before it moves it.

!!! warning "The motor will move"
    Before you start: clamp the motor to the bench with **no load coupled**,
    wire the drive's STO (Safe Torque Off) input to a hardware E-stop within
    reach, and keep the defaults (an 18° move limited to 0.5 rev/s) for the
    first run. Software limits are not a substitute for a hardware interlock.

## What you need

- A **native Linux machine**. WSL2, containers on macOS and most VMs cannot
  pass raw Ethernet frames reliably. A small x86 PC with an Intel i210/i211
  NIC is ideal; a Raspberry Pi 5 works too.
- A **dedicated Ethernet port** for EtherCAT, cabled directly to the drive's
  IN port (no switch).
- A **CiA 402 drive** that supports cyclic synchronous position mode (CSP) and
  distributed clocks, already tuned with the vendor's tool so that it holds
  position stably on its own.
- The drive's **encoder resolution** (counts per motor revolution), from its
  manual or object `0x608F`.
- Julia ≥ 1.10, a C compiler, `curl`, and sudo rights.

## 1. Prepare the network port

Find the port's name with `ip link`. Here it is `eth1`. Stop NetworkManager
from managing it (it would try DHCP and send frames of its own), bring it up,
and switch off interrupt coalescing so frames are not held back:

```bash
sudo nmcli device set eth1 managed no
sudo ip link set eth1 up
sudo ethtool -C eth1 rx-usecs 0
```

## 2. Build SOEM and point ArtyHill at it

ArtyHill talks to the drives through SOEM 1.4 as a shared library. The
emulator test script downloads and builds it for you:

```bash
test/ethercat/run.sh      # builds ~/.cache/artyhill-soem/libsoem.so, then runs the emulator tests
```

Then set the library for the `bench` environment, which is saved as a
preference ([`set_libsoem!`](@ref)):

```bash
julia --project=bench -e 'using Pkg; Pkg.instantiate()'
julia --project=bench -e 'using ArtyHill; set_libsoem!(expanduser("~/.cache/artyhill-soem/libsoem.so"))'
```

## 3. Get the privileges

Raw EtherCAT frames need `CAP_NET_RAW`; the real-time loop needs
`CAP_SYS_NICE` (for `SCHED_FIFO`) and `CAP_IPC_LOCK` (for `mlockall`).
`bench/rtrun.sh` runs any command as *you* with exactly those three
capabilities, through a short-lived systemd unit:

```bash
bench/rtrun.sh julia --version
```

Prefer it over the alternatives: `setcap` on the julia binary puts Julia in
secure-execution mode, where the loader may no longer find `libjulia`, and
`sudo julia` uses root's package depot and leaves root-owned files behind.

## 4. Run the bring-up

```bash
COUNTS_PER_REV=131072 bench/rtrun.sh julia --threads=2 --project=bench bench/drive_bench.jl eth1
```

The script walks through five stages. Here is what each one does, and what to
check before you answer "y" to the next.

**Stage 1: scan, configure, operational.** SOEM scans the line. In PRE-OP,
for every drive, the script reads its identity (`0x1018`) and supported modes
(`0x6502`), sets CSP mode (`0x6060 = 8`) over CoE ([`sdo_write!`](@ref)) and
programs a minimal PDO mapping ([`configure_pdo!`](@ref)):

| Direction | Objects |
|-----------|---------|
| outputs (RxPDO `0x1600`) | controlword `0x6040`, target position `0x607A` |
| inputs (TxPDO `0x1A00`) | statusword `0x6041`, position `0x6064`, velocity `0x606C` |

Then DC Sync0 is started at the loop period and the line goes to OP.

```text
  slave 1: vendor 0x..., product 0x..., supported modes 0x...
  OP reached: 1 slave(s), expected working counter 3
```

If this fails, see [Troubleshooting](@ref drive-troubleshooting) below.

**Stage 2: read positions, no torque.** For one second the drive stays in
*Ready to switch on* while positions are read. Turn the shaft by hand by a
known amount (if that is safe) and run again: the reported position should
change by that fraction of a revolution, with the sign you expect. If it is
off by a factor, `COUNTS_PER_REV` is wrong.

**Stage 3: enable and hold.** After you confirm, [`cia402_enable!`](@ref)
lets the CiA 402 state machine walk the drive to *Operation enabled*. In CSP,
the target is the actual position until the moment of enabling, so the motor
should energize **without any jump**. A clunk here means the drive is
interpreting the target differently (units, or a position offset): stop and
check before going further.

**Stage 4: move.** After you confirm, the real-time loop runs at 1 kHz
(`SCHED_FIFO` priority 80, memory locked, GC paused) and commands one smooth
out-and-back move, `MOVE_REV` revolutions over `MOVE_SECONDS`, starting and
ending at zero speed. A [`LimitedController`](@ref) caps the commanded speed at
`MAX_RPS` whatever the profile says. The watchdog stops the loop after 3
consecutive communication errors or 5 consecutive overruns.

**Stage 5: disable and close.** Always runs, even after an error in stages 2–4:
the drive is held, disabled (no torque) and the line closed.

## 5. Read the results

```text
LoopStats: 4500 iterations, 0 overruns, 0 errors
  wake-up jitter mean ..., p99 ≤ ..., p99.9 ≤ ..., max ...
  step latency:   mean ..., p99 ≤ ..., p99.9 ≤ ..., max ...
  mlockall=true SCHED_FIFO=true pinned=false gc_off=true safe_cmd_sent=false
  abort reason: none, DC phase error now: ... ns
  max following error: ...° (... counts) over 4500 cycles
  telemetry → bench/drive_telemetry.csv
```

- **`mlockall=true SCHED_FIFO=true`**: the privileges worked. If either is
  `false`, the run was in soft real-time mode and the timing numbers mean
  little.
- **Overruns and errors** should both be 0. Errors are cycles where the
  working counter was short, meaning a frame was lost or a drive did not answer.
- **Wake-up jitter** is how late the loop woke up. On a stock kernel, expect
  occasional spikes of hundreds of µs; on PREEMPT_RT with an isolated CPU, a
  few tens of µs at worst (see [Tutorial: PREEMPT_RT latency](@ref)).
- **DC phase error** is how far frames land from the target point in the
  Sync0 cycle ([`dc_phase_error_ns`](@ref)). It should settle well below the
  period; a value that keeps growing means the loop is not locked.
- **Following error** is the largest gap between commanded and actual position.
  It reflects the drive's own tuning as much as ArtyHill.
- **`drive_telemetry.csv`** has one row per cycle: timestamps, jitter, latency,
  status, the three readings per axis (`y1…`: position, velocity, torque in
  radians, rad/s and raw units) and the commands (`u1…`). Plot `u1` against
  `y1` to see the tracking.

## 6. Next steps

Raise `MOVE_REV`, then `MAX_RPS`, a little at a time, and couple the load only
after several clean runs. Pin the loop to an isolated CPU with `CPU=3` once
the [PREEMPT_RT setup](@ref "Tutorial: PREEMPT_RT latency") is in place.
To run your own controller, copy the script and replace `OutAndBack` with any
callable `(u, x, y) -> nothing`: `y` holds `[pos₁, vel₁, torque₁, pos₂, …]`
and `u` the target per axis.

### Settings

All are environment variables, so pass them before `bench/rtrun.sh`.

| Variable | Default | Meaning |
|----------|---------|---------|
| `AXES` | 1 | drives on the line; all get the same move |
| `COUNTS_PER_REV` | 131072 | encoder counts per revolution |
| `MOVE_REV` | 0.05 | move amplitude, revolutions |
| `MOVE_SECONDS` | 4 | duration of the out-and-back move |
| `MAX_RPS` | 0.5 | hard limit on commanded speed, rev/s |
| `CYCLE_US` | 1000 | loop and DC Sync0 period, µs |
| `CPU` | -1 | pin the loop to this CPU (-1: no pinning) |
| `CONFIGURE_PDO` | 1 | program the PDO mapping; 0 for fixed-PDO drives |

## [Troubleshooting](@id drive-troubleshooting)

**`init_backend!` fails, or 0 slaves found.** Check the interface name, that
the link LED is on, that the cable goes into the drive's IN port, and that
nothing else (NetworkManager, a second master) uses the port. Run with
`bench/rtrun.sh`, since raw sockets need `CAP_NET_RAW`.

**`SDOError` with an abort code in stage 1.** The message names the object
and the reason. `0x06020000` (object does not exist) on `0x6502` is harmless.
On `0x1C12`/`0x1600` it usually means the drive has a **fixed PDO mapping**:
look up the mapping in the drive's ESI file or manual, edit `RX_MAP` and
`TX_MAP` at the top of the script to list exactly those objects in that order,
and run with `CONFIGURE_PDO=0`. The codec layout is derived from the same
lists, so it always matches.

**The drive never reaches OP.** Many drives refuse OP in CSP without DC Sync0
at a period they support; try `CYCLE_US=2000` or `4000`. Check the drive's
error display or its error code object (`0x603F`).

**Not operation_enabled in stage 3.** The drive is in *Fault* or STO is
active. Release the E-stop, clear the cause shown by the drive, and rerun.
From your own code, [`cia402_fault_reset!`](@ref) followed by
[`cia402_enable!`](@ref) does the same.

**Errors during the move.** A lost frame now and then points to the NIC or
interrupt coalescing (step 1); bursts of overruns point to the scheduler (run
the [latency tutorial](@ref "Tutorial: PREEMPT_RT latency") first).
