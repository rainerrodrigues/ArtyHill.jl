# ArtyHill.jl

[![CI](https://github.com/rainerrodrigues/ArtyHill.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/rainerrodrigues/ArtyHill.jl/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://rainerrodrigues.github.io/ArtyHill.jl/dev/)
[![codecov](https://codecov.io/gh/rainerrodrigues/ArtyHill.jl/graph/badge.svg)](https://codecov.io/gh/rainerrodrigues/ArtyHill.jl)
[![Aqua QA](https://raw.githubusercontent.com/JuliaTesting/Aqua.jl/master/badge.svg)](https://github.com/JuliaTesting/Aqua.jl)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Hardware-in-the-Loop (HIL) and real-time embedded prototyping in Julia. One controller, unchanged,
runs against a pure simulation, SocketCAN / EtherCAT hardware, or Verilog/RTL in a Verilator
co-simulation, and finally ships as a C-callable library.

| Layer | What's in it |
| --- | --- |
| **Hardware abstraction** | `SimulationBackend` (`DiscreteLinearPlant`, `NonlinearPlant`) · `SocketCANBackend` (classic + CAN FD, kernel filters, heartbeat) · `EtherCATBackend` (SOEM 1.4: CoE/SDO drive configuration, DC Sync0 with loop phase lock) with `CiA402Codec` · `verijul_backend` (RTL co-simulation) |
| **Deterministic runtime** | `ControlWorkspace` · `control_step!` · `run_realtime_loop!` (`mlockall`, CPU pinning, `SCHED_FIFO`, GC pause, `clock_nanosleep(TIMER_ABSTIME)`, latency histograms) · `spawn_realtime_loop` on a dedicated thread · `SPSCRing` lock-free telemetry |
| **Safety** | `LimitedController` (clamp, rate limit, NaN guard) · watchdog on consecutive overruns/errors · `safe_command` on every exit · `heartbeat!` hook |
| **Verification & codegen** | `check_step_allocs` / `check_loop_allocs` (AllocCheck) · `compile_controller` (StaticCompiler, libc-only `.so`) · `juliac_build` (Julia 1.12 `juliac`) · `clean_telemetry` (KaosCleaner) |

## Quick start

```julia
using ArtyHill, AllocCheck

const DT = 1e-3
plant   = DiscreteLinearPlant([1 DT; 0 1], reshape([DT^2/2, DT], 2, 1), [1.0 0; 0 1], [1.0, 0])
backend = SimulationBackend(plant)    # later: SocketCANBackend("can0"; tx_base_id=0x100, rx_base_id=0x200)
ws      = ControlWorkspace{2,1,2}()   # N states, M commands, P measurements
ctrl    = LimitedController(StaticFeedback([40.0 12.0]), -5.0, 5.0; max_rate=1e3, dt=DT)

@assert isempty(check_loop_allocs(ws, ctrl, backend))   # static proof: zero heap allocations

ring = SPSCRing(ws, 1 << 14)
init_backend!(backend)
stats = run_realtime_loop!(ws, ctrl, backend, 1_000_000, 10_000;
                           telemetry=ring, priority=80, cpu=3,
                           max_consecutive_overruns=5, safe_command=[0.0])
close_backend!(backend)
latency_percentile(stats, 0.999; kind=:wake)            # jitter, ns
```

Examples: [cycloidal_drive.jl](examples/cycloidal_drive.jl) (reducer geometry → simulation →
identification from telemetry → controller sweep; see the [walkthrough](docs/src/examples.md)),
[sim_hil.jl](examples/sim_hil.jl) (1 kHz run with a telemetry consumer),
[codegen_controller.jl](examples/codegen_controller.jl) (StaticCompiler → `pd_controller.so`, libc only),
[juliac_controller.jl](examples/juliac_controller.jl) (juliac → C library wrapping a `LimitedController`).

## Optional extensions

| Load | You get |
| --- | --- |
| `using AllocCheck` | `check_step_allocs`, `check_loop_allocs` |
| `using StaticCompiler, StaticTools` | `compile_controller` |
| `using JuliaC` (Julia ≥ 1.12) | `juliac_build` |
| `using DiscretePIDs` | `pid_bank`: DiscretePIDs.jl controllers as one allocation-free controller |
| `using VeriJul` | `verijul_backend`: a Verilated design as the plant / I/O hardware |
| `using KaosCleaner` (Julia ≥ 1.12, [install from GitHub](https://github.com/rainerrodrigues/KaosCleaner)) | `clean_telemetry`: Hampel outlier removal and resampling of telemetry logs |

## Testing

```bash
julia --threads=2 --project -e 'using Pkg; Pkg.test()'
```

Real SocketCAN I/O on a virtual bus (CAN FD needs mtu 72):

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
ARTYHILL_VCAN=vcan0 julia --threads=2 --project -e 'using Pkg; Pkg.test()'
```

EtherCAT end to end, with no hardware and no root: the real SOEM 1.4 master (downloaded and built once)
talks over a veth pair to [an emulator](test/ethercat/ecat_emu.c) of CiA 402 drives, inside an
unprivileged network namespace:

```bash
test/ethercat/run.sh
```

First bring-up of a real CiA 402 drive (asks before enabling and before moving):

```bash
COUNTS_PER_REV=131072 bench/rtrun.sh julia --threads=2 --project=bench bench/drive_bench.jl eth1
```

Latency against `cyclictest`, on a PREEMPT_RT machine:

```bash
bench/rtrun.sh ./bench/compare_cyclictest.sh 1000 600000 3 80
```

## Real-time setup checklist

- PREEMPT_RT kernel; isolate a core (`isolcpus=3 nohz_full=3 rcu_nocbs=3`) and pass `cpu=3`.
- Root, or `CAP_SYS_NICE` + `CAP_IPC_LOCK` with `ulimit -r 99` and `ulimit -l unlimited`.
  Julia reserves a lot of virtual memory, so `mlockall` fails with `ENOMEM` under the default memlock limit.
- `strict=true` turns a failed setup step into an error instead of a warning (soft mode).
- Run the loop on its own thread (`--threads=2`) and keep other threads allocation-light: the GC is paused process-wide.
- Warm up (call `control_step!` a few times) before starting the loop.
- EtherCAT: SOEM 1.4 as a shared library. Point ArtyHill at it with `set_libsoem!(path)` and restart Julia.
  Raw sockets need `CAP_NET_RAW`.

## Status

- Tests pass on Julia 1.10, 1.11 and 1.12, with Aqua quality checks and JET static analysis in CI.
- `SocketCANBackend` runs real socket I/O on `vcan0` in CI (classic CAN, CAN FD, heartbeat).
- `EtherCATBackend` + `CiA402Codec` run in CI against the real SOEM 1.4 master and emulated drives:
  - scan and process-image layout;
  - CoE SDO configuration with real abort codes, and PDO remapping;
  - drives that refuse OP without DC Sync0, and loop phase-locking to the DC clock;
  - 1 kHz CSP motion, bump-free enable, fault latch and reset, and a cable-pull watchdog.
- The VeriJul backend runs against real Verilator-compiled RTL in CI, as well as a C stand-in.
- **Not yet run on physical hardware.** The emulator follows the EtherCAT and CiA 402 specs, but real drives
  add vendor quirks, segmented SDO transfers and stricter DC timing. Use a bench drive with hardware
  interlocks before connecting anything that moves; `bench/drive_bench.jl` walks through a first bring-up.
- **No PREEMPT_RT numbers yet.** `bench/compare_cyclictest.sh` is ready but has only run on a stock kernel.

See the [documentation](https://rainerrodrigues.github.io/ArtyHill.jl/dev/) for guides on backends, the
real-time loop, safety, code generation and the cycloidal-drive example, and step-by-step tutorials for
[bringing up a real servo drive](https://rainerrodrigues.github.io/ArtyHill.jl/dev/tutorials/drive_bringup/) and
[measuring PREEMPT_RT latency](https://rainerrodrigues.github.io/ArtyHill.jl/dev/tutorials/preempt_rt/).
