# ArtyHill.jl

Hardware-in-the-Loop (HIL) and real-time embedded prototyping in Julia: one
controller, unchanged, from pure simulation → SocketCAN / EtherCAT hardware →
a standalone `libjulia`-free shared library.

| Layer | What's in it |
| --- | --- |
| **1. Hardware abstraction** | `AbstractHILBackend` · `SimulationBackend` (`DiscreteLinearPlant`, `NonlinearPlant`) · `SocketCANBackend` (raw libc sockets) · `EtherCATBackend` (SOEM 1.4) |
| **2. Deterministic runtime** | `ControlWorkspace` (`MVector` buffers) · `control_step!` · `run_realtime_loop!` (`mlockall`, CPU pinning, `SCHED_FIFO`, GC pause, `clock_nanosleep(TIMER_ABSTIME)`) · `SPSCRing` lock-free telemetry |
| **3. Verification & codegen** | `check_step_allocs` / `check_loop_allocs` (AllocCheck extension) · `compile_controller` (StaticCompiler extension) · `write_telemetry_csv` |

## Quick start

```julia
using ArtyHill, AllocCheck

plant   = DiscreteLinearPlant([1 1e-3; 0 1], reshape([5e-7, 1e-3], 2, 1), [1.0 0; 0 1], [1.0, 0])
backend = SimulationBackend(plant)           # later: SocketCANBackend("can0"; tx_base_id=0x100, rx_base_id=0x200)
ws      = ControlWorkspace{2,1,2}()          # N states, M commands, P measurements
ctrl    = StaticFeedback([40.0 12.0])        # any callable ctrl(u, x_hat, y) works

@assert isempty(check_step_allocs(ws, ctrl, backend))   # static proof: zero heap allocations

ring = SPSCRing(ws, 1 << 14)
init_backend!(backend)
stats = run_realtime_loop!(ws, ctrl, backend, 1_000_000, 10_000; telemetry=ring, priority=80, cpu=3)
close_backend!(backend)
```

See [examples/sim_hil.jl](examples/sim_hil.jl) (1 kHz run plus a background telemetry consumer) and
[examples/codegen_controller.jl](examples/codegen_controller.jl) (compiles a PD law to
`pd_controller.so`, which depends only on libc).

## Writing a backend

```julia
struct MyBackend <: AbstractHILBackend ... end
ArtyHill.init_backend!(b::MyBackend)::Int32
ArtyHill.read_sensors!(y, b::MyBackend)::Int32      # fill y in place, no allocations
ArtyHill.write_actuators!(u, b::MyBackend)::Int32   # no allocations
ArtyHill.close_backend!(b::MyBackend)::Int32
```

Return `HIL_OK` (0) or a negative `HIL_ERR_*` code. Exceptions allocate, so keep them out of the hot path.
For real drive protocols, plug in your own payload codec (`encode_can`/`decode_can`, or
`encode_outputs!`/`decode_inputs!` for EtherCAT process images) instead of the default packed `Float64`.

## Real-time setup checklist

- PREEMPT_RT kernel; isolate a core (`isolcpus=3 nohz_full=3 rcu_nocbs=3`) and pass `cpu=3`.
- Privileges: root, or `CAP_SYS_NICE` + `CAP_IPC_LOCK` with `ulimit -r 99` and `ulimit -l unlimited`.
  Julia reserves a lot of virtual memory, so `mlockall` fails with `ENOMEM` under the default memlock limit.
- `RTConfig(strict=true)` turns any failed setup step into an error instead of a warning (soft mode).
- Run the loop on its own Julia thread (`--threads=2`) and keep other threads allocation-light: the GC is paused for the whole process.
- Warm up (call `control_step!` a few times) before starting the loop, so compilation never lands on a deadline.
- SocketCAN: `ip link set can0 up type can bitrate 1000000`, or a `vcan0` for testing.
- EtherCAT: build SOEM 1.4 as `libsoem.so` on the loader path; raw sockets need `CAP_NET_RAW`.

## Where this differs from the original design sketch

- Interface fallbacks **throw at setup** instead of silently returning `nothing`.
- `@check_allocs` is not wrapped around `control_step!` (that would check only on call, and only for the
  types it sees then). `check_step_allocs` / `check_loop_allocs` prove the *exact* concrete instantiation,
  including the full loop with timing and telemetry.
- The hot loop reads the clock with `clock_gettime` and makes **static** SOEM `ccall`s. `time_ns()` (the
  `jl_hrtime` runtime call) and calls through C function pointers are flagged by AllocCheck, so the loop
  avoids them.
- CAN and timespec buffers are preallocated `RefValue`s instead of fresh `Ref`s per cycle.
- `SCHED_FIFO`, CPU affinity, `mlockall` and the GC state are all **restored** on exit, including on error.
- Bumper.jl is not used: every hot-path buffer is static (`SVector`/`MVector`/preallocated `Vector`).

## Status

The 5,044 tests pass on Julia 1.11 (simulation, allocation proofs, SPSC concurrency, soft-RT loop timing).
`SocketCANBackend` and `EtherCATBackend` are allocation-checked but **not yet exercised on real hardware**.
Use `vcan0` and a bench EtherCAT slave before connecting anything that moves.
