# ArtyHill.jl

Hardware-in-the-Loop (HIL) and real-time embedded prototyping in Julia. One
controller, unchanged, runs against a pure simulation, against SocketCAN /
EtherCAT hardware, against RTL in a Verilator co-simulation, and finally as a
standalone C-callable library.

| Layer | Contents |
| --- | --- |
| Hardware abstraction | [`SimulationBackend`](@ref), [`SocketCANBackend`](@ref), [`EtherCATBackend`](@ref) with [`CiA402Codec`](@ref), [`verijul_backend`](@ref) |
| Deterministic runtime | [`ControlWorkspace`](@ref), [`control_step!`](@ref), [`run_realtime_loop!`](@ref), [`SPSCRing`](@ref) |
| Safety | [`LimitedController`](@ref), watchdog limits in [`RTConfig`](@ref), [`send_safe_command!`](@ref), [`heartbeat!`](@ref) |
| Verification & codegen | [`check_step_allocs`](@ref), [`check_loop_allocs`](@ref), [`compile_controller`](@ref), [`juliac_build`](@ref) |

## Quick start

```julia
using ArtyHill, AllocCheck

const DT = 1e-3
plant   = DiscreteLinearPlant([1 DT; 0 1], reshape([DT^2/2, DT], 2, 1), [1.0 0; 0 1], [1.0, 0])
backend = SimulationBackend(plant)
ws      = ControlWorkspace{2,1,2}()                       # N states, M commands, P measurements
ctrl    = LimitedController(StaticFeedback([40.0 12.0]), -5.0, 5.0; max_rate=1e3, dt=DT)

@assert isempty(check_loop_allocs(ws, ctrl, backend))     # static proof: zero heap allocations

ring = SPSCRing(ws, 1 << 14)
init_backend!(backend)
stats = run_realtime_loop!(ws, ctrl, backend, 1_000_000, 10_000;
                           telemetry=ring, priority=80, cpu=3, safe_command=[0.0])
close_backend!(backend)
```

Swap `SimulationBackend(plant)` for `SocketCANBackend("can0"; ...)` or
`EtherCATBackend("eth0"; ...)` and nothing else changes.
