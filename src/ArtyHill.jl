"""
    ArtyHill

Hardware-in-the-Loop (HIL) and real-time embedded prototyping for Julia.

Three decoupled layers:

1. **Hardware abstraction** – `AbstractHILBackend` with allocation-free
   `read_sensors!` / `write_actuators!` (pure simulation, SocketCAN, EtherCAT/SOEM).
2. **Deterministic runtime** – zero-allocation `ControlWorkspace`, `control_step!`,
   and `run_realtime_loop!` (mlockall, SCHED_FIFO, CPU pinning, absolute-time
   `clock_nanosleep`, GC pause) plus a lock-free SPSC telemetry ring.
3. **Verification & codegen** – allocation proofs via the AllocCheck extension
   (`check_step_allocs`) and standalone compilation via the StaticCompiler
   extension (`compile_controller`).
"""
module ArtyHill

using StaticArrays
using Libdl
using Preferences

export AbstractHILBackend, SimulationBackend, SocketCANBackend, EtherCATBackend
export init_backend!, read_sensors!, write_actuators!, close_backend!, heartbeat!
export AbstractPlant, DiscreteLinearPlant, plant_output!, plant_step!, reset_plant!
export ControlWorkspace, control_step!, StaticFeedback, LimitedController
export RTConfig, LoopStats, run_realtime_loop!, monotonic_ns, latency_percentile, send_safe_command!
export SPSCRing, try_push!, try_pop!, drain!, TelemetrySample
export check_step_allocs, compile_controller
export HIL_OK, HIL_ERR_IO, HIL_ERR_NOT_INIT, HIL_ERR_SYS

include("status.jl")
include("backends/interface.jl")
include("backends/simulation.jl")
include("backends/socketcan.jl")
include("backends/ethercat.jl")
include("backends/cia402.jl")
include("workspace.jl")
include("telemetry.jl")
include("realtime.jl")
include("codegen.jl")

end # module
