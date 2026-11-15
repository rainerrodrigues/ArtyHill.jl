# Software-in-the-loop run at 1 kHz with lock-free telemetry.
#
#   julia --threads=2 --project=examples examples/sim_hil.jl
#   sudo julia --threads=2 --project=examples examples/sim_hil.jl   # with SCHED_FIFO + mlockall
#
# Swap `SimulationBackend(plant)` for `SocketCANBackend("can0"; ...)` or
# `EtherCATBackend("eth0"; ...)` — nothing else changes.
using ArtyHill, AllocCheck

const DT = 1e-3
plant = DiscreteLinearPlant([1.0 DT; 0.0 1.0], reshape([0.5DT^2, DT], 2, 1),
                            [1.0 0.0; 0.0 1.0], [1.0, 0.0])
backend = SimulationBackend(plant)
ws = ControlWorkspace{2,1,2}()
ctrl = StaticFeedback([40.0 12.0])

# 1. Prove the hot path never touches the GC.
@assert isempty(check_step_allocs(ws, ctrl, backend))
ring = SPSCRing(ws, 1 << 14)
@assert isempty(check_loop_allocs(ws, ctrl, backend; telemetry=ring))

# 2. Background consumer moves samples off the real-time thread.
samples = TelemetrySample{2,1}[]
done = Threads.Atomic{Bool}(false)
consumer = Threads.@spawn begin
    while !done[]
        drain!(s -> push!(samples, s), ring) == 0 && sleep(0.005)
    end
    drain!(s -> push!(samples, s), ring)
end

# 3. Run 3 s at 1 kHz. Soft mode (warn + continue) if not privileged.
init_backend!(backend)
for _ in 1:10; control_step!(ws, ctrl, backend); end     # warm-up / compile
init_backend!(backend)                                     # reset plant
stats = run_realtime_loop!(ws, ctrl, backend, 1_000_000, 3_000;
                           telemetry=ring, priority=80, cpu=-1)
close_backend!(backend)
done[] = true; wait(consumer)

display(stats); println()
println("final position = ", plant.x[1], ", samples logged = ", length(samples),
        ", dropped = ", ring.dropped)
open(joinpath(@__DIR__, "telemetry.csv"), "w") do io
    write_telemetry_csv(io, samples)
end
