# Measure ArtyHill loop timing on this machine.
#
#   julia --threads=2 --project=bench bench/latency.jl [period_us] [iterations] [cpu] [priority]
#   sudo julia --threads=2 --project=bench bench/latency.jl 1000 100000 3 80
#
# Prints mean / percentiles / max of wake-up jitter and step latency and writes
# the jitter histogram to bench/artyhill_hist.csv (bucket_us,count), in the same
# 1 µs buckets as `cyclictest -h`. Use bench/compare_cyclictest.sh to run both
# under identical settings.
using ArtyHill

period_us  = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 1_000
iterations = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 10_000
cpu        = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : -1
priority   = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 80

const DT = period_us * 1e-6
plant = DiscreteLinearPlant([1.0 DT; 0.0 1.0], reshape([0.5DT^2, DT], 2, 1), [1.0 0.0; 0.0 1.0], [1.0, 0.0])
backend = SimulationBackend(plant)
ws = ControlWorkspace{2,1,2}()
ctrl = LimitedController(StaticFeedback([40.0 12.0]), -100.0, 100.0)

init_backend!(backend)
for _ in 1:1_000; control_step!(ws, ctrl, backend); end   # warm-up
init_backend!(backend)

cfg = RTConfig(; period_ns=period_us * 1_000, iterations, cpu, priority,
               max_consecutive_errors=0, hist_bucket_ns=1_000, hist_buckets=10_000)
stats = run_realtime_loop!(ws, ctrl, backend, cfg; safe_command=[0.0])
display(stats); println()

out = joinpath(@__DIR__, "artyhill_hist.csv")
open(out, "w") do io
    println(io, "bucket_us,count")
    for (i, c) in enumerate(stats.wake_hist)
        c > 0 && println(io, i - 1, ',', c)
    end
end
println("jitter histogram → ", out)
