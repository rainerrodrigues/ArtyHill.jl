# Ecosystem extensions

Each extension loads automatically when its package is loaded alongside ArtyHill.

| Load | You get |
| --- | --- |
| `using AllocCheck` | [`check_step_allocs`](@ref), [`check_loop_allocs`](@ref) |
| `using StaticCompiler, StaticTools` | [`compile_controller`](@ref) |
| `using JuliaC` (Julia ≥ 1.12) | [`juliac_build`](@ref) |
| `using DiscretePIDs` | [`pid_bank`](@ref): DiscretePIDs.jl controllers as one allocation-free ArtyHill controller |
| `using VeriJul` | [`verijul_backend`](@ref): co-simulate Verilog/RTL as the plant or I/O hardware |
| `using KaosCleaner` (Julia ≥ 1.12) | [`clean_telemetry`](@ref): outlier removal and resampling of telemetry logs |

## RTL co-simulation with VeriJul

```julia
using ArtyHill, VeriJul
dut = load_dut("pwm_current_loop.v", "pwm_current_loop")
backend = verijul_backend(dut; inputs=[:duty], outputs=[:current_adc],
                          cycles_per_step=1000, input_scale=4095, output_scale=100)
```

The FPGA logic is clocked `cycles_per_step` times per control period, so the
Julia controller sees the same sample-to-actuation timing it will see on hardware.

## Cleaning telemetry

```julia
using ArtyHill, KaosCleaner
df = clean_telemetry("telemetry.csv"; outliers="hampel", resample="1ms")
```
