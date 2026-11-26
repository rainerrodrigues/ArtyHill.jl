# Verification & code generation entry points. The implementations live in
# package extensions so the core package stays light:
#   ext/ArtyHillAllocCheckExt.jl       (loaded by `using AllocCheck`)
#   ext/ArtyHillStaticCompilerExt.jl   (loaded by `using StaticCompiler, StaticTools`)

"""
    check_step_allocs(ws, controller!, backend) -> Vector

Statically analyse the LLVM IR of `control_step!` specialized for these argument
types and return every possible heap-allocation / dynamic-dispatch site (empty
vector = proven allocation-free). Requires `using AllocCheck`.
"""
function check_step_allocs end

"""
    check_loop_allocs(ws, controller!, backend; telemetry=nothing) -> Vector

Like [`check_step_allocs`](@ref) but for the complete inner real-time loop
(sleeping, timing, statistics and telemetry push). Requires `using AllocCheck`.
"""
function check_loop_allocs end

"""
    compile_controller(f, argtypes, path, name; kind=:shlib) -> String

Compile `f(argtypes...)` into a standalone, libjulia-free shared library
(`kind=:shlib`) or executable (`kind=:executable`) in directory `path` using
StaticCompiler.jl. `f` must take/return only C-compatible types (`Ptr`, numbers)
and must not touch the Julia runtime (no GC allocation, no exceptions) — wrap
raw pointers with `StaticTools.MallocVector`/`unsafe_load`. Requires
`using StaticCompiler, StaticTools`.
"""
function compile_controller end

export check_loop_allocs

# --- further extension entry points ------------------------------------------

"""
    juliac_build(file; output, kind=:lib, trim="safe", project="", bundle=nothing,
                 ccallables=true, verbose=false) -> String

Build `file` with Julia 1.12's `juliac` (via JuliaC.jl) into a shared library
(`kind=:lib`, exporting every `Base.@ccallable` function) or an executable
(`kind=:exe`, `file` must define `@main`). Unlike [`compile_controller`](@ref),
the result links a *trimmed* `libjulia`, so it supports far more of the
language (exceptions, most of Base) at the cost of a runtime dependency; pass
`bundle="dir"` to copy that runtime next to the output for deployment.
Requires Julia ≥ 1.12 and `using JuliaC`.
"""
function juliac_build end

"""
    pid_bank(pids...; setpoints=zeros, inputs=1:M) -> controller

Turn `M` `DiscretePIDs.DiscretePID`s into one allocation-free ArtyHill
controller: `u[k] = calculate_control!(pids[k], setpoint[k], y[inputs[k]])`.
Change references at run time with `bank.setpoint[k] = r`. Requires `using DiscretePIDs`.
"""
function pid_bank end

"""
    verijul_backend(dut; inputs, outputs, clock=:clk, cycles_per_step=1,
                    input_scale=1.0, output_scale=1.0, signed_outputs=true) -> AbstractHILBackend

Co-simulate RTL as the "hardware": a VeriJul.jl `DUT` (Verilator model) becomes
a backend. Each `write_actuators!` converts `u[k] * input_scale[k]` to an
integer on input port `inputs[k]` and clocks the design `cycles_per_step`
times; `read_sensors!` reads output port `outputs[k]` (two's complement when
`signed_outputs`) divided by `output_scale[k]`. Ports must be ≤ 64 bits.
Use it to test FPGA logic (PWM generators, encoder decoders, current loops)
against the Julia controller before synthesis. Requires `using VeriJul`.
"""
function verijul_backend end

"""
    clean_telemetry(csv_path; outliers="hampel", resample=nothing,
                    resample_method="linear", columns=nothing) -> DataFrame

Post-process a telemetry CSV written by [`write_telemetry_csv`](@ref) with
KaosCleaner.jl: outliers in the measurement/command columns (`y*`, `u*`, or
`columns`) become missing, and with `resample="1ms"` the log is put on a
regular time grid, ready for system identification. Requires Julia ≥ 1.12 and
`using KaosCleaner` (`Pkg.add("KaosCleaner")`).
"""
function clean_telemetry end

export juliac_build, pid_bank, verijul_backend, clean_telemetry
