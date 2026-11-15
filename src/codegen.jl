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
