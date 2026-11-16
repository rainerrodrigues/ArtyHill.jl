# Code generation

Two routes produce a library that a C/C++ real-time target can call.

| | [`compile_controller`](@ref) (StaticCompiler.jl) | [`juliac_build`](@ref) (JuliaC.jl) |
| --- | --- | --- |
| Julia | 1.10 / 1.11 | ≥ 1.12 |
| Runtime | none: depends on libc only | trimmed `libjulia` (bundle it with `bundle=`) |
| Language subset | no GC allocation, no exceptions, raw pointers | most of Julia, including ArtyHill's own types |

## StaticCompiler

```julia
using ArtyHill, StaticCompiler, StaticTools
pd(u::Ptr{Float64}, x::Ptr{Float64}, y::Ptr{Float64}) =
    (unsafe_store!(u, -40unsafe_load(y, 1) - 12unsafe_load(y, 2)); Int32(0))
compile_controller(pd, (Ptr{Float64}, Ptr{Float64}, Ptr{Float64}), "build", "pd")
```

## juliac

Write a file with `Base.@ccallable` entry points (see `examples/juliac_controller.jl`,
which wraps a [`LimitedController`](@ref)), then:

```julia
using ArtyHill, JuliaC
juliac_build("examples/juliac_controller.jl"; output="build/libpd_limited")
```
