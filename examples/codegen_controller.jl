# Compile an allocation-free control law into a standalone shared library
# (no libjulia at runtime) that a C/C++ real-time target can call:
#
#     int32_t pd_controller(double *u, const double *x, const double *y);
#
# Run with:  julia --project=examples examples/codegen_controller.jl
using ArtyHill, StaticCompiler, StaticTools

const KP = 40.0
const KD = 12.0

function pd_controller(u::Ptr{Float64}, x::Ptr{Float64}, y::Ptr{Float64})::Int32
    pos = unsafe_load(y, 1)
    vel = unsafe_load(y, 2)
    unsafe_store!(u, -KP * pos - KD * vel, 1)
    unsafe_store!(x, pos, 1)
    unsafe_store!(x, vel, 2)
    return Int32(0)
end

build = joinpath(@__DIR__, "build")
path = compile_controller(pd_controller, (Ptr{Float64}, Ptr{Float64}, Ptr{Float64}), build, "pd_controller")
println("built ", path)

# Smoke test: call the compiled symbol through the C ABI.
using Libdl
lib = dlopen(joinpath(build, "pd_controller." * Libdl.dlext))
fptr = dlsym(lib, :pd_controller)
u = zeros(1); x = zeros(2); y = [0.1, -0.5]
rc = ccall(fptr, Int32, (Ptr{Float64}, Ptr{Float64}, Ptr{Float64}), u, x, y)
println("rc = $rc, u = $(u[1]) (expected $(-KP*0.1 - KD*(-0.5)))")
