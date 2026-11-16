# Build an ArtyHill controller into a C-callable shared library with Julia 1.12's
# juliac (trimmed runtime). Compared with StaticCompiler (codegen_controller.jl)
# this can use ArtyHill's own types — here a PD law behind actuator limits:
#
#   julia +1.12 --project=<env with ArtyHill + JuliaC> -e '
#       using ArtyHill, JuliaC
#       juliac_build("examples/juliac_controller.jl"; output="build/libpd_limited")'
#
# C signature:  int32_t pd_limited_step(double *u, double *x, const double *y);
module PDLimited

using ArtyHill, StaticArrays

const CTRL = LimitedController(StaticFeedback([40.0 12.0]), -5.0, 5.0; max_rate=1000.0, dt=1e-3)
const U = zeros(MVector{1,Float64})
const X = zeros(MVector{2,Float64})
const Y = zeros(MVector{2,Float64})

Base.@ccallable function pd_limited_step(u::Ptr{Float64}, x::Ptr{Float64}, y::Ptr{Float64})::Int32
    Y[1] = unsafe_load(y, 1); Y[2] = unsafe_load(y, 2)
    CTRL(U, X, Y)
    unsafe_store!(u, U[1], 1)
    unsafe_store!(x, X[1], 1); unsafe_store!(x, X[2], 2)
    return Int32(0)
end

Base.@ccallable function pd_limited_reset()::Int32
    reset_limits!(CTRL, SVector(0.0))
    return Int32(0)
end

end
