"""
    AbstractPlant

A plant model driven by [`SimulationBackend`](@ref). Implement

    plant_output!(y, plant)   # write measurements of the current state into y
    plant_step!(plant, u)     # advance one sample period under input u
    reset_plant!(plant)       # return to the initial condition

All three must be allocation-free.
"""
abstract type AbstractPlant end

function plant_output! end
function plant_step! end
function reset_plant! end

"""
    SimulationBackend(plant)

Pure-software backend: sensors are read from `plant`, and every actuator write
advances `plant` by one sample period. Use it for software-in-the-loop runs and
CI before swapping in a hardware backend — the controller code is unchanged.
"""
struct SimulationBackend{P<:AbstractPlant} <: AbstractHILBackend
    plant::P
end

init_backend!(b::SimulationBackend) = (reset_plant!(b.plant); HIL_OK)
close_backend!(::SimulationBackend) = HIL_OK
@inline read_sensors!(y, b::SimulationBackend) = (plant_output!(y, b.plant); HIL_OK)
@inline write_actuators!(u, b::SimulationBackend) = (plant_step!(b.plant, u); HIL_OK)

# ---------------------------------------------------------------------------
# Discrete-time LTI plant:  x⁺ = A x + B u,   y = C x
# ---------------------------------------------------------------------------

"""
    DiscreteLinearPlant(A, B, C, x0)

Discrete-time state-space plant with `N` states, `M` inputs and `P` outputs,
stored in `StaticArrays` so stepping it never allocates.
"""
mutable struct DiscreteLinearPlant{N,M,P,T,NN,NM,PN} <: AbstractPlant
    const A::SMatrix{N,N,T,NN}
    const B::SMatrix{N,M,T,NM}
    const C::SMatrix{P,N,T,PN}
    const x0::SVector{N,T}
    x::SVector{N,T}
end

function DiscreteLinearPlant(A::AbstractMatrix, B::AbstractMatrix, C::AbstractMatrix,
                             x0::AbstractVector)
    N = size(A, 1); M = size(B, 2); P = size(C, 1)
    size(A) == (N, N) && size(B, 1) == N && size(C, 2) == N && length(x0) == N ||
        throw(DimensionMismatch("inconsistent A/B/C/x0 sizes"))
    T = promote_type(eltype(A), eltype(B), eltype(C), eltype(x0))
    sx0 = SVector{N,T}(x0)
    return DiscreteLinearPlant(SMatrix{N,N,T}(A), SMatrix{N,M,T}(B), SMatrix{P,N,T}(C), sx0, sx0)
end

@inline function plant_output!(y, p::DiscreteLinearPlant{N,M,P}) where {N,M,P}
    yv = p.C * p.x
    @inbounds for i in 1:P
        y[i] = yv[i]
    end
    return nothing
end

@inline function plant_step!(p::DiscreteLinearPlant{N,M}, u) where {N,M}
    p.x = p.A * p.x + p.B * SVector{M}(ntuple(i -> @inbounds(u[i]), Val(M)))
    return nothing
end

reset_plant!(p::DiscreteLinearPlant) = (p.x = p.x0; nothing)

# ---------------------------------------------------------------------------
# Generic nonlinear plant from user functions (backlash, torque ripple, ...)
# ---------------------------------------------------------------------------

"""
    NonlinearPlant(f, h, x0, Val(M))

Plant defined by `x⁺ = f(x, u)` and `y = h(x)` where `x`, `u` and the returned
values are `SVector`s. `f` and `h` must be type-stable and allocation-free.
Useful for modelling gearbox backlash or torque ripple before touching hardware.
"""
mutable struct NonlinearPlant{F,H,N,M,T} <: AbstractPlant
    const f::F
    const h::H
    const x0::SVector{N,T}
    x::SVector{N,T}
end
NonlinearPlant(f::F, h::H, x0::SVector{N,T}, ::Val{M}) where {F,H,N,T,M} =
    NonlinearPlant{F,H,N,M,T}(f, h, x0, x0)

@inline function plant_output!(y, p::NonlinearPlant)
    yv = p.h(p.x)
    @inbounds for i in eachindex(yv)
        y[i] = yv[i]
    end
    return nothing
end

@inline function plant_step!(p::NonlinearPlant{F,H,N,M}, u) where {F,H,N,M}
    p.x = p.f(p.x, SVector{M}(ntuple(i -> @inbounds(u[i]), Val(M))))
    return nothing
end

reset_plant!(p::NonlinearPlant) = (p.x = p.x0; nothing)

export NonlinearPlant
