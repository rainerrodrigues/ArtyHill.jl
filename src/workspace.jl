"""
    ControlWorkspace{N,M,P,T}(; x0=zeros)

Preallocated, fixed-size buffers for one control loop:
`x_hat` (N estimated states), `u_cmd` (M commands), `y_meas` (P measurements).
All buffers are `MVector`s, allocated once at construction and reused for
every step, so their memory is contiguous and can be passed straight to C.
"""
struct ControlWorkspace{N,M,P,T}
    x_hat::MVector{N,T}
    u_cmd::MVector{M,T}
    y_meas::MVector{P,T}
end

ControlWorkspace{N,M,P,T}() where {N,M,P,T} =
    ControlWorkspace(zeros(MVector{N,T}), zeros(MVector{M,T}), zeros(MVector{P,T}))
ControlWorkspace{N,M,P}() where {N,M,P} = ControlWorkspace{N,M,P,Float64}()

"""
    ControlWorkspace(N, M, P=N, T=Float64)

Convenience constructor (not type-stable; call it once at setup time).
"""
ControlWorkspace(N::Integer, M::Integer, P::Integer=N, ::Type{T}=Float64) where {T} =
    ControlWorkspace{N,M,P,T}()

"""
    control_step!(ws, controller!, backend) -> Int32

One sense → compute → actuate cycle:

    read_sensors!(ws.y_meas, backend)
    controller!(ws.u_cmd, ws.x_hat, ws.y_meas)
    write_actuators!(ws.u_cmd, backend)

`controller!` may be any callable (closure, functor). The method specializes on
its concrete type, so with an allocation-free controller and backend the whole
step allocates zero bytes — verify with [`check_step_allocs`](@ref).
Returns the first non-OK status, or `HIL_OK`.
"""
@inline function control_step!(ws::ControlWorkspace, controller!::F, backend::B)::Int32 where {F,B<:AbstractHILBackend}
    rs = read_sensors!(ws.y_meas, backend)
    controller!(ws.u_cmd, ws.x_hat, ws.y_meas)
    ws_ = write_actuators!(ws.u_cmd, backend)
    return rs != HIL_OK ? rs : ws_
end

"""
    StaticFeedback(K, r=0)

Allocation-free static output feedback `u = -K (y - r)` with `K::SMatrix{M,P}`.
Also updates `x_hat = y` when `N == P` (a pass-through "estimator").
"""
struct StaticFeedback{M,P,T,MP}
    K::SMatrix{M,P,T,MP}
    r::SVector{P,T}
end
StaticFeedback(K::AbstractMatrix, r=zeros(size(K, 2))) =
    StaticFeedback(SMatrix{size(K)...}(K), SVector{size(K, 2)}(r))

@inline function (c::StaticFeedback{M,P})(u, x_hat, y) where {M,P}
    ys = SVector{P}(ntuple(i -> @inbounds(y[i]), Val(P)))
    uv = -c.K * (ys - c.r)
    @inbounds for i in 1:M
        u[i] = uv[i]
    end
    if length(x_hat) == P
        @inbounds for i in 1:P
            x_hat[i] = ys[i]
        end
    end
    return nothing
end
