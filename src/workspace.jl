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

"""
    LimitedController(controller!, umin, umax; max_rate=Inf, dt=1.0)

Wrap any controller with actuator protection, applied after it runs:

1. non-finite commands (`NaN`, `±Inf`) are replaced by the previous command;
2. each command is limited to change by at most `max_rate * dt` per step;
3. each command is clamped to `[umin[i], umax[i]]`.

`umin`, `umax` and `max_rate` may be scalars or length-`M` vectors. The
previous command starts at zero clamped into the limits; call
[`reset_limits!`](@ref) to change it. Allocation-free.
"""
struct LimitedController{C,M,T}
    inner::C
    umin::SVector{M,T}
    umax::SVector{M,T}
    max_step::SVector{M,T}
    u_prev::MVector{M,T}
end

function LimitedController(inner, umin, umax; max_rate=Inf, dt::Real=1.0)
    M = max(length(umin), length(umax), length(max_rate))
    T = Float64
    expand(v) = SVector{M,T}(length(v) == 1 ? ntuple(_ -> T(first(v)), M) : Tuple(T.(v)))
    lo, hi, r = expand(umin), expand(umax), expand(max_rate)
    all(lo .<= hi) || throw(ArgumentError("umin must be ≤ umax"))
    all(r .> 0) || throw(ArgumentError("max_rate must be positive"))
    return LimitedController{typeof(inner),M,T}(inner, lo, hi, r .* T(dt), MVector(clamp.(zero(lo), lo, hi)))
end

"Set the limiter's notion of the previously applied command (e.g. the drive's current output)."
function reset_limits!(c::LimitedController{C,M}, u_prev=zeros(M)) where {C,M}
    @inbounds for i in 1:M
        c.u_prev[i] = clamp(u_prev[i], c.umin[i], c.umax[i])
    end
    return c
end

@inline function (c::LimitedController{C,M})(u, x_hat, y) where {C,M}
    c.inner(u, x_hat, y)
    @inbounds for i in 1:M
        v = u[i]
        p = c.u_prev[i]
        isfinite(v) || (v = p)
        v = clamp(v, p - c.max_step[i], p + c.max_step[i])
        v = clamp(v, c.umin[i], c.umax[i])
        u[i] = v
        c.u_prev[i] = v
    end
    return nothing
end

export reset_limits!
