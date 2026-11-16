module ArtyHillDiscretePIDsExt

using ArtyHill
using StaticArrays
using DiscretePIDs: DiscretePID, calculate_control!

struct PIDBank{M,T}
    pids::NTuple{M,DiscretePID{T}}
    setpoint::MVector{M,T}
    inputs::NTuple{M,Int}
end

function ArtyHill.pid_bank(pids::DiscretePID{T}...; setpoints=zeros(T, length(pids)),
                           inputs=1:length(pids)) where {T}
    M = length(pids)
    length(setpoints) == M && length(inputs) == M ||
        throw(DimensionMismatch("need one setpoint and one input index per PID"))
    return PIDBank{M,T}(pids, MVector{M,T}(setpoints), Tuple(Int.(inputs)))
end

@inline function (b::PIDBank{M})(u, x_hat, y) where {M}
    @inbounds for k in 1:M
        u[firstindex(u) + k - 1] = calculate_control!(b.pids[k], b.setpoint[k], y[firstindex(y) + b.inputs[k] - 1])
    end
    return nothing
end

end
