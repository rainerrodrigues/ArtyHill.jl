module ArtyHillAllocCheckExt

using ArtyHill
using ArtyHill: ControlWorkspace, AbstractHILBackend, LoopStats, Timespec, _rt_inner!
import AllocCheck

function ArtyHill.check_step_allocs(ws::ControlWorkspace, controller!, backend::AbstractHILBackend)
    return AllocCheck.check_allocs(ArtyHill.control_step!,
                                   (typeof(ws), typeof(controller!), typeof(backend)))
end

function ArtyHill.check_loop_allocs(ws::ControlWorkspace, controller!, backend::AbstractHILBackend;
                                    telemetry=nothing)
    return AllocCheck.check_allocs(_rt_inner!,
                                   (LoopStats, typeof(ws), typeof(controller!), typeof(backend),
                                    Int64, Int64, typeof(telemetry), Base.RefValue{Timespec}, Int, Int,
                                    Threads.Atomic{Bool}))
end

end
