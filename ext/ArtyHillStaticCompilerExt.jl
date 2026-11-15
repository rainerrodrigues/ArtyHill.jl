module ArtyHillStaticCompilerExt

using ArtyHill
import StaticCompiler
import StaticTools

# StaticCompiler expects the argument types as a tuple of types, e.g. (Ptr{Float64}, Int).
function ArtyHill.compile_controller(f, argtypes::Tuple, path::AbstractString,
                                     name::AbstractString=string(nameof(f)); kind::Symbol=:shlib,
                                     kwargs...)
    mkpath(path)
    if kind === :shlib
        return StaticCompiler.compile_shlib(f, argtypes, path, name; kwargs...)
    elseif kind === :executable
        return StaticCompiler.compile_executable(f, argtypes, path, name; kwargs...)
    else
        throw(ArgumentError("kind must be :shlib or :executable, got :$kind"))
    end
end

ArtyHill.compile_controller(f, argtypes::Type{<:Tuple}, args...; kw...) =
    ArtyHill.compile_controller(f, Tuple(argtypes.parameters), args...; kw...)

end
