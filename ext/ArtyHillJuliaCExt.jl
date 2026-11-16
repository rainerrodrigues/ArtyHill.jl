module ArtyHillJuliaCExt

using ArtyHill
import JuliaC

function ArtyHill.juliac_build(file::AbstractString; output::AbstractString, kind::Symbol=:lib,
                               trim::Union{Nothing,String}="safe", project::AbstractString="",
                               bundle::Union{Nothing,AbstractString}=nothing, ccallables::Bool=(kind === :lib),
                               verbose::Bool=false)
    VERSION >= v"1.12" || error("juliac_build requires Julia 1.12 or newer")
    output_type = kind === :lib ? "--output-lib" : kind === :exe ? "--output-exe" :
                  throw(ArgumentError("kind must be :lib or :exe"))
    mkpath(dirname(abspath(output)))
    img = JuliaC.ImageRecipe(; output_type, file=String(file), trim_mode=trim,
                             add_ccallables=ccallables, project=String(project), verbose)
    link = JuliaC.LinkRecipe(; image_recipe=img, outname=String(output),
                             rpath=bundle === nothing ? "@julia" : "@bundle")
    JuliaC.compile_products(img)
    JuliaC.link_products(link)
    if bundle !== nothing
        JuliaC.bundle_products(JuliaC.BundleRecipe(; link_recipe=link, output_dir=String(bundle)))
    end
    return link.outname
end

end
