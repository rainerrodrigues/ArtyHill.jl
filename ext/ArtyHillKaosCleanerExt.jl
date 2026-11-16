module ArtyHillKaosCleanerExt

using ArtyHill
import KaosCleaner
using KaosCleaner: CleanOptions, load_dataset, clean_dataset!

function ArtyHill.clean_telemetry(path::AbstractString; outliers::Union{Nothing,String}="hampel",
                                  outlier_window::Integer=7, outlier_threshold=nothing,
                                  resample::Union{Nothing,String}=nothing,
                                  resample_method::String="linear", columns=nothing)
    header = split(strip(readline(path)), ',')
    "t_ns" in header || throw(ArgumentError("$path is not an ArtyHill telemetry CSV (no t_ns column)"))
    cols = columns === nothing ? [String(c) for c in header if occursin(r"^[yu]\d+$", c)] :
                                 [String(c) for c in columns]
    plan = outliers === nothing ? nothing : Dict(c => outliers for c in cols)
    opts = CleanOptions(; time_column="t_ns", epoch=Dict("t_ns" => "ns"),
                        outliers=plan, outlier_window=Int(outlier_window),
                        outlier_threshold=outlier_threshold === nothing ? nothing : Float64(outlier_threshold),
                        resample, resample_method,
                        rename=false, dedupe=false, drop_empty_rows=false)
    return clean_dataset!(load_dataset(path, opts), opts)
end

end
