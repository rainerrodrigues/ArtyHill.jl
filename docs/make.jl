using Documenter, ArtyHill

makedocs(;
    sitename = "ArtyHill.jl",
    modules = [ArtyHill],
    authors = "Rainer Rodrigues",
    format = Documenter.HTML(; prettyurls = get(ENV, "CI", "false") == "true"),
    pages = [
        "Home" => "index.md",
        "Guide" => [
            "Backends" => "backends.md",
            "Real-time loop" => "realtime.md",
            "Safety" => "safety.md",
            "Code generation" => "codegen.md",
            "Ecosystem extensions" => "extensions.md",
        ],
        "API reference" => "api.md",
    ],
    warnonly = [:missing_docs],
)

deploydocs(; repo = "github.com/rainerrodrigues/ArtyHill.jl.git", devbranch = "main", push_preview = false)
