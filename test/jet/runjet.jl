# Static analysis of ArtyHill with JET.jl (type errors, undefined names, method
# errors JET can prove). Run from the repository root:
#
#   julia --project=test/jet -e 'using Pkg; Pkg.develop(path="."); Pkg.instantiate()'
#   julia --project=test/jet test/jet/runjet.jl
using JET, ArtyHill, Test

@testset "JET" begin
    JET.test_package(ArtyHill; target_defined_modules=true)
end
