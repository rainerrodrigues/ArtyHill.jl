# From reducer geometry to a tuned controller, end to end:
#
#   1. Geometry → model: reduction ratio and backlash of a cycloidal drive come from
#      CycloidCore (github.com/rainerrodrigues/cycloidal-generator) when it is
#      installed, otherwise from the same design's precomputed values.
#   2. Simulation: motor → backlash + compliance → load as an ArtyHill NonlinearPlant,
#      run through the real-time loop with telemetry.
#   3. Identification from telemetry: backlash from the reversal hysteresis, reflected
#      inertia and damping by least squares.
#   4. Control: a bandwidth sweep with gains from the identified model, comparing
#      load-side feedback (backlash inside the loop, prone to hunting) with motor-side
#      feedback plus backlash compensation (no hunting, small static offset).
#
#   julia --threads=2 --project=examples examples/cycloidal_drive.jl
#
# `run_example(; period_ns)` returns the numbers; the test suite runs it with a short
# wall-clock period (model time is always 1 ms per step).
using ArtyHill, StaticArrays

# --------------------------------------------------------------------- 1. geometry

function drive_geometry()
    if Base.find_package("CycloidCore") !== nothing
        CC = Base.require(Base.PkgId(Base.UUID("aa856039-837d-49d9-97c7-f812b01e6cd3"), "CycloidCore"))
        p = Base.invokelatest(CC.DriveParams)                  # "Default — Ø80 mm, 9:1"
        bl = Base.invokelatest(CC.backlash, p)
        return (ratio=Base.invokelatest(CC.reduction_ratio, p), backlash=bl.total, source="CycloidCore")
    end
    # Same design, values computed by CycloidCore.backlash(DriveParams()).
    return (ratio=9, backlash=0.023984454863420684, source="precomputed (CycloidCore not installed)")
end

# ------------------------------------------------------------------ 2. plant model

const DT = 1e-3            # model time per control step [s]
const SUBSTEPS = 20

Base.@kwdef struct DriveModel
    N::Float64                 # reduction ratio
    backlash::Float64          # total play at the output [rad]
    Jm::Float64 = 2e-5         # motor + input-side inertia [kg m²]
    bm::Float64 = 1e-4         # motor viscous damping [N m s/rad]
    JL::Float64 = 4e-3         # load inertia [kg m²]
    bL::Float64 = 2e-3         # load viscous damping [N m s/rad]
    k::Float64 = 400.0         # transmission stiffness at the output [N m/rad]
    c::Float64 = 0.5           # transmission damping [N m s/rad]
    Fc::Float64 = 0.03         # load Coulomb friction [N m]
    umax::Float64 = 0.3        # motor torque limit [N m]
end

# Smooth Coulomb friction characteristic (sign function with a 10 mrad/s ramp).
@inline friction_shape(ω) = tanh(ω / 0.01)

# Dead zone of half-width h: zero inside, linear outside.
@inline deadzone(d, h) = d > h ? d - h : (d < -h ? d + h : zero(d))

# x = (θm, ωm, θl, ωl); θm is the motor angle, θl the load angle.
function drive_step(m::DriveModel)
    h = m.backlash / 2
    return (x, u) -> begin
        θm, ωm, θl, ωl = x
        τm = clamp(u[1], -m.umax, m.umax)
        dt = DT / SUBSTEPS
        for _ in 1:SUBSTEPS
            δ = θm / m.N - θl                                      # twist at the output
            engaged = abs(δ) > h
            τ = m.k * deadzone(δ, h) + (engaged ? m.c * (ωm / m.N - ωl) : 0.0)
            ωm += dt * (τm - m.bm * ωm - τ / m.N) / m.Jm
            ωl += dt * (τ - m.bL * ωl - m.Fc * friction_shape(ωl)) / m.JL
            θm += dt * ωm
            θl += dt * ωl
        end
        SVector(θm, ωm, θl, ωl)
    end
end
# Encoders: motor angle reflected to the output, and the load angle.
drive_output(m::DriveModel) = x -> SVector(x[1] / m.N, x[3])

make_backend(m::DriveModel) =
    SimulationBackend(NonlinearPlant(drive_step(m), drive_output(m), SVector(0.0, 0.0, 0.0, 0.0), Val(1)))

# ------------------------------------------------------------ running experiments

# Run `controller` for `n` steps through ArtyHill's real-time loop and return the telemetry.
function experiment(m::DriveModel, controller, n; period_ns)
    backend = make_backend(m)
    init_backend!(backend)
    ws = ControlWorkspace{2,1,2}()
    ring = SPSCRing(ws, n)
    stats = run_realtime_loop!(ws, controller, backend, period_ns, n; telemetry=ring,
                               priority=0, lock_memory=false, max_consecutive_errors=0)
    samples = TelemetrySample{2,1}[]
    drain!(s -> push!(samples, s), ring)
    length(samples) == n || error("telemetry dropped samples")
    θm = [s.y[1] for s in samples]          # motor angle at the output side
    θl = [s.y[2] for s in samples]
    u = [s.u[1] for s in samples]
    return (; θm, θl, u, stats)
end

# Motor-side PD on the output-referred motor angle (collocated: always stable).
mutable struct MotorPD{R}
    const kp::Float64
    const kd::Float64
    const ref::R            # step index -> motor-side reference [rad, output side]
    k::Int
    prev::Float64
end
MotorPD(kp, kd, ref) = MotorPD(kp, kd, ref, 0, 0.0)
function (c::MotorPD)(u, x, y)
    c.k += 1
    ω = c.k == 1 ? 0.0 : (y[1] - c.prev) / DT
    c.prev = y[1]
    u[1] = c.kp * (c.ref(c.k, y) - y[1]) - c.kd * ω
    return nothing
end

# ---------------------------------------------------------------- 3. identification

# Backlash: motor-vs-load lag swings by the full play at every reversal.
function identify_backlash(θm, θl; skip=500)
    lag = θm[skip:end] .- θl[skip:end]
    return maximum(lag) - minimum(lag)
end

# Least squares on the energy balance of the two masses. Adding the motor equation
# (reflected to the output) and the load equation cancels the gear torque exactly,
# so this holds in and out of the backlash:
#   N·u = (Jm·N²)·φ̈m + (bm·N²)·φ̇m + JL·θ̈l + bL·θ̇l + Fc·sgn(θ̇l)   (φm = θm/N)
function identify_inertia(φm, θl, u, N; skip=50)
    A = zeros(SMatrix{5,5,Float64})
    rhs = zeros(SVector{5,Float64})
    for i in (skip + 1):(length(φm) - 1)
        αm = (φm[i + 1] - 2φm[i] + φm[i - 1]) / DT^2
        ωm = (φm[i + 1] - φm[i - 1]) / 2DT
        αl = (θl[i + 1] - 2θl[i] + θl[i - 1]) / DT^2
        ωl = (θl[i + 1] - θl[i - 1]) / 2DT
        φ = SVector(αm, ωm, αl, ωl, friction_shape(ωl))
        τ = N * (u[i - 1] + u[i]) / 2        # torque acting across the central difference
        A += φ * φ'
        rhs += φ * τ
    end
    Jm, bm, JL, bL, Fc = A \ rhs
    return (J=Jm + JL, B=bm + bL, Jm_reflected=Jm, JL=JL, Fc=Fc)
end

# ---------------------------------------------------------------------- 4. control

rms(x) = sqrt(sum(abs2, x) / length(x))

function run_example(; period_ns=1_000_000, verbose=true)
    geo = drive_geometry()
    m = DriveModel(; N=geo.ratio, backlash=geo.backlash)
    say(args...) = verbose && println(args...)
    say("Drive: $(geo.ratio):1, backlash $(round(geo.backlash * 180 / π * 60; digits=1)) arcmin ($(geo.source))")

    # Experiment A: slow triangle on the motor side → backlash from hysteresis.
    tri(k, _) = 0.15 * (2 / π) * asin(sin(2π * k * DT / 2.0))
    A = experiment(m, MotorPD(25.0 / m.N, 0.4 / m.N, tri), 6_000; period_ns)
    b_hat = identify_backlash(A.θm, A.θl)
    say("  identified backlash: $(round(b_hat * 180 / π * 60; digits=1)) arcmin")

    # Experiment B: chirp torque → inertias and damping of both sides.
    chirp = let k = Ref(0)
        (u, x, y) -> (k[] += 1; t = k[] * DT; u[1] = 0.03 * sin(2π * (0.5 + 3t) * t); nothing)
    end
    B = experiment(m, chirp, 3_000; period_ns)
    id = identify_inertia(B.θm, B.θl, B.u, m.N)
    J_true = m.Jm * m.N^2 + m.JL                                  # output-side equivalent
    say("  identified inertia: $(round(id.J; sigdigits=3)) kg m² (true $(round(J_true; sigdigits=3))), " *
        "damping $(round(id.B; sigdigits=3))")

    say("  identified friction: $(round(id.Fc; sigdigits=3)) N m (true $(m.Fc))")

    # Experiment C: controller sweep. Move the load to 0.2 rad along a 1 s
    # minimum-jerk trajectory; for each bandwidth, compute PD gains from the
    # identified model and compare where the feedback comes from:
    #   load PD   — load encoder only (non-collocated: backlash inside the loop)
    #   motor PD  — motor encoder (collocated: backlash outside the loop), reference
    #               shifted by the identified half-backlash so the gear stays on its
    #               driving flank (no load-side correction, so a small offset remains)
    r = 0.2
    traj(k) = (s = min(k * DT / 1.0, 1.0); r * (10s^3 - 15s^4 + 6s^5))
    n = 3_000
    tail = (n - 1_000):n
    p2p(x) = maximum(x) - minimum(x)
    mrad(x) = round(x * 1e3; digits=2)
    sweep = NamedTuple[]
    say("  last second of a 0.2 rad move, load error in mrad (RMS / peak-to-peak):")
    for ωn in (25.0, 40.0, 60.0)
        ζ = 0.8
        kp, kd = id.J * ωn^2 / m.N, max(2ζ * ωn * id.J - id.B, 0.0) / m.N   # motor-torque units
        load_pd = let prev = Ref(0.0), k = Ref(0)
            (u, x, y) -> begin
                k[] += 1
                ω = k[] == 1 ? 0.0 : (y[2] - prev[]) / DT
                prev[] = y[2]
                u[1] = kp * (traj(k[]) - y[2]) - kd * ω
                nothing
            end
        end
        motor_pd = MotorPD(kp, kd, (k, y) -> traj(k) + sign(r) * b_hat / 2)
        res = map((load_pd, motor_pd)) do ctrl
            C = experiment(m, ctrl, n; period_ns)
            (rms=rms(r .- C.θl[tail]), p2p=p2p(C.θl[tail]))
        end
        push!(sweep, (; ωn, load_pd=res[1], motor_pd=res[2]))
        say("    ωn = $(Int(ωn)) rad/s   load PD $(mrad(res[1].rms)) / $(mrad(res[1].p2p))" *
            "   motor PD + compensation $(mrad(res[2].rms)) / $(mrad(res[2].p2p))")
    end
    return (; geo, model=m, b_hat, id, J_true, sweep)
end

if abspath(PROGRAM_FILE) == @__FILE__
    run_example()
end
