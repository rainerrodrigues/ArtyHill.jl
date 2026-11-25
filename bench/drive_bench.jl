# Bring up a real CiA 402 servo drive over EtherCAT, step by step, then run a
# short, slow, rate-limited move in the real-time loop and report timing,
# DC phase lock and following error.
#
#   COUNTS_PER_REV=131072 bench/rtrun.sh julia --threads=2 --project=bench bench/drive_bench.jl eth1
#
# Walkthrough: docs/src/tutorials/drive_bringup.md
#
# Settings (environment variables, all optional):
#   AXES=1               number of drives on the line (all get the same move)
#   COUNTS_PER_REV=131072  encoder counts per motor revolution (check the drive)
#   MOVE_REV=0.05        move amplitude in revolutions (default: 18°)
#   MOVE_SECONDS=4       duration of the move (one smooth out-and-back)
#   MAX_RPS=0.5          hard limit on commanded speed, revolutions per second
#   CYCLE_US=1000        loop and DC Sync0 period
#   CPU=-1               pin the loop to this CPU (use an isolated one)
#   CONFIGURE_PDO=1      program RX_MAP/TX_MAP over CoE in PRE-OP. Set 0 for
#                        drives with fixed PDOs, after editing the maps below
#                        to match the drive's fixed mapping (see its ESI).
#
# SAFETY: the motor WILL move. Clamp the motor or remove the load, keep a
# hardware E-stop (STO) in reach, and start with the defaults. The script asks
# before enabling the drive and before moving.
using ArtyHill

const IFACE          = isempty(ARGS) ? error("usage: drive_bench.jl <interface>") : ARGS[1]
const NAXES          = parse(Int, get(ENV, "AXES", "1"))
const COUNTS_PER_REV = parse(Float64, get(ENV, "COUNTS_PER_REV", "131072"))
const MOVE_REV       = parse(Float64, get(ENV, "MOVE_REV", "0.05"))
const MOVE_SECONDS   = parse(Float64, get(ENV, "MOVE_SECONDS", "4"))
const MAX_RPS        = parse(Float64, get(ENV, "MAX_RPS", "0.5"))
const CYCLE_NS       = 1_000 * parse(Int, get(ENV, "CYCLE_US", "1000"))
const CPU            = parse(Int, get(ENV, "CPU", "-1"))
const CONFIGURE_PDO  = get(ENV, "CONFIGURE_PDO", "1") == "1"
const DT             = CYCLE_NS * 1e-9
const SCALE          = COUNTS_PER_REV / 2π           # counts per radian: user units are rad
const AMP            = 2π * MOVE_REV

# Minimal cyclic-synchronous-position mapping. 0x6060 (mode) is set by SDO.
const RX_MAP = [PDOEntry(0x6040, 0, 16), PDOEntry(0x607A, 0, 32)]                            # controlword, target position
const TX_MAP = [PDOEntry(0x6041, 0, 16), PDOEntry(0x6064, 0, 32), PDOEntry(0x606C, 0, 32)]   # statusword, position, velocity

function confirm(msg)
    print(msg, " [y/N] ")
    readline() in ("y", "Y", "yes") || (println("stopped."); exit(0))
end

pos(y, k) = y[3(k - 1) + 1]

"Hold every axis where it is."
struct Hold end
(::Hold)(u, x, y) = (for k in 1:NAXES; u[k] = pos(y, k); end; nothing)

"Smooth out-and-back: center + AMP·(1 − cos ωt)/2, zero speed at both ends."
mutable struct OutAndBack
    center::Vector{Float64}
    k::Int
end
function (m::OutAndBack)(u, x, y)
    m.k += 1
    s = 0.5 * AMP * (1 - cos(2π * min(m.k * DT, MOVE_SECONDS) / MOVE_SECONDS))
    for k in 1:NAXES
        u[k] = m.center[k] + s
    end
    return nothing
end

function preop!(b, slave)
    vendor  = sdo_read(b, slave, 0x1018, 1, UInt32)
    product = sdo_read(b, slave, 0x1018, 2, UInt32)
    modes   = try sdo_read(b, slave, 0x6502, 0, UInt32) catch; missing end
    println("  slave $slave: vendor 0x", string(vendor, base=16), ", product 0x", string(product, base=16),
            ", supported modes 0x", modes === missing ? "?" : string(modes, base=16))
    modes === missing || modes & 0x80 != 0 || @warn "slave $slave does not report CSP support (0x6502 bit 7)"
    sdo_write!(b, slave, 0x6060, 0, Int8(8))                      # cyclic synchronous position
    if CONFIGURE_PDO
        configure_pdo!(b, slave, 0x1C12, 0x1600, RX_MAP)
        configure_pdo!(b, slave, 0x1C13, 0x1A00, TX_MAP)
    end
end

function cycles!(ws, ctrl, b, n)
    st = HIL_OK
    for _ in 1:n
        s = control_step!(ws, ctrl, b)
        s == HIL_OK || (st = s)
        Libc.systemsleep(DT)
    end
    return st
end

states(codec) = join((cia402_state(codec, k) for k in 1:NAXES), ", ")

println("== 1. Scan, CoE setup in PRE-OP, DC Sync0, OP on $IFACE")
layout = CiA402Layout(RX_MAP, TX_MAP)
codec = CiA402Codec{NAXES}(layout; mode=:csp, position_scale=SCALE, velocity_scale=SCALE)
b = EtherCATBackend(IFACE; output_bytes=layout.out_stride * NAXES, codec, preop_config=preop!,
                    dc_cycle_ns=CYCLE_NS)
ws = ControlWorkspace{3NAXES,NAXES,3NAXES}()
st = init_backend!(b)
st == HIL_OK || error("init_backend! failed ($st). Check the cable, interface name, CAP_NET_RAW, and the log above.")
b.slave_count == NAXES || error("found $(b.slave_count) slaves, expected AXES=$NAXES")
println("  OP reached: $(b.slave_count) slave(s), expected working counter $(b.expected_wkc)")

try
    println("\n== 2. Disabled: read positions for 1 s (no torque)")
    cycles!(ws, Hold(), b, round(Int, 1 / DT)) == HIL_OK || error("process-data errors while disabled")
    for k in 1:NAXES
        println("  axis $k: ", round(pos(ws.y_meas, k) / 2π, digits=5), " rev, state ", cia402_state(codec, k))
    end
    println("  Turn the motor shaft by hand (if safe) and rerun to check the sign and COUNTS_PER_REV.")

    confirm("\n== 3. Enable the drive(s) and hold position?")
    cia402_enable!(codec, true)
    cycles!(ws, Hold(), b, round(Int, 1 / DT))
    all(k -> cia402_state(codec, k) === :operation_enabled, 1:NAXES) ||
        error("drive did not reach operation_enabled: ", states(codec))
    println("  holding, states: ", states(codec))

    confirm("\n== 4. Move $(MOVE_REV) rev out and back over $(MOVE_SECONDS) s (limit $(MAX_RPS) rev/s)?")
    center = [pos(ws.y_meas, k) for k in 1:NAXES]
    lim = LimitedController(OutAndBack(center, 0), fill(-Inf, NAXES), fill(Inf, NAXES);
                            max_rate=fill(2π * MAX_RPS, NAXES), dt=DT)
    reset_limits!(lim, copy(center))
    ring = SPSCRing(ws, 1 << 16)
    samples = telemetry_type(ws)[]
    done = Threads.Atomic{Bool}(false)
    consumer = Threads.@spawn while true
        n = drain!(s -> push!(samples, s), ring)
        done[] && n == 0 && break
        n == 0 && sleep(0.005)
    end
    cfg = RTConfig(; period_ns=CYCLE_NS, iterations=round(Int, (MOVE_SECONDS + 0.5) / DT), cpu=CPU,
                   priority=80, max_consecutive_errors=3, max_consecutive_overruns=5,
                   hist_bucket_ns=1_000, hist_buckets=10_000)
    # No safe_command: a fixed fallback target could be up to MOVE_REV away from
    # where the axis is when the watchdog trips. After an abort the `finally`
    # below holds the current position and disables the drive; if frames stop
    # altogether, the drive's own sync-manager watchdog stops the motor.
    stats = run_realtime_loop!(ws, lim, b, cfg; telemetry=ring)
    done[] = true; wait(consumer)
    display(stats); println()
    println("  abort reason: ", stats.abort_reason, ", DC phase error now: ", dc_phase_error_ns(b), " ns")
    ferr = maximum(abs(s.u[k] - pos(s.y, k)) for s in samples for k in 1:NAXES; init=0.0)
    println("  max following error: ", round(ferr / 2π * 360, digits=3), "° (",
            round(Int, ferr * SCALE), " counts) over ", length(samples), " cycles")
    out = joinpath(@__DIR__, "drive_telemetry.csv")
    open(io -> write_telemetry_csv(io, samples), out, "w")
    println("  telemetry → ", out)
finally
    println("\n== 5. Disable and close")
    cia402_enable!(codec, false)
    cycles!(ws, Hold(), b, 100)
    println("  states: ", states(codec))
    close_backend!(b)
end
