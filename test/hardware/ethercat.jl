# EtherCAT end-to-end: the real SOEM 1.4 master (built from source) talks over a
# veth pair to `test/ethercat/ecat_emu.c`, which emulates a line of CiA 402
# drives at the frame level. Runs only when ARTYHILL_ETHERCAT names the master
# side of a veth pair whose peer is ARTYHILL_ETHERCAT_PEER, ARTYHILL_LIBSOEM_DIR
# holds libsoem.so, and the process has CAP_NET_RAW. `test/ethercat/run.sh`
# sets all of that up inside an unprivileged user + network namespace.

const ECAT_IF   = ENV["ARTYHILL_ETHERCAT"]
const ECAT_PEER = ENV["ARTYHILL_ETHERCAT_PEER"]
const SOEM_DIR  = ENV["ARTYHILL_LIBSOEM_DIR"]
const NAXES     = 2
const START_POS = (Int32(12_345), Int32(-5_000))     # counts; checks bump-free enable
const SCALE     = 1000.0                              # counts per unit

function start_emulator()
    cc = something(Sys.which("cc"), Sys.which("gcc"))
    exe = joinpath(mktempdir(), "ecat_emu")
    run(`$cc -O2 -o $exe $(joinpath(@__DIR__, "..", "ethercat", "ecat_emu.c"))`)
    out = Pipe()
    proc = run(pipeline(`$exe $ECAT_PEER $NAXES $(START_POS[1]) $(START_POS[2])`; stdout=out, stderr=stderr);
               wait=false)
    close(out.in)
    @test readline(out) == "ready"
    return proc
end

signal!(proc, sig) = ccall(:kill, Cint, (Cint, Cint), getpid(proc), sig)
const SIGUSR1, SIGUSR2 = 10, 12

function cycles!(ws, ctrl, b, n)
    st = HIL_OK
    for _ in 1:n
        s = control_step!(ws, ctrl, b)
        s == HIL_OK || (st = s)
        Libc.systemsleep(0.001)
    end
    return st
end

@testset "EtherCAT via SOEM ↔ emulated CiA 402 drives ($ECAT_IF)" begin
    ArtyHill.LIBSOEM == "libsoem" || @warn "LIBSOEM preference is set; the emulator test may use another library"
    pushfirst!(Libdl.DL_LOAD_PATH, SOEM_DIR)
    proc = start_emulator()
    layout = CiA402Layout(out_stride=8, in_stride=12, controlword=0, target=2, mode_of_operation=6,
                          statusword=0, position_actual=2, velocity_actual=6, torque_actual=10)
    codec = CiA402Codec{NAXES}(layout; mode=:csp, position_scale=SCALE, velocity_scale=SCALE)
    b = EtherCATBackend(ECAT_IF; output_bytes=8 * NAXES, codec)
    ws = ControlWorkspace{3NAXES,NAXES,3NAXES}()
    target = zeros(NAXES)
    ctrl = (u, x, y) -> (for k in 1:NAXES; u[k] = target[k]; end; nothing)
    try
        @testset "scan, configure, OP" begin
            @test init_backend!(b) == HIL_OK
            @test b.slave_count == NAXES
            @test b.expected_wkc == 3NAXES             # LRW: 2 per output slave + 1 per input slave
            @test unsafe_load(b.ec_slave) == ArtyHill.EC_STATE_OPERATIONAL
        end

        @testset "state machine held in ready-to-switch-on while disabled" begin
            @test cycles!(ws, ctrl, b, 20) == HIL_OK
            @test all(k -> cia402_state(codec, k) === :ready_to_switch_on, 1:NAXES)
            @test ws.y_meas[1] ≈ START_POS[1] / SCALE
            @test ws.y_meas[4] ≈ START_POS[2] / SCALE
        end

        @testset "disabled: commanded target ignored, actual position sent" begin
            target .= 99.0
            cycles!(ws, ctrl, b, 5)
            raw_target(k) = GC.@preserve b unsafe_load(Ptr{Int32}(pointer(b.iomap) + 8(k - 1) + 2))
            @test raw_target(1) == START_POS[1]
            @test raw_target(2) == START_POS[2]
            @test ws.y_meas[1] ≈ START_POS[1] / SCALE   # nothing moved
        end

        @testset "enable is bump-free" begin
            target .= (START_POS[1] / SCALE, START_POS[2] / SCALE)
            cia402_enable!(codec, true)
            cycles!(ws, ctrl, b, 10)
            @test all(k -> cia402_state(codec, k) === :operation_enabled, 1:NAXES)
            @test ws.y_meas[1] ≈ START_POS[1] / SCALE
            @test ws.y_meas[4] ≈ START_POS[2] / SCALE
        end

        @testset "1 kHz real-time loop drives both axes to target" begin
            lim = LimitedController(ctrl, -1e3, 1e3; max_rate=2.0, dt=1e-3)   # ≤ 2 counts per cycle
            reset_limits!(lim, [ws.y_meas[1], ws.y_meas[4]])
            target .= (START_POS[1] / SCALE + 0.5, START_POS[2] / SCALE - 0.25)
            ring = SPSCRing(ws, 4096)
            stats = run_realtime_loop!(ws, lim, b,
                                       1_000_000, 1_000; telemetry=ring, priority=0, lock_memory=false,
                                       max_consecutive_errors=3)
            @test stats.errors == 0
            @test stats.abort_reason === :none
            @test ws.y_meas[1] ≈ target[1] atol = 1e-3
            @test ws.y_meas[4] ≈ target[2] atol = 1e-3
            @test length(ring) == 1_000
            @info "EtherCAT loop over veth" stats.max_latency_ns latency_percentile(stats, 0.99)
        end

        @testset "drive fault → enable latched off → manual reset" begin
            signal!(proc, SIGUSR1)
            cycles!(ws, ctrl, b, 10)
            @test all(k -> cia402_state(codec, k) === :fault, 1:NAXES)
            @test codec.enable[] == false
            cia402_fault_reset!(codec)
            cycles!(ws, ctrl, b, 10)
            @test all(k -> cia402_state(codec, k) === :ready_to_switch_on, 1:NAXES)
            pos_before = ws.y_meas[1]
            hold = (u, x, y) -> (u[1] = y[1]; u[2] = y[4]; nothing)
            cia402_enable!(codec, true)
            cycles!(ws, hold, b, 10)
            @test all(k -> cia402_state(codec, k) === :operation_enabled, 1:NAXES)
            @test ws.y_meas[1] ≈ pos_before          # re-enable is bump-free too
        end

        @testset "cable pull → watchdog aborts and sends safe command" begin
            hold = (u, x, y) -> (u[1] = y[1]; u[2] = y[4]; nothing)
            signal!(proc, SIGUSR2)                      # frames are now dropped
            stats = run_realtime_loop!(ws, hold, b, 1_000_000, 1_000; priority=0, lock_memory=false,
                                       max_consecutive_errors=3, safe_command=[ws.y_meas[1], ws.y_meas[4]])
            @test stats.abort_reason === :errors
            # One-cycle pipeline: the first read after the pull may still get the
            # frame sent just before it, then three consecutive errors trip the watchdog.
            @test stats.errors == 3
            @test stats.iterations in 3:4
            @test stats.safe_command_sent
            @test b.last_wkc < b.expected_wkc
            signal!(proc, SIGUSR2)                      # reconnect
            cycles!(ws, ctrl, b, 5)
            @test control_step!(ws, hold, b) == HIL_OK
        end

        @testset "allocation-free step against live SOEM" begin
            hold = (u, x, y) -> (u[1] = y[1]; u[2] = y[4]; nothing)
            control_step!(ws, hold, b)
            @test (@allocated control_step!(ws, hold, b)) == 0
        end
    finally
        @test close_backend!(b) == HIL_OK
        @test b.initialized == false
        signal!(proc, 15)
        wait(proc)
        filter!(!=(SOEM_DIR), Libdl.DL_LOAD_PATH)
    end
end
