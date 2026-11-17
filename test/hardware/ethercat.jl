# EtherCAT end-to-end: the real SOEM 1.4 master (built from source) talks over a
# veth pair to `test/ethercat/ecat_emu.c`, which emulates a line of CiA 402
# servo drives at the frame level (ESC registers, SII, CoE mailbox, DC).
# Runs only when ARTYHILL_ETHERCAT names the master side of a veth pair whose
# peer is ARTYHILL_ETHERCAT_PEER, ARTYHILL_LIBSOEM_DIR holds libsoem.so and
# ARTYHILL_SOEM_SRC the SOEM sources, and the process has CAP_NET_RAW.
# `test/ethercat/run.sh` sets all of that up in an unprivileged namespace.

const ECAT_IF   = ENV["ARTYHILL_ETHERCAT"]
const ECAT_PEER = ENV["ARTYHILL_ETHERCAT_PEER"]
const SOEM_DIR  = ENV["ARTYHILL_LIBSOEM_DIR"]
const SOEM_SRC  = get(ENV, "ARTYHILL_SOEM_SRC", "")
const NAXES     = 2
const START_POS = (Int32(12_345), Int32(-5_000))     # counts; checks bump-free enable
const SCALE     = 1000.0                              # counts per unit
const CYCLE_NS  = 1_000_000
const PHASE_NS  = 500_000                             # frames should hit mid-cycle
const SIGUSR1, SIGUSR2, SIGTERM = 10, 12, 15
const CC = something(Sys.which("cc"), Sys.which("gcc"))
const EMU = let exe = joinpath(mktempdir(), "ecat_emu")
    run(`$CC -O2 -o $exe $(joinpath(@__DIR__, "..", "ethercat", "ecat_emu.c"))`)
    exe
end

function start_emulator(flags...)
    out = Pipe()
    proc = run(pipeline(`$EMU $flags $ECAT_PEER $NAXES $(START_POS[1]) $(START_POS[2])`;
                        stdout=out, stderr=stderr); wait=false)
    close(out.in)
    @test readline(out) == "ready"
    return proc
end
signal!(proc, sig) = ccall(:kill, Cint, (Cint, Cint), getpid(proc), sig)
stop_emulator(proc) = (signal!(proc, SIGTERM); wait(proc))

function cycles!(ws, ctrl, b, n)
    st = HIL_OK
    for _ in 1:n
        s = control_step!(ws, ctrl, b)
        s == HIL_OK || (st = s)
        Libc.systemsleep(0.001)
    end
    return st
end

# Custom mapping programmed over CoE: no mode object in the RxPDO (mode is set by
# SDO instead), and the emulator's sync-phase object appended to the TxPDO.
const RX_MAP = [PDOEntry(0x6040, 0, 16), PDOEntry(0x607A, 0, 32)]
const TX_MAP = [PDOEntry(0x6041, 0, 16), PDOEntry(0x6064, 0, 32), PDOEntry(0x606C, 0, 32),
                PDOEntry(0x2000, 1, 32)]

pushfirst!(Libdl.DL_LOAD_PATH, SOEM_DIR)
ArtyHill.LIBSOEM == "libsoem" || @warn "LIBSOEM preference is set; the emulator test may use another library"

@testset "EtherCAT via SOEM ↔ emulated CiA 402 drives ($ECAT_IF)" begin
    if !isempty(SOEM_SRC)
        @testset "ec_errort layout matches SOEM headers" begin
            src = joinpath(mktempdir(), "layout.c")
            write(src, """
                #include <stdio.h>
                #include <stddef.h>
                #include "ethercat.h"
                int main(void) {
                    printf("%zu %zu %zu %zu %zu %zu %zu\\n", sizeof(ec_errort), offsetof(ec_errort, Signal),
                           offsetof(ec_errort, Slave), offsetof(ec_errort, Index), offsetof(ec_errort, SubIdx),
                           offsetof(ec_errort, Etype), offsetof(ec_errort, AbortCode));
                    printf("%zu\\n", offsetof(ec_slavet, state));
                    printf("%d\\n", (int)EC_ERR_TYPE_SDO_ERROR);
                    return 0;
                }""")
            exe = src[1:end-2]
            inc = ["-I$(joinpath(SOEM_SRC, d))" for d in ("soem", "osal", "osal/linux", "oshw/linux")]
            run(`$CC $inc -o $exe $src`)
            lines = readlines(`$exe`)
            c = parse.(Int, split(lines[1]))
            T = ArtyHill.ECError
            jl = [sizeof(T); [Int(fieldoffset(T, i)) for i in (3, 4, 5, 6, 7, 8)]]
            @test c == jl
            @test parse(Int, lines[2]) == 0          # ec_slave[0].state is written via a raw pointer
            @test parse(Int, lines[3]) == ArtyHill.EC_ERR_TYPE_SDO_ERROR
        end
    end

    @testset "SII-only slaves (no mailbox, no DC requirement)" begin
        proc = start_emulator("--sii")
        b = EtherCATBackend(ECAT_IF; output_bytes=8NAXES)
        try
            @test init_backend!(b) == HIL_OK
            @test b.slave_count == NAXES
            err = try sdo_read(b, 1, 0x1018, 1, UInt32); nothing catch e; e end
            @test err isa SDOError && err.abort_code == 0     # no CoE mailbox → no answer
            @test occursin("no response", sprint(showerror, err))
        finally
            close_backend!(b)
            stop_emulator(proc)
        end
    end

    proc = start_emulator("--dc-required")
    try
        @testset "drives refuse OP without DC Sync0" begin
            b = EtherCATBackend(ECAT_IF; output_bytes=8NAXES)
            @test init_backend!(b) == HIL_ERR_IO
            @test b.initialized == false
        end

        sdo_log = Dict{Symbol,Any}()
        function preop!(b, slave)
            sdo_log[Symbol(:vendor, slave)] = sdo_read(b, slave, 0x1018, 1, UInt32)
            sdo_write!(b, slave, 0x6065, 0, UInt32(300_000))          # following-error window
            sdo_log[Symbol(:window, slave)] = sdo_read(b, slave, 0x6065, 0, UInt32)
            sdo_write!(b, slave, 0x6060, 0, Int8(8))                   # CSP, set by SDO
            if slave == 1   # expected failures, each with the slave's abort code
                for (key, f) in ((:ro,      () -> sdo_write!(b, slave, 0x1018, 1, UInt32(1))),
                                 (:missing, () -> sdo_read(b, slave, 0x5000, 0, UInt32)),
                                 (:size,    () -> sdo_write!(b, slave, 0x6065, 0, UInt16(1))),
                                 (:range,   () -> sdo_write!(b, slave, 0x6060, 0, Int8(3))),
                                 (:map,     () -> sdo_write!(b, slave, 0x1600, 1, UInt32(0x10180120))))
                    sdo_log[key] = try f(); nothing catch e; e end
                end
            end
            sdo_log[Symbol(:rxbytes, slave)] = configure_pdo!(b, slave, 0x1C12, 0x1600, RX_MAP)
            sdo_log[Symbol(:txbytes, slave)] = configure_pdo!(b, slave, 0x1C13, 0x1A00, TX_MAP)
        end

        layout = CiA402Layout(RX_MAP, TX_MAP)
        codec = CiA402Codec{NAXES}(layout; mode=:csp, position_scale=SCALE, velocity_scale=SCALE)
        b = EtherCATBackend(ECAT_IF; output_bytes=layout.out_stride * NAXES, codec, preop_config=preop!,
                            dc_cycle_ns=CYCLE_NS, dc_frame_phase_ns=PHASE_NS)
        ws = ControlWorkspace{3NAXES,NAXES,3NAXES}()
        target = zeros(NAXES)
        ctrl = (u, x, y) -> (for k in 1:NAXES; u[k] = target[k]; end; nothing)
        hold = (u, x, y) -> (u[1] = y[1]; u[2] = y[4]; nothing)
        try
            @testset "CoE configuration in PRE-OP, DC Sync0, OP" begin
                @test init_backend!(b) == HIL_OK
                @test b.slave_count == NAXES
                @test unsafe_load(b.ec_slave) == ArtyHill.EC_STATE_OPERATIONAL
                @test sdo_log[:vendor1] == 0x4152 && sdo_log[:vendor2] == 0x4152
                @test sdo_log[:window1] == 300_000
                @test sdo_log[:ro].abort_code == 0x06010002
                @test sdo_log[:missing].abort_code == 0x06020000
                @test sdo_log[:size].abort_code == 0x06070010
                @test sdo_log[:range].abort_code == 0x06090030
                @test sdo_log[:map].abort_code == 0x06040041
                @test occursin("read-only", sprint(showerror, sdo_log[:ro]))
                @test sdo_log[:rxbytes1] == 6 && sdo_log[:txbytes1] == 14
                @test layout.out_stride == 6 && layout.in_stride == 14 && layout.mode_of_operation == -1
                @test b.expected_wkc == 3NAXES
                # PDO objects are writable in PRE-OP only
                err = try sdo_write!(b, 1, 0x1600, 0, UInt8(0)); nothing catch e; e end
                @test err isa SDOError && err.abort_code == 0x08000022
                @test sdo_read(b, 2, 0x6060, 0, Int8) == 8
            end

            @testset "disabled: commanded target ignored, actual position sent" begin
                target .= 99.0
                @test cycles!(ws, ctrl, b, 20) == HIL_OK
                @test all(k -> cia402_state(codec, k) === :ready_to_switch_on, 1:NAXES)
                raw_target(k) = GC.@preserve b unsafe_load(Ptr{Int32}(pointer(b.iomap) + 6(k - 1) + 2))
                @test raw_target(1) == START_POS[1]
                @test raw_target(2) == START_POS[2]
                @test ws.y_meas[1] ≈ START_POS[1] / SCALE
                @test ws.y_meas[4] ≈ START_POS[2] / SCALE
            end

            @testset "enable is bump-free" begin
                cia402_enable!(codec, true)
                cycles!(ws, hold, b, 10)
                @test all(k -> cia402_state(codec, k) === :operation_enabled, 1:NAXES)
                @test ws.y_meas[1] ≈ START_POS[1] / SCALE
                @test ws.y_meas[4] ≈ START_POS[2] / SCALE
            end

            @testset "1 kHz loop phase-locked to the DC clock drives both axes" begin
                lim = LimitedController(ctrl, -1e3, 1e3; max_rate=2.0, dt=1e-3)   # ≤ 2 counts per cycle
                reset_limits!(lim, [ws.y_meas[1], ws.y_meas[4]])
                target .= (START_POS[1] / SCALE + 0.5, START_POS[2] / SCALE - 0.25)
                ring = SPSCRing(ws, 4096)
                stats = run_realtime_loop!(ws, lim, b, CYCLE_NS, 1_500; telemetry=ring, priority=0,
                                           lock_memory=false, max_consecutive_errors=3)
                @test stats.errors == 0 && stats.abort_reason === :none
                @test ws.y_meas[1] ≈ target[1] atol = 1e-3
                @test ws.y_meas[4] ≈ target[2] atol = 1e-3
                @test length(ring) == 1_500

                # Now locked: measure where frames land relative to Sync0 (emulator's view).
                sdo_write!(b, 1, 0x2000, 2, UInt32(0))                   # reset phase statistics
                run_realtime_loop!(ws, hold, b, CYCLE_NS, 1_000; priority=0, lock_memory=false)
                mean_phase = Int(sdo_read(b, 1, 0x2000, 5, UInt32))
                frames = sdo_read(b, 1, 0x2000, 4, UInt32)
                @info "DC phase lock" target_ns = PHASE_NS mean_phase_ns = mean_phase frames dc_phase_error_ns(b)
                @test frames >= 900
                @test abs(mean_phase - PHASE_NS) < 150_000
                @test dc_time(b) > 0
                # the phase is also mapped into the TxPDO: read it straight from the image
                pdo_phase = GC.@preserve b unsafe_load(Ptr{UInt32}(pointer(b.iomap) + b.output_bytes + 10))
                @test 0 <= pdo_phase < CYCLE_NS
            end

            @testset "drive fault → enable latched off → manual reset" begin
                signal!(proc, SIGUSR1)
                cycles!(ws, hold, b, 10)
                @test all(k -> cia402_state(codec, k) === :fault, 1:NAXES)
                @test codec.enable[] == false
                cia402_fault_reset!(codec)
                cycles!(ws, hold, b, 10)
                @test all(k -> cia402_state(codec, k) === :ready_to_switch_on, 1:NAXES)
                pos_before = ws.y_meas[1]
                cia402_enable!(codec, true)
                cycles!(ws, hold, b, 10)
                @test all(k -> cia402_state(codec, k) === :operation_enabled, 1:NAXES)
                @test ws.y_meas[1] ≈ pos_before          # re-enable is bump-free too
            end

            @testset "cable pull → watchdog aborts and sends safe command" begin
                signal!(proc, SIGUSR2)                      # frames are now dropped
                stats = run_realtime_loop!(ws, hold, b, CYCLE_NS, 1_000; priority=0, lock_memory=false,
                                           max_consecutive_errors=3, safe_command=[ws.y_meas[1], ws.y_meas[4]])
                @test stats.abort_reason === :errors
                # One-cycle pipeline: the first read after the pull may still get the
                # frame sent just before it, then three consecutive errors trip the watchdog.
                @test stats.errors == 3
                @test stats.iterations in 3:4
                @test stats.safe_command_sent
                @test b.last_wkc < b.expected_wkc
                signal!(proc, SIGUSR2)                      # reconnect
                cycles!(ws, hold, b, 5)
                @test control_step!(ws, hold, b) == HIL_OK
            end

            @testset "allocation-free step and DC correction against live SOEM" begin
                control_step!(ws, hold, b)
                corr_allocs(b) = (cycle_correction_ns(b, Int64(CYCLE_NS)); @allocated cycle_correction_ns(b, Int64(CYCLE_NS)))
                @test (@allocated control_step!(ws, hold, b)) == 0
                @test corr_allocs(b) == 0
            end
        finally
            @test close_backend!(b) == HIL_OK
            @test b.initialized == false
        end
    finally
        stop_emulator(proc)
    end
end

filter!(!=(SOEM_DIR), Libdl.DL_LOAD_PATH)
