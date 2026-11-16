using Test
using ArtyHill
using StaticArrays
using AllocCheck
using Libdl
using DiscretePIDs
using VeriJul

# Double integrator, dt = 1 ms, position + velocity measured.
const DT = 1e-3
make_plant() = DiscreteLinearPlant([1.0 DT; 0.0 1.0], reshape([0.5DT^2, DT], 2, 1),
                                   [1.0 0.0; 0.0 1.0], [1.0, 0.0])
make_ctrl() = StaticFeedback([40.0 12.0])       # PD: u = -40 x - 12 v

@testset "ArtyHill" begin
    @testset "interface fallbacks" begin
        struct DummyBackend <: AbstractHILBackend end
        @test_throws ArgumentError init_backend!(DummyBackend())
        @test_throws ArgumentError read_sensors!(zeros(2), DummyBackend())
    end

    @testset "simulation closed loop converges" begin
        backend = SimulationBackend(make_plant())
        ws = ControlWorkspace{2,1,2}()
        ctrl = make_ctrl()
        @test init_backend!(backend) == HIL_OK
        for _ in 1:5_000
            @test control_step!(ws, ctrl, backend) == HIL_OK
        end
        @test abs(backend.plant.x[1]) < 1e-3
        @test ws.x_hat == ws.y_meas
        @test close_backend!(backend) == HIL_OK
    end

    @testset "nonlinear plant with backlash" begin
        # Motor position θm drives load θl through a dead band of ±b.
        b = 0.01
        f(x, u) = begin
            θm, θl = x
            θm2 = θm + DT * u[1]
            d = θm2 - θl
            θl2 = d > b ? θm2 - b : d < -b ? θm2 + b : θl
            SVector(θm2, θl2)
        end
        h(x) = x
        plant = NonlinearPlant(f, h, SVector(0.0, 0.0), Val(1))
        backend = SimulationBackend(plant)
        ws = ControlWorkspace{2,1,2}()
        drive = (u, x, y) -> (u[1] = 1.0; nothing)
        init_backend!(backend)
        for _ in 1:100; control_step!(ws, drive, backend); end
        @test plant.x[1] ≈ 0.1
        @test plant.x[2] ≈ 0.1 - b
        @test isempty(check_step_allocs(ws, drive, backend))
    end

    @testset "zero allocations" begin
        backend = SimulationBackend(make_plant())
        ws = ControlWorkspace{2,1,2}()
        ctrl = make_ctrl()
        @test isempty(check_step_allocs(ws, ctrl, backend))
        ring = SPSCRing(ws, 64)
        @test isempty(check_loop_allocs(ws, ctrl, backend))
        @test isempty(check_loop_allocs(ws, ctrl, backend; telemetry=ring))
        control_step!(ws, ctrl, backend)  # warm up
        @test (@allocated control_step!(ws, ctrl, backend)) == 0

        can = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x200, heartbeat_id=0x701)
        @test isempty(check_step_allocs(ws, ctrl, can))
        @test isempty(check_loop_allocs(ws, ctrl, can))
        canfd = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x200, canfd=true)
        @test isempty(check_step_allocs(ws, ctrl, canfd))
        lim = LimitedController(ctrl, -1.0, 1.0; max_rate=100.0, dt=1e-3)
        @test isempty(check_loop_allocs(ws, lim, backend; telemetry=ring))
        ws6 = ControlWorkspace{6,2,6}()
        cia = EtherCATBackend("eth0"; output_bytes=16,
                              codec=ArtyHill.CiA402Codec{2}(CiA402Layout(out_stride=8, in_stride=10)))
        @test isempty(check_step_allocs(ws6, (u, x, y) -> nothing, cia))
        ecat = EtherCATBackend("eth0"; output_bytes=8)
        @test isempty(check_step_allocs(ws, ctrl, ecat))
    end

    @testset "uninitialized hardware backends report errors" begin
        ws = ControlWorkspace{2,1,2}()
        can = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x200)
        @test control_step!(ws, make_ctrl(), can) == HIL_ERR_NOT_INIT
        ecat = EtherCATBackend("eth0"; output_bytes=8)
        if Libdl.dlopen(ArtyHill.LIBSOEM; throw_error=false) === nothing
            @test init_backend!(ecat) == HIL_ERR_SYS   # SOEM not installed here
        end
        @test control_step!(ws, make_ctrl(), ecat) == HIL_ERR_NOT_INIT
    end

    @testset "CAN frame layout and codec" begin
        @test sizeof(ArtyHill.CanFrame) == 16
        @test sizeof(ArtyHill.SockaddrCan) == 24
        c = ArtyHill.Float64Codec()
        @test ArtyHill.decode_can(c, ArtyHill.encode_can(c, -3.25), 0x08) == -3.25
    end

    @testset "SPSC ring" begin
        r = SPSCRing{Int}(5)
        @test capacity(r) == 8
        @test try_pop!(r) === nothing
        for i in 1:8; @test try_push!(r, i); end
        @test !try_push!(r, 9)
        @test r.dropped == 1
        @test try_pop!(r) == 1
        out = Int[]
        @test drain!(x -> push!(out, x), r) == 7
        @test out == 2:8
        @test isempty(r)

        # Concurrent producer/consumer preserves order.
        if Threads.nthreads() > 1
            r2 = SPSCRing{Int}(16); n = 100_000; got = Int[]
            t = Threads.@spawn for i in 1:n
                while !try_push!(r2, i); end
            end
            while length(got) < n
                x = try_pop!(r2); x === nothing || push!(got, x)
            end
            wait(t)
            @test got == 1:n
        end
    end

    @testset "LimitedController" begin
        raw = (u, x, y) -> (u[1] = y[1]; u[2] = y[2]; nothing)
        c = LimitedController(raw, [-1.0, 0.0], [1.0, 10.0]; max_rate=[1000.0, Inf], dt=1e-3)
        u = zeros(2); x = zeros(2)
        c(u, x, [5.0, 5.0]);   @test u ≈ [1.0, 5.0]        # rate 1/step on axis 1, clamp
        c(u, x, [5.0, 50.0]);  @test u ≈ [1.0, 10.0]
        c(u, x, [-5.0, -3.0]); @test u ≈ [0.0, 0.0]        # rate-limited from 1 → 0
        c(u, x, [NaN, Inf]);   @test u ≈ [0.0, 0.0]        # non-finite → hold previous
        reset_limits!(c, [0.5, 2.0])
        c(u, x, [0.5, NaN]);   @test u ≈ [0.5, 2.0]
        @test_throws ArgumentError LimitedController(raw, 1.0, -1.0)
    end

    @testset "watchdog and safe command" begin
        # Backend that records the last command and fails after `fail_after` writes.
        mutable struct FlakyBackend <: AbstractHILBackend
            writes::Int
            fail_after::Int
            last_u::Float64
        end
        ArtyHill.init_backend!(::FlakyBackend) = HIL_OK
        ArtyHill.close_backend!(::FlakyBackend) = HIL_OK
        ArtyHill.read_sensors!(y, ::FlakyBackend) = (y .= 0; HIL_OK)
        function ArtyHill.write_actuators!(u, b::FlakyBackend)
            b.writes += 1; b.last_u = u[1]
            return b.writes > b.fail_after ? HIL_ERR_IO : HIL_OK
        end
        ws = ControlWorkspace{1,1,1}()
        push1 = (u, x, y) -> (u[1] = 7.0; nothing)

        b = FlakyBackend(0, 20, 0.0)
        st = run_realtime_loop!(ws, push1, b, 200_000, 1_000; priority=0, lock_memory=false,
                                max_consecutive_errors=3, safe_command=[0.0])
        @test st.abort_reason === :errors
        @test st.iterations == 23
        @test st.safe_command_sent == false        # backend still failing → reported, not hidden
        @test b.last_u == 0.0

        b = FlakyBackend(0, typemax(Int), 0.0)
        st = run_realtime_loop!(ws, push1, b, 200_000, 50; priority=0, lock_memory=false, safe_command=[-1.0])
        @test st.abort_reason === :none && st.safe_command_sent && b.last_u == -1.0

        # Controller exception: safe command still sent, GC re-enabled, error rethrown.
        n = Ref(0)
        boom = (u, x, y) -> (n[] += 1; n[] == 5 && error("controller blew up"); u[1] = 3.0; nothing)
        b = FlakyBackend(0, typemax(Int), 0.0)
        @test_throws ErrorException run_realtime_loop!(ws, boom, b, 200_000, 50; priority=0,
                                                       lock_memory=false, safe_command=[0.0])
        @test b.last_u == 0.0
        @test GC.enable(true)

        @test_throws DimensionMismatch run_realtime_loop!(ws, push1, b, 200_000, 5; priority=0,
                                                          lock_memory=false, safe_command=[0.0, 0.0])
    end

    @testset "latency histogram" begin
        s = LoopStats(; hist_bucket_ns=1_000, hist_buckets=10)
        for v in [500, 1_500, 1_500, 2_500, 50_000]
            ArtyHill._hist_add!(s.latency_hist, s.hist_bucket_ns, v)
        end
        @test latency_percentile(s, 0.2) == 1_000
        @test latency_percentile(s, 0.6) == 2_000
        @test latency_percentile(s, 0.8) == 3_000
        @test latency_percentile(s, 1.0) == typemax(Int64)   # overflow bucket
        @test latency_percentile(s, 0.5; kind=:wake) == 0
    end

    @testset "CiA 402 codec" begin
        @test cia402_state(0x0250) === :switch_on_disabled
        @test cia402_state(0x0231) === :ready_to_switch_on
        @test cia402_state(0x0233) === :switched_on
        @test cia402_state(0x0237) === :operation_enabled
        @test cia402_state(0x0218) === :fault
        @test cia402_state(0x0217) === :quick_stop

        L = CiA402Layout(out_stride=8, in_stride=12, mode_of_operation=6, torque_actual=10)
        c = CiA402Codec{1}(L; mode=:csp, position_scale=1000.0, velocity_scale=10.0, torque_scale=1.0)
        outb = zeros(UInt8, 8); inb = zeros(UInt8, 12); y = zeros(3)
        setin!(sw, pos, vel, tq) = GC.@preserve inb begin
            p = pointer(inb)
            unsafe_store!(Ptr{UInt16}(p), sw); unsafe_store!(Ptr{Int32}(p + 2), pos)
            unsafe_store!(Ptr{Int32}(p + 6), vel); unsafe_store!(Ptr{Int16}(p + 10), tq)
        end
        cycle!(u) = GC.@preserve inb outb begin
            ArtyHill.decode_inputs!(y, c, pointer(inb)); ArtyHill.encode_outputs!(pointer(outb), c, u)
            (unsafe_load(Ptr{UInt16}(pointer(outb))), unsafe_load(Ptr{Int32}(pointer(outb) + 2)),
             unsafe_load(Ptr{Int8}(pointer(outb) + 6)))
        end

        setin!(0x0250, Int32(1234), Int32(50), Int16(-7))
        cw, tgt, mode = cycle!([9.0])
        @test cw == 0x0006 && mode == 8
        @test tgt == 1234                          # disabled: target tracks actual position
        @test y ≈ [1.234, 5.0, -7.0]

        cia402_enable!(c, true)
        setin!(0x0231, Int32(1234), Int32(0), Int16(0)); @test cycle!([9.0])[1] == 0x0007
        setin!(0x0233, Int32(1234), Int32(0), Int16(0)); @test cycle!([9.0])[1:2] == (0x000f, 1234)
        setin!(0x0237, Int32(1234), Int32(0), Int16(0)); @test cycle!([2.5])[1:2] == (0x000f, 2500)
        @test cycle!([1e12])[2] == typemax(Int32)  # saturating, no InexactError
        @test cycle!([NaN])[2] == 0

        setin!(0x0218, Int32(0), Int32(0), Int16(0)); @test cycle!([0.0])[1] == 0x0000  # no auto reset
        c2 = CiA402Codec{1}(L; auto_fault_reset=true)
        GC.@preserve inb begin ArtyHill.decode_inputs!(y, c2, pointer(inb)) end
        @test c2.controlword[1] == 0x0080
        cia402_enable!(c, false)
        setin!(0x0237, Int32(10), Int32(0), Int16(0)); @test cycle!([2.5])[1:2] == (0x0006, 10)
    end

    @testset "SocketCAN options" begin
        @test sizeof(ArtyHill.CanFDFrame) == 72
        b = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x200, rx_count=3)
        f = ArtyHill.can_filters(b)
        @test [x.can_id for x in f] == [0x200, 0x201, 0x202]
        @test all(x -> x.can_mask == 0x7ff | 0x80000000 | 0x40000000, f)
        bx = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x18ff0000, extended=true, rx_count=1)
        @test ArtyHill.can_filters(bx)[1].can_id == 0x18ff0000 | 0x80000000
        @test isempty(ArtyHill.can_filters(SocketCANBackend("vcan0"; tx_base_id=1, rx_base_id=2)))
        @test heartbeat!(b, 1) == HIL_OK          # heartbeat disabled → no-op even uninitialized
        @test_throws ArgumentError SocketCANBackend("vcan0"; tx_base_id=1, rx_base_id=2, heartbeat_every=0)
    end

    @testset "real-time loop (soft mode)" begin
        backend = SimulationBackend(make_plant())
        ws = ControlWorkspace{2,1,2}()
        ctrl = make_ctrl()
        init_backend!(backend)
        ring = SPSCRing(ws, 4096)
        stats = run_realtime_loop!(ws, ctrl, backend, 1_000_000, 500;
                                   telemetry=ring, priority=0, lock_memory=false)
        @test stats.iterations == 500
        @test stats.errors == 0
        @test GC.enable(true)            # GC state restored (was enabled)
        @test length(ring) == 500
        samples = TelemetrySample{2,1}[]
        drain!(s -> push!(samples, s), ring)
        dts = diff([s.t_ns for s in samples])
        @test abs(sum(dts) / length(dts) - 1_000_000) < 50_000   # no cumulative drift
        io = IOBuffer(); write_telemetry_csv(io, samples)
        @test startswith(String(take!(io)), "t_ns,wake_late_ns,latency_ns,status,overrun,y1,y2,u1\n")
        @test 0 < latency_percentile(stats, 0.99) <= stats.max_latency_ns + 1_000
        @test stats.abort_reason === :none
        show(devnull, MIME"text/plain"(), stats)
    end

    @testset "DiscretePIDs extension" begin
        pid = DiscretePID(; K=40.0, Ti=2.0, Td=0.3, Ts=DT, umin=-100.0, umax=100.0)
        bank = pid_bank(pid; setpoints=[0.0], inputs=[1])
        backend = SimulationBackend(make_plant())
        ws = ControlWorkspace{2,1,2}()
        init_backend!(backend)
        for _ in 1:8_000; control_step!(ws, bank, backend); end
        @test abs(backend.plant.x[1]) < 1e-2
        bank.setpoint[1] = 0.5
        for _ in 1:15_000; control_step!(ws, bank, backend); end
        @test backend.plant.x[1] ≈ 0.5 atol = 1e-2
        @test isempty(check_loop_allocs(ws, bank, backend))
        @test_throws DimensionMismatch pid_bank(pid; setpoints=[0.0, 1.0])
    end

    @testset "VeriJul co-simulation backend (mock RTL)" begin
        cc = something(Sys.which("cc"), Sys.which("gcc"), Sys.which("clang"), Some(nothing))
        if cc === nothing
            @info "Skipping VeriJul backend test: no C compiler"
        else
            dir = mktempdir()
            lib = joinpath(dir, "libaccumulator." * Libdl.dlext)
            run(`$cc -shared -fPIC -O2 -o $lib $(joinpath(@__DIR__, "mock", "accumulator.c"))`)
            ports = [VeriJul.HardwarePort("clk", :in, 1), VeriJul.HardwarePort("u", :in, 16),
                     VeriJul.HardwarePort("y", :out, 16)]
            dut = VeriJul.DUT(lib, "accumulator", ports)
            b = verijul_backend(dut; inputs=[:u], outputs=[:y], cycles_per_step=2,
                                input_scale=100.0, output_scale=100.0)
            ws = ControlWorkspace{1,1,1}()
            @test control_step!(ws, (u, x, y) -> nothing, b) == HIL_ERR_NOT_INIT
            @test init_backend!(b) == HIL_OK
            cmd = Ref(0.5)
            ctrl = (u, x, y) -> (u[1] = cmd[]; nothing)
            control_step!(ws, ctrl, b)                    # y: 0 → 2 clocks × 50 = 100 raw
            control_step!(ws, ctrl, b); @test ws.y_meas[1] ≈ 1.0
            cmd[] = -1.5
            control_step!(ws, ctrl, b)                    # +100 then 2 × -150
            control_step!(ws, ctrl, b); @test ws.y_meas[1] ≈ -1.0   # signed 16-bit readback
            @test b.time > 0
            # Wrap-around like real 16-bit hardware
            cmd[] = 300.0
            for _ in 1:3; control_step!(ws, ctrl, b); end
            @test -327.68 <= ws.y_meas[1] < 327.68
            @test_throws ArgumentError verijul_backend(dut; inputs=[:y], outputs=[:y])
            @test close_backend!(b) == HIL_OK
            VeriJul.destroy!(dut)
        end
    end

    if haskey(ENV, "ARTYHILL_VCAN")
        include("hardware/vcan.jl")
    else
        @info "Skipping SocketCAN integration tests (set ARTYHILL_VCAN=vcan0 to run them)"
    end
end
