using Test
using ArtyHill
using StaticArrays
using AllocCheck
using Libdl

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

        can = SocketCANBackend("vcan0"; tx_base_id=0x100, rx_base_id=0x200)
        @test isempty(check_step_allocs(ws, ctrl, can))
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
        @test startswith(String(take!(io)), "t_ns,wake_late_ns,latency_ns,status,y1,y2,u1\n")
        show(devnull, MIME"text/plain"(), stats)
    end
end
