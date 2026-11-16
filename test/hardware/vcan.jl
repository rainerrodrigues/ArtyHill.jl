# Real SocketCAN I/O against a virtual CAN interface. Runs only when
# ARTYHILL_VCAN names an interface that is up, e.g.:
#
#   sudo modprobe vcan
#   sudo ip link add dev vcan0 type vcan
#   sudo ip link set vcan0 mtu 72 up          # mtu 72 enables CAN FD
#   ARTYHILL_VCAN=vcan0 julia --project -e 'using Pkg; Pkg.test()'
#
# Two backends share the bus with mirrored ids, so what one writes the other reads.

const VCAN = ENV["ARTYHILL_VCAN"]

function settle_read!(y, b; tries=200)
    for _ in 1:tries
        read_sensors!(y, b)
        any(!iszero, y) && return true
        Libc.systemsleep(0.001)
    end
    return false
end

@testset "SocketCAN on $VCAN" begin
    @testset "classic CAN round trip + filters" begin
        a = SocketCANBackend(VCAN; tx_base_id=0x100, rx_base_id=0x200, rx_count=2)
        b = SocketCANBackend(VCAN; tx_base_id=0x200, rx_base_id=0x100, rx_count=3)
        noise = SocketCANBackend(VCAN; tx_base_id=0x300, rx_base_id=0x7f0)
        @test init_backend!(a) == HIL_OK
        @test init_backend!(b) == HIL_OK
        @test init_backend!(noise) == HIL_OK
        try
            # Unrelated traffic must not disturb the reader (kernel filter drops it).
            @test write_actuators!([99.0, 99.0], noise) == HIL_OK
            @test write_actuators!([1.5, -2.25, 1e-3], a) == HIL_OK
            yb = zeros(3)
            @test settle_read!(yb, b)
            sleep(0.01); read_sensors!(yb, b)
            @test yb == [1.5, -2.25, 1e-3]

            @test write_actuators!([4.0, 8.0], b) == HIL_OK
            ya = zeros(2)
            @test settle_read!(ya, a)
            @test ya == [4.0, 8.0]

            # Allocation-free on a live socket, too.
            write_actuators!([1.0, 2.0], b); read_sensors!(ya, a)
            u = [1.0, 2.0]
            @test (@allocated write_actuators!(u, b)) == 0
            @test (@allocated read_sensors!(ya, a)) == 0
        finally
            close_backend!(a); close_backend!(b); close_backend!(noise)
        end
        @test a.socket_fd == -1
    end

    @testset "heartbeat frame" begin
        hb = SocketCANBackend(VCAN; tx_base_id=0x100, rx_base_id=0x200, heartbeat_id=0x701, heartbeat_every=1)
        mon = SocketCANBackend(VCAN; tx_base_id=0x000, rx_base_id=0x701,
                               codec=ArtyHill.Float64Codec())
        @test init_backend!(hb) == HIL_OK && init_backend!(mon) == HIL_OK
        try
            @test heartbeat!(hb, 1) == HIL_OK
            sleep(0.01)
            n = ccall(:read, Cssize_t, (Cint, Ptr{ArtyHill.CanFrame}, Csize_t),
                      mon.socket_fd, mon.frame, sizeof(ArtyHill.CanFrame))
            @test n == 16
            @test mon.frame[].can_id == 0x701 && mon.frame[].can_dlc == 1 && mon.frame[].data[1] == 0x05
        finally
            close_backend!(hb); close_backend!(mon)
        end
    end

    mtu = tryparse(Int, strip(read("/sys/class/net/$VCAN/mtu", String)))
    if mtu == 72
        @testset "CAN FD round trip" begin
            a = SocketCANBackend(VCAN; tx_base_id=0x120, rx_base_id=0x220, canfd=true, rx_count=1)
            b = SocketCANBackend(VCAN; tx_base_id=0x220, rx_base_id=0x120, canfd=true, rx_count=1)
            @test init_backend!(a) == HIL_OK && init_backend!(b) == HIL_OK
            try
                u = collect(1.0:6.0)
                @test write_actuators!(u, a) == HIL_OK
                y = zeros(6)
                @test settle_read!(y, b)
                @test y == u
            finally
                close_backend!(a); close_backend!(b)
            end
        end
    else
        @info "Skipping CAN FD test: set `ip link set $VCAN mtu 72` to enable it"
    end
end
