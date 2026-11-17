# Changelog

## Unreleased (0.1.0)

First release.

### Hardware abstraction
- `SimulationBackend` with `DiscreteLinearPlant` and `NonlinearPlant`.
- `SocketCANBackend`: classic CAN and CAN FD, kernel receive filters, CANopen-style heartbeat.
- `EtherCATBackend` on SOEM 1.4:
  - CoE/SDO access (`sdo_read`, `sdo_write!`, `SDOError` with abort codes);
  - a PRE-OP configuration hook, and PDO remapping with `configure_pdo!`;
  - DC Sync0, with the loop phase-locked to the DC reference clock.
- `CiA402Codec`: drive state machine, CSP/CSV/CST, bump-free enable, fault latch and
  `cia402_fault_reset!`, and layouts derived from PDO mappings via `CiA402Layout(rx, tx)`.

### Runtime
- Zero-allocation `control_step!` and `run_realtime_loop!`:
  - `mlockall`, CPU pinning, `SCHED_FIFO`, GC pause, absolute-time sleep;
  - latency histograms and a lock-free telemetry ring;
  - a stop flag, and `spawn_realtime_loop` to run on a dedicated thread.

### Safety
- `LimitedController` (clamp, rate limit, NaN guard).
- Watchdog on consecutive overruns/errors, with the safe command sent on every exit.
- `heartbeat!` and `cycle_correction_ns` backend hooks.

### Verification and code generation
- AllocCheck proofs (`check_step_allocs`, `check_loop_allocs`).
- StaticCompiler (`compile_controller`) and juliac (`juliac_build`) code generation.

### Extensions
- DiscretePIDs (`pid_bank`), VeriJul RTL co-simulation (`verijul_backend`) and
  KaosCleaner telemetry cleaning (`clean_telemetry`).

### Testing
- CI covers Julia 1.10–1.12 with Aqua, JET and Codecov, plus:
  - SocketCAN on `vcan0`;
  - EtherCAT against real SOEM and a frame-level drive emulator;
  - real Verilator RTL.
