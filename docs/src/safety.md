# Safety

!!! warning
    These features reduce risk; they do not make a system safe. Always use
    hardware interlocks: an emergency stop that cuts power, and the drive's own
    limits and watchdog.

## Actuator limits

[`LimitedController`](@ref) wraps any controller. Non-finite commands are
replaced by the previous command, each command's change per step is limited,
and the result is clamped to `[umin, umax]`.

## Watchdog and safe command

```julia
cfg = RTConfig(; period_ns=1_000_000, iterations=10^6,
               max_consecutive_overruns=5, max_consecutive_errors=3)
stats = run_realtime_loop!(ws, ctrl, backend, cfg; safe_command=[0.0])
stats.abort_reason      # :none, :overruns, :errors or :exception
stats.safe_command_sent # false if even the safe command failed to reach the drive
```

`safe_command` is written on **every** exit: normal completion, a watchdog trip,
or an exception from the controller or backend (the exception is rethrown
afterwards). Call [`send_safe_command!`](@ref) yourself from an emergency-stop
handler.

## Drive-side watchdog

Implement [`heartbeat!`](@ref) for your backend, or use `heartbeat_id` on
[`SocketCANBackend`](@ref), so the drive disables itself if the master stops.
On EtherCAT, configure the slave's SyncManager watchdog.
