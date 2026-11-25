# Tutorial: PREEMPT_RT latency

How late does ArtyHill's loop wake up, in the worst case, on a properly set up
real-time Linux machine? This tutorial sets one up and measures it next to
`cyclictest`, the standard kernel latency benchmark, under the same settings
and the same background load. The difference between the two is what Julia and
ArtyHill add on top of the kernel.

## What you need

- A **native Linux machine** you can reboot and change the kernel of. WSL2 and
  most VMs run the host's scheduler underneath and give meaningless numbers.
- At least 4 CPU cores: one is set aside for the loop, the rest carry the
  system and the background load. The examples use CPU 3; pick the last core
  of your machine.
- Julia ≥ 1.10 and sudo rights.

## 1. Install a PREEMPT_RT kernel

PREEMPT_RT makes almost all of the kernel preemptible, so a high-priority
thread runs within microseconds of becoming ready. Since Linux 6.12 it is part
of the mainline kernel; distributions ship it as a separate package:

- **Debian 13:** `sudo apt install linux-image-rt-amd64` (or `-rt-arm64`).
- **Ubuntu 24.04 LTS:** `sudo pro attach` (Ubuntu Pro is free for personal
  use), then `sudo pro enable realtime-kernel`.
- **Raspberry Pi 5 and others:** build a kernel ≥ 6.12 with `CONFIG_PREEMPT_RT=y`.

Reboot into it and check:

```bash
cat /sys/kernel/realtime    # 1
uname -v                    # contains PREEMPT_RT
```

## 2. Set aside a CPU

Add these to the kernel command line. On GRUB systems, append them to
`GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`, then run `sudo update-grub`
and reboot:

```text
isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
```

| Parameter | Effect on CPU 3 |
|-----------|-----------------|
| `isolcpus=3` | the scheduler puts no ordinary tasks there |
| `nohz_full=3` | no periodic timer tick while a single task runs |
| `rcu_nocbs=3` | kernel RCU callbacks run on the other CPUs |
| `irqaffinity=0-2` | device interrupts go to the other CPUs |

Check with `cat /sys/devices/system/cpu/isolated` (prints `3`). Disable
`irqbalance`, which would otherwise move interrupts back:

```bash
sudo systemctl disable --now irqbalance
```

## 3. Remove sources of jitter

- In the **BIOS/UEFI**: disable deep C-states (allow C1 at most), Turbo
  Boost and SMT/Hyper-Threading if you can. Waking a core from a deep sleep
  state alone can cost 100 µs.
- Fix the **CPU frequency**:

  ```bash
  sudo apt install linux-cpupower    # Debian; linux-tools-common on Ubuntu
  sudo cpupower frequency-set -g performance
  ```

## 4. Install the tools

```bash
sudo apt install rt-tests stress-ng
julia --project=bench -e 'using Pkg; Pkg.instantiate()'
```

`rt-tests` provides `cyclictest`; `stress-ng` generates background load.

## 5. Measure

Start ten minutes of load on the **other** CPUs: CPU work, disk I/O and memory
pressure, which is what exposes latency problems.

```bash
taskset -c 0-2 stress-ng --cpu 3 --io 2 --vm 1 --vm-bytes 25% --timeout 15m &
```

Then run both benchmarks, one after the other, at 1 kHz for 600 000 cycles
(10 minutes each), on CPU 3, at `SCHED_FIFO` priority 80.
`bench/rtrun.sh` gives them the privileges they need, as your own user (see
[Tutorial: bring up a real servo drive](@ref)):

```bash
bench/rtrun.sh ./bench/compare_cyclictest.sh 1000 600000 3 80
```

The script prints the kernel it runs on, then each result:

```text
kernel: ...-rt-amd64   preempt: 1
settings: period=1000us iterations=600000 cpu=3 prio=80

=== cyclictest ===
# Min Latencies: ...
# Avg Latencies: ...
# Max Latencies: ...
# p99 <= ... us, p99.9 <= ... us

=== ArtyHill (wake-up jitter is the comparable number) ===
LoopStats: 600000 iterations, 0 overruns, 0 errors
  wake-up jitter mean ..., p99 ≤ ..., p99.9 ≤ ..., max ...
  step latency:   mean ..., p99 ≤ ..., p99.9 ≤ ..., max ...
  mlockall=true SCHED_FIFO=true pinned=true gc_off=true safe_cmd_sent=false
```

Full histograms,
in 1 µs buckets, are written to `bench/cyclictest_hist.txt` and
`bench/artyhill_hist.csv`.

## 6. Read the results

- Check the last line first: **`mlockall`, `SCHED_FIFO` and `pinned` must all
  be `true`**. Otherwise the loop ran in soft real-time mode and the numbers
  do not count.
- **Compare ArtyHill's wake-up jitter with cyclictest's latency.** Both measure
  the same thing: how long after its deadline the thread actually woke up.
  Whatever ArtyHill shows beyond cyclictest is the cost of Julia and the loop.
- **The maximum is what matters** for a control loop, not the mean: a 1 kHz
  loop must finish every cycle within 1 ms, including the worst one. p99.9
  tells you whether the maximum is a single outlier or a pattern.
- **Step latency** is the compute time of one `control_step!` (here with a
  simulated plant). Wake-up jitter plus step latency must stay below the
  period, with margin for the backend's I/O.
- **Overruns** count cycles that finished after the next deadline. On a
  well-set-up machine, there should be none.

For contrast, repeat the measurement **without** PREEMPT_RT (boot the stock
kernel) and **without** isolation (pass CPU `-1`): that shows how much each
step contributes.

## 7. Record the numbers

Write down, for each configuration:

| Kernel | Isolation | Load | cyclictest max / p99.9 (µs) | ArtyHill wake max / p99.9 (µs) | ArtyHill step max (µs) | Overruns |
|--------|-----------|------|------|------|------|------|
| PREEMPT_RT | CPU 3 | stress-ng | | | | |
| stock | CPU 3 | stress-ng | | | | |
| stock | none | stress-ng | | | | |

along with the CPU model, the kernel version (`uname -r`) and the BIOS
settings you changed. Latency numbers are only comparable between identical
setups.

## Troubleshooting

**`mlockall` or `SCHED_FIFO` is `false`.** Run through `bench/rtrun.sh`. If
you use plain `sudo` instead, Julia runs as root with root's package depot.

**`cyclictest not found`.** Install `rt-tests` (step 4).

**Huge spikes (milliseconds) every few seconds.** Usually firmware: SMIs
(System Management Interrupts) from the BIOS, which no kernel setting can
stop. Try disabling power management and hardware monitoring options in the
BIOS; on some machines the spikes cannot be removed.

**ArtyHill is much worse than cyclictest.** Check that nothing else in the
Julia process allocates heavily while the loop runs: the GC is paused for the
whole process, but other threads can still trigger page faults. Run the loop
alone with `--threads=2`, as the benchmark does.
