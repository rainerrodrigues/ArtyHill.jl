#!/usr/bin/env bash
# Run a command as yourself, but with the privileges a real-time EtherCAT loop
# needs: raw sockets, SCHED_FIFO and mlockall.
#
#   bench/rtrun.sh julia --threads=2 --project=bench bench/drive_bench.jl eth1
#   bench/rtrun.sh ./bench/compare_cyclictest.sh 1000 600000 3 80
#
# The capabilities are granted as *ambient* capabilities through a transient
# systemd unit. Unlike `setcap` on the julia binary, this does not put Julia in
# secure-execution mode (where the loader ignores $ORIGIN and LD_LIBRARY_PATH),
# and unlike `sudo julia` it keeps your own depot, packages and file ownership.
# Needs systemd and sudo rights to start the unit.
set -euo pipefail
[[ $# -gt 0 ]] || { sed -n '2,7p' "$0"; exit 2; }

# Settings read by the bench scripts, passed through when set.
envs=(--setenv=HOME="$HOME" --setenv=PATH="$PATH")
for v in AXES COUNTS_PER_REV MOVE_REV MOVE_SECONDS MAX_RPS CYCLE_US CPU CONFIGURE_PDO \
         JULIA JULIA_DEPOT_PATH JULIA_PROJECT JULIA_NUM_THREADS; do
    [[ -n "${!v:-}" ]] && envs+=(--setenv="$v=${!v}")
done

exec sudo systemd-run --quiet --pty --collect --same-dir \
    --uid="$(id -u)" --gid="$(id -g)" \
    -p AmbientCapabilities="CAP_NET_RAW CAP_SYS_NICE CAP_IPC_LOCK" \
    -p LimitMEMLOCK=infinity -p LimitRTPRIO=99 \
    "${envs[@]}" -- "$@"
