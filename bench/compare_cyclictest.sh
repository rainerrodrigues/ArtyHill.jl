#!/usr/bin/env bash
# Compare ArtyHill's loop jitter with cyclictest (rt-tests) under identical settings.
#
#   sudo ./bench/compare_cyclictest.sh [period_us] [iterations] [cpu] [priority]
#
# Needs: rt-tests (`sudo apt install rt-tests`), Julia on PATH (or JULIA=...),
# root or CAP_SYS_NICE + CAP_IPC_LOCK. For meaningful numbers use a PREEMPT_RT
# kernel and an isolated CPU (isolcpus=/nohz_full=).
set -euo pipefail
PERIOD_US=${1:-1000}
ITER=${2:-100000}
CPU=${3:-1}
PRIO=${4:-80}
JULIA=${JULIA:-julia}
HERE="$(cd "$(dirname "$0")" && pwd)"

command -v cyclictest >/dev/null || { echo "cyclictest not found: sudo apt install rt-tests" >&2; exit 1; }
echo "kernel: $(uname -r)   preempt: $(cat /sys/kernel/realtime 2>/dev/null || echo '0 (not PREEMPT_RT)')"
echo "settings: period=${PERIOD_US}us iterations=${ITER} cpu=${CPU} prio=${PRIO}"

echo; echo "=== cyclictest ==="
cyclictest -m -q -t1 -p"$PRIO" -i"$PERIOD_US" -l"$ITER" -a"$CPU" -h10000 > "$HERE/cyclictest_hist.txt"
grep -E "^# (Min|Avg|Max) Latencies" "$HERE/cyclictest_hist.txt" || true
# p99 / p99.9 from the histogram
awk -v n="$ITER" '/^[0-9]/ { c+=$2; if (!a && c>=0.99*n) {a=$1}; if (!b && c>=0.999*n) {b=$1} }
     END { printf "# p99 <= %d us, p99.9 <= %d us\n", a+1, b+1 }' "$HERE/cyclictest_hist.txt"

echo; echo "=== ArtyHill (wake-up jitter is the comparable number) ==="
"$JULIA" --threads=2 --project="$HERE" "$HERE/latency.jl" "$PERIOD_US" "$ITER" "$CPU" "$PRIO"
