#!/usr/bin/env bash
# Run ArtyHill's EtherCAT end-to-end test: real SOEM 1.4 master ↔ emulated
# CiA 402 drives over a veth pair. No hardware and no root needed: everything
# runs inside an unprivileged user + network namespace (`unshare -rn`).
#
#   test/ethercat/run.sh            # uses `julia` on PATH (or JULIA=...)
#
# SOEM v1.4.0 is downloaded and built once into $ARTYHILL_SOEM_CACHE
# (default ~/.cache/artyhill-soem). On Ubuntu ≥ 23.10, unprivileged user
# namespaces may be restricted by AppArmor; there, run as root instead
# (`sudo -E test/ethercat/run.sh`), which skips `unshare`.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
JULIA=${JULIA:-julia}
CACHE=${ARTYHILL_SOEM_CACHE:-$HOME/.cache/artyhill-soem}
SOEM_VER=1.4.0

if [[ ! -f "$CACHE/libsoem.so" ]]; then
    mkdir -p "$CACHE"
    if [[ ! -d "$CACHE/SOEM-$SOEM_VER" ]]; then
        curl -fsSL "https://github.com/OpenEtherCATsociety/SOEM/archive/refs/tags/v$SOEM_VER.tar.gz" \
            | tar xz -C "$CACHE"
    fi
    S="$CACHE/SOEM-$SOEM_VER"
    cc -shared -fPIC -O2 -I"$S/soem" -I"$S/osal" -I"$S/osal/linux" -I"$S/oshw/linux" \
        "$S"/soem/*.c "$S/osal/linux/osal.c" "$S"/oshw/linux/*.c -lpthread -lrt -o "$CACHE/libsoem.so"
    echo "built $CACHE/libsoem.so"
fi

inner() {
    ip link add ecat0 type veth peer name ecat1
    ip link set ecat0 up
    ip link set ecat1 up
    # The namespace has no route to the internet: resolve the test env offline
    # from what `prefetch` downloaded.
    JULIA_PKG_OFFLINE=true ARTYHILL_ETHERCAT=ecat0 ARTYHILL_ETHERCAT_PEER=ecat1 ARTYHILL_LIBSOEM_DIR="$CACHE" \
        ARTYHILL_SOEM_SRC="$CACHE/SOEM-$SOEM_VER" \
        "$JULIA" --threads=2 --project="$ROOT" -e 'using Pkg; Pkg.test(; coverage=get(ENV, "ARTYHILL_COVERAGE", "") == "1")'
}

# Download the package and its test-only dependencies while we still have network.
prefetch() {
    ARTYHILL_ROOT="$ROOT" "$JULIA" -e '
        using Pkg
        root = ENV["ARTYHILL_ROOT"]
        proj = Pkg.TOML.parsefile(joinpath(root, "Project.toml"))
        test_deps = [PackageSpec(name=n, uuid=proj["extras"][n]) for n in proj["targets"]["test"]]
        Pkg.activate(; temp=true)
        Pkg.develop(path=root)
        Pkg.add(test_deps)'
}

if [[ "${1:-}" == "--inner" ]]; then
    inner
    exit
fi

prefetch
if [[ $(id -u) -eq 0 ]]; then
    exec unshare -n "$0" --inner          # own network namespace, already root
else
    exec unshare -rn "$0" --inner         # unprivileged user + network namespace
fi
