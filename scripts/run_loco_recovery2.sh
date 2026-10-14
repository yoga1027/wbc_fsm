#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$project_root" -B "$project_root/build"
cmake --build "$project_root/build" -j 4
exec "$project_root/build/wbc_fsm" --loco-recovery2 "$@"
