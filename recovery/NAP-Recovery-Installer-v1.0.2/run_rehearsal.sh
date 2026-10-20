#!/bin/sh
# macOS or Linux workstation; never installs or activates on a comma.
set -eu
MODE="${1:-check}"
MATERIALS="${2:-$HOME/NAP-Recovery}"
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PRIVATE="$MATERIALS/nap-private-recovery-20261020T032742Z.tar.gz"
RUNTIME="$MATERIALS/nap-runtime-aarch64-20261020T040848Z.tar.gz"
case "$MODE" in
  check)
    exec python3 "$HERE/scripts/nap_package.py" check \
      --private "$PRIVATE" --runtime "$RUNTIME"
    ;;
  prepare)
    exec python3 "$HERE/scripts/nap_package.py" prepare \
      --private "$PRIVATE" --runtime "$RUNTIME" \
      --destination "$MATERIALS/NAP-restoration-preview"
    ;;
  *)
    echo "Usage: sh run_rehearsal.sh [check|prepare] [materials-directory]" >&2
    exit 2
    ;;
esac
