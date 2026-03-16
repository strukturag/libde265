#!/bin/bash
#
# Run AFL++-based fuzzing on libde265.
#
# Usage:
#   ./scripts/fuzz-afl.sh [corpus_dir] [findings_dir] [extra_args...]
#
# Examples:
#   ./scripts/fuzz-afl.sh                                # use defaults
#   ./scripts/fuzz-afl.sh /path/to/corpus afl-findings   # custom dirs
#
# Build:
#   cmake --preset afl
#   cmake --build build-afl
#
# Prerequisites:
#   apt install afl++
#

set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-afl"
DEC265="$BUILD_DIR/dec265/dec265"
DEFAULT_CORPUS="$ROOT_DIR/build-fuzzing/corpus-minimized"
DEFAULT_FINDINGS="$BUILD_DIR/afl-findings"

if [ ! -x "$DEC265" ]; then
    echo "Error: dec265 not found at $DEC265"
    echo "Build it first:"
    echo "  cmake --preset afl"
    echo "  cmake --build build-afl"
    exit 1
fi

if ! command -v afl-fuzz &>/dev/null; then
    echo "Error: afl-fuzz not found. Install AFL++:"
    echo "  apt install afl++"
    exit 1
fi

CORPUS="${1:-$DEFAULT_CORPUS}"
FINDINGS="${2:-$DEFAULT_FINDINGS}"
shift 2 2>/dev/null || true

mkdir -p "$FINDINGS"

echo "dec265:    $DEC265"
echo "Corpus:    $CORPUS"
echo "Findings:  $FINDINGS"
echo ""

exec afl-fuzz -i "$CORPUS" -o "$FINDINGS" -- "$DEC265" -q @@
