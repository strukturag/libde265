#!/bin/bash
#
# Run libFuzzer-based fuzzing on libde265.
#
# Usage:
#   ./scripts/fuzz-libfuzzer.sh [corpus_dir] [extra_args...]
#
# Examples:
#   ./scripts/fuzz-libfuzzer.sh                          # use default corpus
#   ./scripts/fuzz-libfuzzer.sh /path/to/corpus -jobs=16 # parallel fuzzing
#   ./scripts/fuzz-libfuzzer.sh corpus -merge=1 corpus-minimized  # minimize corpus
#
# Build:
#   cmake --preset fuzzing
#   cmake --build build-fuzzing
#

set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-fuzzing"
FUZZER="$BUILD_DIR/fuzzing/stream_fuzzer"
DEFAULT_CORPUS="$ROOT_DIR/build-fuzzing/corpus-minimized"

if [ ! -x "$FUZZER" ]; then
    echo "Error: fuzzer not found at $FUZZER"
    echo "Build it first:"
    echo "  cmake --preset fuzzing"
    echo "  cmake --build build-fuzzing"
    exit 1
fi

CORPUS="${1:-$DEFAULT_CORPUS}"
shift 2>/dev/null || true

echo "Fuzzer:  $FUZZER"
echo "Corpus:  $CORPUS"
echo ""

exec "$FUZZER" "$CORPUS" "$@"
