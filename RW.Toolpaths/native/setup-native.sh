#!/usr/bin/env bash
# setup-native.sh — builds libboostvoronoi.so on Linux/macOS
# Usage:
#   ./setup-native.sh                         # auto-install boost if missing
#   ./setup-native.sh --boost-dir /path/boost # explicit header dir
#   ./setup-native.sh --clean                 # wipe build dir first

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOOST_INCLUDE_DIR=""
CLEAN=0

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --boost-dir) BOOST_INCLUDE_DIR="$2"; shift 2 ;;
        --clean)     CLEAN=1; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

find_boost() {
    for candidate in \
        "${BOOST_ROOT:-}" \
        "${BOOST_INCLUDEDIR:-}" \
        "/usr/include" \
        "/usr/local/include" \
        "$SCRIPT_DIR/boost_headers"
    do
        if [[ -n "$candidate" && -f "$candidate/boost/polygon/voronoi.hpp" ]]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

# ── 1. Resolve Boost ──────────────────────────────────────────────────────────

if [[ -z "$BOOST_INCLUDE_DIR" ]]; then
    BOOST_INCLUDE_DIR="$(find_boost || true)"
fi

if [[ -z "$BOOST_INCLUDE_DIR" ]]; then
    echo "[boost] Headers not found — attempting install via system package manager..."
    if command -v apt-get &>/dev/null; then
        sudo apt-get install -y libboost-dev
        BOOST_INCLUDE_DIR="$(find_boost)"
    elif command -v brew &>/dev/null; then
        brew install boost
        BOOST_INCLUDE_DIR="$(brew --prefix boost)/include"
    elif command -v dnf &>/dev/null; then
        sudo dnf install -y boost-devel
        BOOST_INCLUDE_DIR="$(find_boost)"
    else
        echo "[boost] Cannot find a package manager. Install Boost headers manually."
        echo "        Then re-run:  ./setup-native.sh --boost-dir /path/to/boost"
        exit 1
    fi
fi

echo "[boost] Using headers at: $BOOST_INCLUDE_DIR"

# ── 2. CMake configure ────────────────────────────────────────────────────────

BUILD_DIR="$SCRIPT_DIR/build"
INSTALL_PREFIX="$(realpath "$SCRIPT_DIR/..")"

if [[ "$CLEAN" -eq 1 && -d "$BUILD_DIR" ]]; then
    echo "[cmake] Cleaning $BUILD_DIR ..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"

echo "[cmake] Configuring..."
cmake "$SCRIPT_DIR" \
    -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    "-DBOOST_INCLUDEDIR=$BOOST_INCLUDE_DIR" \
    "-DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX"

# ── 3. Build ──────────────────────────────────────────────────────────────────

echo "[cmake] Building..."
cmake --build "$BUILD_DIR" --config Release -- -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"

# ── 4. Install ────────────────────────────────────────────────────────────────

echo "[cmake] Installing..."
cmake --install "$BUILD_DIR" --config Release

echo ""
echo "[done] Native library installed:"
find "$INSTALL_PREFIX/runtimes" -name "libboostvoronoi*" 2>/dev/null
