#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <spectacularAI_orbbecPlugin_DIR> <OrbbecSDK_DIR>"
    exit 1
fi

if [ -z "$2" ]; then
    echo "Usage: $0 <spectacularAI_orbbecPlugin_DIR> <OrbbecSDK_DIR>"
    exit 1
fi

set -eux

spectacularAI_orbbecPlugin_DIR=$1
OrbbecSDK_DIR=$2

: "${BUILD_TYPE:=Release}"
: "${WINDOWS_SDK_VERSION:=10.0.19041.0}"
: "${WINDOWS_SDK_VERSION:=10.0.19041.0}"
: "${VISUAL_STUDIO_VERSION:=Visual Studio 16 2019}"

CMAKE_FLAGS=(-G "${VISUAL_STUDIO_VERSION}" -A x64 -DCMAKE_SYSTEM_VERSION=${WINDOWS_SDK_VERSION})

ROOT=$(pwd)
TARGET="$ROOT/target"

mkdir -p "$TARGET"
cd "$TARGET"
cmake "${CMAKE_FLAGS[@]}" \
  -DOrbbecSDK_DIR="${OrbbecSDK_DIR}" \
  -DspectacularAI_orbbecPlugin_DIR="$spectacularAI_orbbecPlugin_DIR" \
  ..
cmake --build . --config $BUILD_TYPE
