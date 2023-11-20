#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <spectacularAI_k4aPlugin_DIR>"
    exit 1
fi

set -eux

spectacularAI_k4aPlugin_DIR=$1

: "${BUILD_TYPE:=Release}"
: "${WINDOWS_SDK_VERSION:=10.0.19041.0}"
: "${WINDOWS_SDK_VERSION:=10.0.19041.0}"
: "${VISUAL_STUDIO_VERSION:=Visual Studio 16 2019}"
: "${BUILD_MAPPING_VISU:=ON}"

CMAKE_FLAGS=(-G "${VISUAL_STUDIO_VERSION}" -A x64 -DCMAKE_SYSTEM_VERSION=${WINDOWS_SDK_VERSION})

ROOT=$(pwd)
TARGET="$ROOT/target"

K4A_SDK_FOLDER_PATTERN=( /c/Program\ Files/Azure\ Kinect\ SDK* ) # Version number at the end
K4A_SDK_FOLDER=${K4A_SDK_FOLDER_PATTERN[0]}

echo "Using k4a SDK from directory: ${K4A_SDK_FOLDER}"

mkdir -p "$TARGET"
cd "$TARGET"
cmake "${CMAKE_FLAGS[@]}" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -Dk4a_DIR="${K4A_SDK_FOLDER}" \
  -DspectacularAI_k4aPlugin_DIR="$spectacularAI_k4aPlugin_DIR" \
  -DBUILD_MAPPING_VISU="$BUILD_MAPPING_VISU" \
  ..
cmake --build . --config $BUILD_TYPE

# Copy depth module used by k4a lib manually, CMake doesn't know about it and fixing that seemed impossible
cp "${K4A_SDK_FOLDER}/sdk/windows-desktop/amd64/release/bin/depthengine_2_0.dll" "${TARGET}/Release"
