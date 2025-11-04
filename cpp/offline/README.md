# Spectacular AI main API example

This example shows how to run basic stereo VIO using offline data to mimic real-time use case. Functions in the main header `spectacularAI/vio.hpp` are used primarily.

* Tested platforms: Linux
* Dependencies: CMake, FFmpeg (for video input)

## Setup

* Install the Spectacular AI SDK
* Clone the submodules: `cd cpp/offline/target && git submodule update --init --recursive`.
* Build this example using CMake:

```bash
mkdir target
cd target
cmake -DspectacularAI_DIR=<path/to/spectacularai-sdk/lib/cmake/spectacularAI> ..
make
```

The `-DspectacularAI_DIR` option is not needed is you have used `sudo make install` for the SDK.

## Usage

In the target folder, run `./vio_jsonl -i path/to/data -o out.jsonl`, where

* `-i` specifies the input folder, see details below. If omitted, mock data will be used.
* `-o` specifies output JSONL file. If omitted, prints instead to stdout.

Input data is read from a given folder with the following hierarchy:

```
├── calibration.json
├── vio_config.yaml
├── data.jsonl
├── data.mp4
└── data2.mp4
```

when using video files. And if instead using PNG images:

```
├── calibration.json
├── vio_config.yaml
├── data.jsonl
├── frames1
│   ├── 00000000.png
│   ├── 00000001.png
│   ├── ...
│   └── 00000600.png
└── frames2
    ├── 00000000.png
    ├── 00000001.png
    ├── ...
    └── 00000600.png
```

## Debugging

The option `-r <folder_name>` records the session input data and VIO output to the given folder. If the produced video files do not look correct when viewed in a video player, there may be issue with the image data input into the SDK.

## Visualization

To plot the position track from `-o out.jsonl`, you can use `python3 plot_positions.py out.jsonl`.

## Copyright

For the included libraries, see
* [nlohmann/json](https://github.com/nlohmann/json): `json/LICENSE.MIT`
* [lodepng](https://lodev.org/lodepng/): `lodepng/LICENSE`

For access to the C++ SDK, contact us at <https://www.spectacularai.com/#contact>.

Available for multiple OSes and CPU architectures.
