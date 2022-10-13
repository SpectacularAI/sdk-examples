# Spectacular AI Replay API example

This example shows how to replay recordings created using Spectacular AI SDK. The header `spectacularAI/replay.hpp` is primarily used.

* Tested platforms: Linux
* Dependencies: CMake, FFmpeg (for video input)

## Setup

* Install the Spectacular AI SDK
* Build this example using CMake:

```
mkdir target
cd target
cmake -DspectacularAI_DIR=<path/to/spectacularai-sdk/lib/cmake/spectacularAI> ..
make
```

The `-DspectacularAI_DIR` option is not needed is you have used `sudo make install` for the SDK.

## Usage

In the target folder, run `./replay -i path/to/data -o out.jsonl`, where

* `-i` specifies the input folder, see details below.
* `-o` specifies output JSONL file.

See the source code for more options. Input data is read from a given folder with the following hierarchy:

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

## Copyright

For access to the C++ SDK, contact us at <https://www.spectacularai.com/#contact>.

Available for multiple OSes and CPU architectures.
