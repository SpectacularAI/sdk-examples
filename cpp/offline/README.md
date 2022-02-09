# SpectacularAI offline example

This example shows how to run basic stereo VIO using offline data.

* Supported platform: Linux
* Dependencies: FFmpeg (for video input)

## Usage

Install the Spectacular AI SDK and build this example using CMake. Then run `./vio_jsonl -i path/to/data -o out.jsonl`, where

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

## Visualization

To plot the position track from `-o out.jsonl`, you can use `python3 plot_positions.py out.jsonl`.

## Copyright

For the included libraries, see
* [nlohmann/json](https://github.com/nlohmann/json): `nlohmann/LICENSE.MIT`
* [lodepng](https://lodev.org/lodepng/): `lodepng/LICENSE`

For access to the C++ SDK, contact us at <https://www.spectacularai.com/#contact>.
Available for multiple OSes and CPU architectures.
