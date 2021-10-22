# Spectacular AI Python SDK for OAK-D

## Prerequisites

 * An [OAK-D device](https://store.opencv.ai/products/oak-d)
 * Supported operating systems: Windows 10 or Linux (e.g., Ubuntu 20+)
 * Supported Python versions 3.6+

## Installation

 1. Install the Python pacakge: `pip install spectacularAI`
 2. Install the extra dependencies for the examples: `pip install matplotlib opencv-python==4.5.3.56`

NOTE: Version 4.5.4.58 (most recent as of October 22, 2021) of `opencv-python` is broken on Ubuntu, therefore recommending an older version.

## Examples

 * **Minimal example**. Prints 6-DoF poses as JSON text: [`python vio_jsonl.py`](vio_jsonl.py)
 * **Basic visualization**. Interactive 3D plot / draw in the air with the device: [`python vio_visu.py`](vio_visu.py)
 * **3D pen**. Draw in the air: cover the OAK-D color camera to activate the ink. [`python pen_3d.py`](pen_3d.py)
 * **Advanced Spatial AI example**. Spectacular AI VIO + Tiny YOLO object detection.
    See [`depthai_combination.py`](depthai_combination.py) for additional dependencies that also need to be installed.

### License

For more info, see the [main README.md](../../README.md).
