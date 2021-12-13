# Spectacular AI Python SDK for OAK-D

![SDK install demo](https://spectacularai.github.io/docs/gif/pip-install.gif)

## Prerequisites

 * An [OAK-D device](https://store.opencv.ai/products/oak-d)
 * Supported operating systems: Windows 10 or Linux (e.g., Ubuntu 20+)
 * Supported Python versions 3.6+

## Installation

Install the Python pacakge: `pip install spectacularAI`

## Examples

To install dependecies for all examples you can use: `pip install -r requirements.txt `

 * **Minimal example**. Prints 6-DoF poses as JSON text: [`python vio_jsonl.py`](vio_jsonl.py)
 * **Basic visualization**. Interactive 3D plot / draw in the air with the device: [`python vio_visu.py`](vio_visu.py)
 * **3D pen**. Draw in the air: cover the OAK-D color camera to activate the ink. [`python pen_3d.py`](pen_3d.py)
 * **Advanced Spatial AI example**. Spectacular AI VIO + Tiny YOLO object detection.
    See [`depthai_combination.py`](depthai_combination.py) for additional dependencies that also need to be installed.
 * **Mixed reality**. In less than 130 lines of Python, with the good old OpenGL functions like `glTranslatef` used for rendering.
    Also requires `PyOpenGL_accelerate` to be installed, see [`mixed_reality.py`](mixed_reality.py) for details.

## API documentation

https://spectacularai.github.io/docs/sdk/python/latest

## Troubleshooting

Rarely, the OAK-D device factory calibration may be inaccurate, which may cause the the VIO performance to be always very bad in all environments. If this is the case, the device can be recalibrated following these instructions: https://www.youtube.com/watch?v=nD0hy9164p8

## License

For more info, see the [main README.md](../../README.md).
