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
 * **Remote visualization over SSH**. Can be achieved by combining the `vio_jsonl.py` and `vio_visu.py` scripts as follows:

        ssh user@example.org 'python -u /full/path/to/vio_jsonl.py' | python -u vio_visu.py --file=-
        
    Here `user@example.org` represents a machine (e.g., Raspberry Pi) that is connected to the OAK-D, but is not necessarily attached to a monitor.
    The above command can then be executed on a laptop/desktop machine, which then shows the trajectory of the OAK-D remotely (like in [this video](https://youtu.be/mBZ8bszNnwI?t=17)).

## API documentation

https://spectacularai.github.io/docs/sdk/python/latest

## Troubleshooting

Rarely, the OAK-D device factory calibration may be inaccurate, which may cause the the VIO performance to be always very bad in all environments. If this is the case, the device can be recalibrated following [Luxonis' instructions](https://docs.luxonis.com/en/latest/pages/calibration/) (see also [our instructions for fisheye cameras](https://spectacularai.github.io/docs/pdf/oak_fisheye_calibration_instructions.pdf) for extra tips).

### Fisheye cameras

It is possible to fit certain OAK-D models with fisheye lenses. These are supported from Spectacular AI SDK version 0.16 onwards, but require the following `spectacularAI.Configuration` changes to be applied in `spectacularAI.Pipeline`:
```
meshRectification = True
depthScaleCorrection = True
```
These settings are work-arounds that also (currently) work with normal OAK-D lenses, but may stop working with future DepthAI versions, and increase initialization time, and therefore they are not recommended for general use. The settings are enabled in all examples in the [`oak-fisheye` branch](https://github.com/SpectacularAI/sdk-examples/tree/oak-fisheye) of this repository.

Also calibrate the camera according to [these instructions](https://spectacularai.github.io/docs/pdf/oak_fisheye_calibration_instructions.pdf), if you have changed the lenses or the device did not include a factory calibration.

## License

For more info, see the [main README.md](../../README.md).
