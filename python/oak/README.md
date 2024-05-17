# Spectacular AI Python SDK examples for OAK-D

![SDK install demo](https://spectacularai.github.io/docs/gif/spatial-ai.gif)

**See https://spectacularai.github.io/docs/sdk/wrappers/oak.html for instructions and API documentation**

## Examples

 * **Minimal example**. Prints 6-DoF poses as JSON text: [`python vio_jsonl.py`](vio_jsonl.py)
 * **Basic visualization**. Interactive 3D plot / draw in the air with the device: [`python vio_visu.py`](vio_visu.py)
 * **3D pen**. Draw in the air: cover the OAK-D color camera to activate the ink. [`python pen_3d.py`](pen_3d.py)
 * **3D mapping**. Build and visualize 3D point cloud of the environment in real-time. [`mapping_visu.py`](mapping_visu.py)
 * **3D mapping with Augmented Reality**. Show 3D mesh or point cloud on top of camera view, using OpenGL. [`mapping_ar.py`](mapping_ar.py)
 * **3D Mapping with ROS Integration**. Runs Spectacular AI VIO and publishes pose information and keyframes over ROS topics. [`mapping_ros.py`](mapping_ros.py)
 * **Advanced Spatial AI example**. Spectacular AI VIO + Tiny YOLO object detection.
    See [`depthai_combination.py`](depthai_combination.py) for additional dependencies that also need to be installed.
 * **Mixed reality**. In less than 130 lines of Python, with the good old OpenGL functions like `glTranslatef` used for rendering.
    Also requires `PyOpenGL_accelerate` to be installed, see [`mixed_reality.py`](mixed_reality.py) for details.
 * **GNSS-VIO** example, reads external GNSS from standard input [`vio_gnss.py`](vio_gnss.py) (see also [these instructions](https://spectacularai.github.io/docs/pdf/GNSS-VIO_OAK-D_Python.pdf))
 * **April Tag example**: Visualize detected April Tags [`python april_tag.py path/to/tags.json`](april_tag.py). See: https://spectacularai.github.io/docs/pdf/april_tag_instructions.pdf
 * **Remote visualization over SSH**. Can be achieved by combining the `vio_jsonl.py` and `vio_visu.py` scripts as follows:

        ssh user@example.org 'python -u /full/path/to/vio_jsonl.py' | python -u vio_visu.py --file=-

    Here `user@example.org` represents a machine (e.g., Raspberry Pi) that is connected to the OAK-D, but is not necessarily attached to a monitor.
    The above command can then be executed on a laptop/desktop machine, which then shows the trajectory of the OAK-D remotely (like in [this video](https://youtu.be/mBZ8bszNnwI?t=17)).
