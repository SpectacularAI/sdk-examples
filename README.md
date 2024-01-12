![SDK install demo](https://spectacularai.github.io/docs/gif/pip-install.gif)

# Spectacular AI SDK examples

**Spectacular AI SDK** fuses data from cameras and IMU sensors (accelerometer and gyroscope)
and outputs an accurate 6-degree-of-freedom pose of a device.
This is called Visual-Inertial SLAM (VISLAM) and it can be used in, among other cases, tracking
(autonomous) robots and vehicles, as well as Augmented, Mixed and Virtual Reality.

The SDK also includes a _Mapping API_ that can be used to access the full SLAM map for
both real-time and offline 3D reconstruction use cases.

### Quick links

#### [SDK documentation](https://spectacularai.github.io/docs/sdk/)
#### [C++ release packages](https://github.com/SpectacularAI/sdk/releases)
#### [Gaussian Splatting & NeRFs](https://spectacularai.github.io/docs/sdk/tools/nerf.html)

### List of examples

 * **[C++](https://github.com/SpectacularAI/sdk-examples/tree/main/cpp)**
 * **[Python / OAK-D](https://github.com/SpectacularAI/sdk-examples/tree/main/python/oak#spectacular-ai-python-sdk-examples-for-oak-d)**

See also the parts of the SDK with public source code:

 * [C++ recording tools](https://github.com/SpectacularAI/sdk/tree/main/cpp)
 * [Python tools](https://github.com/SpectacularAI/sdk/tree/main/python/cli)

## Copyright

The examples in this repository are licensed under Apache 2.0 (see LICENSE).

The SDK itself (not included in this repository) is proprietary to Spectacular AI.
The OAK / Depth AI wrapper available in PyPI is free for non-commercial use on x86_64 Windows and Linux platforms.
For commerical licensing options and more SDK variants (ARM binaries & C++ API),
contact us at https://www.spectacularai.com/#contact .
