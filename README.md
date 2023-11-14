# Spectacular AI SDK examples

![Spatial AI](https://spectacularai.github.io/docs/gif/spatial-ai.gif)

**Spectacular AI SDK** fuses data from cameras and IMU sensors (accelerometer and gyroscope)
and outputs an accurate 6-degree-of-freedom pose of a device.
This is called Visual-Inertial SLAM (VISLAM) and it can be used in, among other cases, tracking
(autonomous) robots and vehicles, as well as Augmented, Mixed and Virtual Reality.

The SDK also includes a _Mapping API_ that can be used to access the full SLAM map for
both real-time and offline 3D reconstruction use cases.

## SDK architecture and documentation

The SDK consists of a device-independent C++ _core SDK_, which is included in SDK _wrappers_ for different out-of-the-box supported devices (see below). All wrappers have a C++ API and their documentation is included in the [release packages](https://github.com/SpectacularAI/sdk/releases).

The SDK also has a Python API, which supports OAK-D devices and post-processing data from other devices using the _Mapping API_ (see [examples here](python/mapping)).

[**Link to Python API documentation**](https://spectacularai.github.io/docs/sdk/python/latest/)

## Supported devices

### Out-of-the-box

The SDK supports a limited set of devices out-of-the-box. This means that the SDK can be used without any manual calibration, integration or parameter tuning, with these devices. If you want to test the SDK as easily as possible, we recommend buying one of these devices.

 * [OAK-D by Luxonis](https://store.opencv.ai/products/oak-d). Other stereo OAK variants with IMU sensors (e.g., OAK-D-PoE, OAK-D-CM3/4) are also supported, but _not_ monocular systems nor OAK-D-Lite. There is an easy-to-use [Python wrapper](python/oak) and a [C++ version](cpp/oak).
 * [Intel RealSense D455](https://www.intelrealsense.com/depth-camera-d455/) and D435i (notice the "i", which is for IMU and is a must-have. D435 is _not_ supported). Currently only supported on x86-64. Available as a [C++ version](cpp/realsense).
 * _Azure Kinect DK_. [Contact us](https://www.spectacularai.com/#contact) for access to the SDK.

All the non-commerical SDKs are available [here](https://github.com/SpectacularAI/sdk).

### Other devices

The _core SDK_ can be integrated on any device with adequate sensors and processing capabilities. At minimum, a single rolling-shutter camera + mid-quality MEMS IMU is sufficient. For better performance, a global-shutter stereo camera and a better MEMS IMU (e.g., CEVA BNO08X or Murata SCHA634) is recommended. At minimum, CPU resources equivalent to approximately one ARM Cortex A72 core (e.g., one core in Raspberry Pi 4) is required.

For more information and access to the core SDK, contact us at https://www.spectacularai.com/#contact.

## Copyright

The examples in this repository are licensed under Apache 2.0 (see LICENSE).

The SDK itself (not included in this repository) is proprietary to Spectacular AI.
The OAK / Depth AI wrapper available in PyPI is free for non-commercial use on x86_64 Windows and Linux platforms.
For commerical licensing options and more SDK variants (ARM binaries & C++ API),
contact us at https://www.spectacularai.com/#contact .
