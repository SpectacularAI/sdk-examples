# Spectacular AI SDK examples

**Spectacular AI SDK** fuses data from cameras and IMU sensors (accelerometer and gyroscope)
and outputs an accurate 6-degree-of-freedom pose of a device.
This is called Visual-Inertial Odometry (VIO) and it can be used in, among other cases, tracking
(autonomous) robots and vehicles, as well as Augmented, Mixed and Virtual Reality.

## Supported devices

### Out-of-the-box

The SDK supports a limited set of devices out-of-the-box. This means that the SDK can be used without any manual calibration, integration or parameter tuning, with these devices. If you want to test the SDK as easily as possible, we recommend buying one of these devices.
At the moment, the only supported device is the [OAK-D by Luxonis](https://store.opencv.ai/products/oak-d).
See the folder [`python/oak`](python/oak) for more information about the OAK-D wrapper.

### Other devices

The SDK can be integrated on any device with adequate sensors and processing capabilities. At minimum, a single rolling-shutter camera + mid-quality MEMS IMU is sufficient. For better performance, a global-shutter stereo camera and a better MEMS IMU (e.g., CEVA BNO08X or Murata SCHA634) is recommended. At minimum, CPU resources equivalent to approximately one ARM Cortex A72 core (e.g., one core in Raspberry Pi 4) is required.

For more information, contact us at https://www.spectacularai.com/#contact.

## Copyright

The examples in this repository are licensed under Apache 2.0 (see LICENSE).

The SDK itself (not included in this repository) is proprietary to Spectacular AI.
The OAK / Depth AI wrapper available in PyPI is free for non-commercial use on x86_64 Windows and Linux platforms.
For commerical licensing options and more SDK variants (ARM binaries & C++ API),
contact us at https://www.spectacularai.com/#contact .
