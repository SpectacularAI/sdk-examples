# RTAB-Map RGB-D mapping example

This example demonstrates how one could implement their own 3D mapping application on top of the SpectacularAI Mapping API.
In this example, the resulting keyframe data (RGB-D images, point clouds) and the estimated poses are input to the popular [RTAB-Map SLAM library](https://github.com/introlab/rtabmap).
RTAB-Map then builds both, a 3D model and a (re-)localization map of the environment in real-time.

## Dependencies

For access to the C++ SDK, contact us at https://www.spectacularai.com/#contact. The SDK is available for multiple OSes and CPU architectures.

Also, RTAB-Map library and its dependencies must be installed (https://github.com/introlab/rtabmap/wiki/Installation).

For Ubuntu you can do it with:
```
sudo apt-get update
sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev

# (Recommended) g2o
git clone https://github.com/RainerKuemmerle/g2o.git 3rdparty/g2o
cd 3rdparty/g2o
mkdir build && cd build
cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF ..
make -j4
sudo make install

git clone https://github.com/introlab/rtabmap.git 3rdparty/rtabmap
cd 3rdparty/rtabmap/build
cmake ..
make -j4
sudo make install
```

## Build

```
mkdir target && cd target
cmake .. && make
```
or with Azure Kinect and Realsense support
```
cmake -DBUILD_K4A=ON -DBUILD_REALSENSE=ON .. && make
```

## Running

With an existing dataset, run:
```
rtabmap_mapper driver # (driver options: replay, k4a, realsense)
```

Optionally, you can also give the program a RTAB-Map config file with argument `--config path/to/rtabmap_config.ini`.
A config file, mainly optimized for Azure Kinect, is also provided here as `rtabmap_config.ini`.
