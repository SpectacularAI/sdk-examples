# RTAB-Map RGB-D mapping example

This example demonstrates how one could implement their own 3D mapping application on top of the SpectacularAI Mapping API.
In this example, the resulting keyframe data (RGB-D images, point clouds) and the estimated poses are input to the popular [RTAB-Map SLAM library ](https://github.com/introlab/rtabmap).
RTAB-Map then builds both, a 3D model and a (re-)localization map of the environment in real-time.

## Dependencies

For access to the C++ SDK, contact us at https://www.spectacularai.com/#contact. The SDK is available for multiple OSes and CPU architectures.

Also, RTAB-Map library and its dependencies must be installed (https://github.com/introlab/rtabmap/wiki/Installation).

For Ubuntu you can do it with:
```
sudo apt-get update
sudo apt-get install libsqlite3-dev libpcl-dev libopencv-dev git cmake libproj-dev libqt5svg5-dev

# (Recommended) g2o
git clone https://github.com/RainerKuemmerle/g2o.git wrappers/rtabmap/3rdparty/g2o
cd wrappers/rtabmap/3rdparty/g2o
mkdir build && cd build
cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF ..
make -j4
sudo make install

git clone https://github.com/introlab/rtabmap.git wrappers/rtabmap/3rdparty/rtabmap
cd wrappers/rtabmap/3rdparty/rtabmap/build
cmake ..
make -j4
sudo make install
```

## Build

```
mkdir target && cd target
cmake .. && make
```

## Running

With an existing dataset, run:
```
rtabmap_mapper -i path/to/dataset -o path/to/rtabmap_output.db
```

Optionally, you can also give the program a RTAB-Map config file with argument `-c path/to/rtabmap_config.ini`.
A config file that uses SIFT features, g2o optimizer and gravity constraints in graph optimization is also provided here as `rtabmap_config.ini`.
