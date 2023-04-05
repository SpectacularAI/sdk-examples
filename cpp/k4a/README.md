# Spectacular AI SDK for Azure Kinect DK

# Mapping visualization example

This example use Azure Kinect device with Mapping API and serializes the output in custom format tha can be sent to Python for visualization.

* Tested platforms: Linux
* Dependencies: CMake, Azure Kinect drivers

## Setup

* Install the Spectacular AI SDK
* Build this example using CMake:

```
mkdir target
cd target
cmake  -DspectacularAI_k4aPlugin_DIR=<path/to/spectacularai-sdk/lib/cmake/spectacularAI/>
make
```

The `-DspectacularAI_k4aPlugin_DIR` option is not needed is you have used `sudo make install` for the SDK.

## Usage

Create a new pipe with:
```
mkfifo ~/my_pipe
```

Launch python visualization and leave it running with:
```
python mapping_visu.py --file ~/my_pipe
```

And finally launch this example file with:
```
./mapping_visu -o ~/my_pipe
```

You should now the visualization running in Open3D window.

You can add -r recording folder to additionally record the session:
```
./mapping_visu -o ~/my_pipe -r <recording_folder>
```

# Copyright

For access to the C++ SDK, contact us at <https://www.spectacularai.com/#contact>.

Available for multiple OSes and CPU architectures.
