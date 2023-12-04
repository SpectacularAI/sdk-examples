# Spectacular AI C++ SDK for RealSense depth cameras

You can find the non-commercial C++ SDK for RealSense from https://github.com/SpectacularAI/sdk/releases.

For commercial licenses and version for other OS and CPU architectures, contact us at https://www.spectacularai.com/#contact.

## Linux

### Quick start

 1. If you have not used the RealSense device before, you will need to setup udev rules. Either, run

        ./bin/3rdparty/librealsense/setup_udev_rules.sh

       or run from the [librealsense GitHub repository](https://github.com/IntelRealSense/librealsense):

        ./scripts/setup_udev_rules.sh

 2. Attach your RealSense D4XX or D3XX device to a USB3 port, using a USB3 cable

 3. Unpack the Spectacular AI SDK archive

 4. In the extracted directory for your platform, run the pre-compiled example binary:

        cd bin
        ./vio_jsonl
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

 5. Run the Python example (in the `bin` directory)

        pip install matplotlib # install dependencies
        python vio_visu.py

 6. Move the RealSense device around, avoid pointing at blank walls or covering
    the cameras. The plotted trajectory should follow the movement of the device.

 7. Recording data (for troubleshooting)

        ./vio_jsonl /path/to/recording_folder

### Installation as a library

 1. Select where you want the library installed, e.g.,

        export MY_INSTALL_PREFIX=~/.local

 2. Run `make PREFIX=$MY_INSTALL_PREFIX install` (or `sudo make install`)
 3. Install librealsense https://github.com/IntelRealSense/librealsense
 4. Ensure you have CMake
 5. Build the vio_jsonl example using CMake

        make MY_INSTALL_PREFIX=~/.local example

## Windows

### Quick start

 1. Unpack the sdk archive
 2. Attach your RealSense device to a USB3 port, using a USB3 cable
 3. (optional smoke test) Run the JSONL example:

        cd bin
        ./vio_jsonl.exe
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

### Using as a library

First install the following tools if you don't already have them:
* Install Git for Windows https://git-scm.com
* Install Visual Studio Community 2019 https://visualstudio.microsoft.com/vs/community/
  * When launching, install dependencies for "Desktop Development with C++"
* Install CMake https://cmake.org/
* Install Python https://www.python.org/downloads/

Use Git Bash or similar for next steps.

Install librealsense from sources:

    cd examples
    git clone https://github.com/IntelRealSense/librealsense
    mkdir librealsense/target
    cd librealsense/target
    cmake -G "Visual Studio 16 2019" -A x64 -DCMAKE_INSTALL_PREFIX=./install -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF ..
    cmake --build . --config Release --target install
    cd ../..

Now you are all set to use the VIO plugin. Let's compile the example to see it works:

    mkdir target
    cd target
    cmake -G "Visual Studio 16 2019" -A x64 -DBUILD_MAPPER=OFF -DspectacularAI_realsensePlugin_DIR=../lib/cmake/spectacularAI -Drealsense2_DIR=./librealsense/target/install/lib/cmake/realsense2 ..
    cmake --build . --config Release -- -m


Finally running the vio_jsonl.exe should give you the pose of the RealSense device in real time:

    ./Release/vio_jsonl.exe

Example output:

    {"orientation":{"w":0.7068698348716513,"x":0.02283470213352065,"y":-0.011350438374188287,"z":0.7068838521820336},"position":{"x":-0.016812673347013137,"y":-0.0231306465130168,"z":-0.0013136235444364183},"time":61088.131537828,"velocity":{"x":-3.936320861854323e-06,"y":-1.8569468854259723e-06,"z":0.00031940298071614516}}

## License information

See `share/doc/` for copyright notices that must be included in all software
using this SDK or librealsense.
