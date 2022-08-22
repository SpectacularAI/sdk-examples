# Spectacular AI C++ SDK for OAK-D

You can find the non-commercial C++ SDK for OAK-D from https://github.com/SpectacularAI/sdk/releases.

For commercial licenses and version for other OS and CPU arthicectures, contact us at https://www.spectacularai.com/#contact.

## Linux

### Quick start

 1. Unpack the SDK archive
 2. Install the system dependencies

        sudo apt install zlib libstdc++ libusb-1.0-0-dev

 3. Attach your OAK-D device to a USB3 port, using a USB3 cable
 4. (optional smoke test) Run the JSONL example:

        cd bin
        ./vio_jsonl
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

 5. Run the Python example (in the `bin` directory)

        pip install matplotlib # install dependencies
        python vio_visu.py

 6. Move the OAK-D around, avoid pointing at blank walls or covering
    the cameras. The plotted trajectory should follow the movement of the device.

### Installation as a library

 1. Install the system dependencies as instructed in "Quick start"
 2. Select where you want the library installed, e.g.,

        export MY_INSTALL_PREFIX=~/.local
        # or: export MY_INSTALL_PREFIX=`pwd`/installed

 2. Run `make PREFIX=$MY_INSTALL_PREFIX install` (or `sudo make install`)
 3. Make sure you have CMake and Git (`sudo apt install cmake git`)
 4. Build and install the Depth AI as a shared library by running the following
    (or see https://github.com/luxonis/depthai-core#dynamic-library for other options)

        make depthai

 5. Build the example using CMake

        make examples


### Running on specific hardware

### Jetson

Running on Jetson (Nano) requires some additional steps in order to get the SDK
working. Luxonis has more detailed at
https://docs.luxonis.com/projects/api/en/latest/install/#jetson, but below are
the minimum required steps.

Open a terminal window and run the following commands:

    sudo apt update && sudo apt upgrade
    sudo reboot now


Change the size of your SWAP. These instructions come from the Getting Started with AI on Jetson from Nvidia:

    # Disable ZRAM:
    sudo systemctl disable nvzramconfig

    # Create 4GB swap file
    sudo fallocate -l 4G /mnt/4GB.swap
    sudo chmod 600 /mnt/4GB.swap
    sudo mkswap /mnt/4GB.swap


(Optional) If you have an issue with the final command, you can try the following:

    sudo vi /etc/fstab

    # Add this line at the bottom of the file
    /mnt/4GB.swap swap swap defaults 0 0

    # Reboot
    sudo reboot now


## Windows

### Quick start

 1. Unpack the SDK archive
 2. Attach your OAK-D device to a USB3 port, using a USB3 cable
 3. (optional smoke test) Run the JSONL example:

        cd bin
        ./vio_jsonl.exe
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

 4. Run the Python example (in the `bin` directory)

        pip install matplotlib # install dependencies
        python vio_visu.py

 5. Move the OAK-D around, avoid pointing at blank walls or covering
    the cameras. The plotted trajectory should follow the movement of the device.


### Using as a library

You must first install DepthAI. Todo this, you need to install following tools if you don't already have them:
* Install Git for Windows https://git-scm.com
* Install Visual Studio Community 2019 https://visualstudio.microsoft.com/vs/community/
  * When launching, install dependencies for "Desktop Development with C++"
* Install CMake https://cmake.org/
* Install Python https://www.python.org/downloads/

Building Depth AI core, open PowerShell window (Shift + Right click in Explorer) in the folder where you extracted the VIO plugin files:

    git clone --recursive https://github.com/luxonis/depthai-core.git --branch v2.17.3
    cd depthai-core
    cmake -G "Visual Studio 16 2019" -A x64 -Bbuild -DBUILD_SHARED_LIBS=ON -DDEPTHAI_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=build
    cmake --build build --config Release -- -m
    cmake --build build --config Release --target install -- -m
    cd ..

Now you are all set to use the VIO plugin. Let's compile the example to see it works:

    cd examples
    mkdir target
    cd target

To properly pass arguments to cmake in PowerShell we need a bit more elaborate command, depending on the shell you are using pick A) or B).
A) In PowerShell:
    Start-Process cmake -ArgumentList "-G ""Visual Studio 16 2019"" -A x64 -DspectacularAI_depthaiPlugin_DIR=..\lib\cmake\spectacularAI -Ddepthai_DIR=..\depthai-core\build\lib\cmake\depthai .." -NoNewWindow

B) In Git Bash or similar:
    cmake -G "Visual Studio 16 2019" -A x64 -DspectacularAI_depthaiPlugin_DIR=../lib/cmake/spectacularAI -Ddepthai_DIR=../depthai-core/build/lib/cmake/depthai ..

Then execute the build:

    cmake --build . --config Release -- -m

Finally running the vio_jsonl.exe should give you the pose of the OAK-D device in real time:

    cd Release
    ./vio_jsonl.exe

Example output:

    {"orientation":{"w":0.7068698348716513,"x":0.02283470213352065,"y":-0.011350438374188287,"z":0.7068838521820336},"position":{"x":-0.016812673347013137,"y":-0.0231306465130168,"z":-0.0013136235444364183},"time":61088.131537828,"velocity":{"x":-3.936320861854323e-06,"y":-1.8569468854259723e-06,"z":0.00031940298071614516}}

## License information

See `share/doc/spectacularAI_depthaiPlugin/LICENSE` for copyright notices
that must be included in all software using this SDK.
