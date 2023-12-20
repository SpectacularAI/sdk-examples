# Spectacular AI SDK for Orbbec

You can find the non-commercial C++ SDK for Orbbec from https://github.com/SpectacularAI/sdk/releases.

For commercial licenses, contact us at https://www.spectacularai.com/#contact.

## Linux

### Quick start

 1. Unpack the SDK archive
 2. If you have not used the Orbbec device before, you will need to setup udev rules. Either, run

        sudo ./bin/3rdparty/OrbbecSDK/install_udev_rules.sh

    or follow the official instructions in [OrbbecSDK](https://github.com/orbbec/OrbbecSDK?tab=readme-ov-file#environment-setup).
 3. Attach your Orbbec device to a USB3 port
 4. Run the JSONL example:

        cd bin
        ./vio_jsonl
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

 5. Run the Python example (in the `bin` directory)

        pip install matplotlib # install dependencies
        python vio_visu.py

### Installation as a library

 1. Install [OrbbecSDK](https://github.com/orbbec/OrbbecSDK)
 2. Select where you want the library installed, e.g.,

        export MY_INSTALL_PREFIX=~/.local
        # or: export MY_INSTALL_PREFIX=`pwd`/installed

 3. Run `make PREFIX=$MY_INSTALL_PREFIX install` (or `sudo make install`)
 4. Make sure you have CMake and Git (`sudo apt install cmake git`)
 5. Build the vio_jsonl example using CMake

        make PREFIX=$MY_INSTALL_PREFIX examples

## Windows

### Quick start

 1. Unpack the SDK archive
 2. If you have not used the Orbbec device before, you will need to setup timestamp registration. Please follow the official instructions in [OrbbecSDK](https://github.com/orbbec/OrbbecSDK?tab=readme-ov-file#environment-setup).
 3. Attach your OrbbecSDK device to a USB3 port
 4. Run the JSONL example:

        cd bin
        ./vio_jsonl.exe
        # Now you should see rapidly flowing JSONL text
        # press Ctrl+C to exit

 5. Run the Python example (in the `bin` directory)

        pip install matplotlib # install dependencies
        python vio_visu.py

### Installation as a library

You need to install following tools if you don't already have them:
* Install [OrbbecSDK](https://github.com/orbbec/OrbbecSDK)
* Install Git for Windows https://git-scm.com
* Install Visual Studio Community 2019 https://visualstudio.microsoft.com/vs/community/
  * When launching, install dependencies for "Desktop Development with C++"
* Install CMake https://cmake.org/
* Install Python https://www.python.org/downloads/

 1. Build vio_jsonl example using CMake. The process is a bit involved so the commands are collected in `./examples/build_windows.sh`. In PowerShell:

        cd examples
        $env:BUILD_MAPPING_VISU="OFF"; sh ./build_windows.sh ..\lib\cmake\spectacularAI <path\to\OrbbecSDK\lib>

 2. Finally running the vio_jsonl.exe should give you the pose of the Orbbec device in real time:

        cd target/Release
        ./vio_jsonl.exe

## Mapping visualization example

This example uses Orbbec device with Mapping API and serializes the output in custom format that can be sent to Python for visualization.

### Dependencies

Install Python visualization dependencies:
`pip install spectacularAI[full]`

### Usage

If you are using the pre-built `./bin/mapping_visu` binary or (have built your own following the building instructions in the next section):

Create a new pipe with (Linux):
```
mkfifo /path/to/my_pipe
```

or

Create a new file with (Windows):
```
type NUL > /path/to/my_pipe
```

Launch this example file with:
```
./mapping_visu -o /path/to/my_pipe
```

Open another command prompt, and launch Python visualization (in `./bin` directory or https://github.com/SpectacularAI/sdk-examples/blob/main/python/oak/mapping_visu.py) and leave it running with:
```
python mapping_visu.py --file /path/to/my_pipe
```

Ideally, you would start both programs around the same time (~10 seconds), so that not too much outputs get queued in the file.
You should now the visualization running in the Python window.

You can add -r recording folder to additionally record the session:
```
./mapping_visu -o /path/to/my_pipe -r <recording_folder>
```

Also, you can adjust all the Python visualization options, see `python mapping_visu.py -h`

### Building (Linux)

* Build this example using CMake:

```
mkdir target
cd target
cmake -DspectacularAI_orbbecPlugin_DIR=<path/to/spectacularAI_orbbecPlugin/install/lib/cmake/spectacularAI/> ..
make
```

The `-DspectacularAI_orbbecPlugin_DIR` option is not needed is you have used `sudo make install` for the SDK.

### Building (Windows)

1. Build vio_jsonl example using CMake. The process is a bit involved so the commands are collected in `./build_windows.sh`. In PowerShell:

        sh ./build_windows.sh <path\to\spectacularAI_orbbecPlugin_1.26.0_windows\lib\cmake\spectacularAI> <path\to\OrbbecSDK\lib>

## License information

See `share/doc/spectacularAI_orbbecPlugin/LICENSE` for copyright notices
that must be included in all software using this SDK.
