# Spectacular AI mapping scripts

These scripts post-process data recorded through the Spectacular AI SDK on supported devices, export the resulting 3D maps to various formats, and visualize the process. The scripts are powered by the Spectacular AI _Mapping API_ 
([documentation](https://spectacularai.github.io/docs/sdk/python/latest/#module-spectacularAI.mapping)).

## Installation

**Requirements** (current): Linux + CUDA-enabled Nvidia GPU that can run Nerfstudio

    pip install -r requirements.txt

## Quick start

Plug in an OAK-D device and run

    ./record_and_process_oak_d.sh
    # Then follow the instructions printed to the terminal

With OAK-D or RealSense devices, you can currently expect to be able to map "table-sized" scenes
quite fast and accurately. Move slow while mapping and shoot from different angles to increase quality.

## Nerfstudio export

Here's how to convert Spectacular AI -recorded data on Kinect, RealSense or other supported devices to NeRFs.
First, make sure you have correctly installed [Nerfstudio](https://github.com/nerfstudio-project/nerfstudio),
then run:

    python replay_to_nerf.py INPUT_PATH \
        /example/output/path/my-nerf \
        --preview --key_frame_distance=0.05
    # run in the Nerfstudio folder, if necessary
    ns-train nerfacto --data /example/output/path/my-nerf

Where

 * `--key_frame_distance` should be set based on the recorded scene size: `0.05` (5cm) is good for small scans and `0.15` for room-sized scans.
 * `INPUT_PATH` is the dataset recorded using the SDK (i.e., the value of `recordingFolder`)
 * `/example/output/path/my-nerf` (placeholder) is the output folder of this script and the input to Nerfstudio

If the processing gets slow, you can also try adding a `--fast` flat to `replay_to_nerf.py` to trade off quality for speed.
Without the `--fast` flag, the processing should take around 10 minutes.

## Gaussian Splatting

**2023-11-14**. Currently supports the Nerfstudio branch powered by [gsplat](https://github.com/nerfstudio-project/gsplat).
Installation instructions: 

 1. Install Nerfstudio and activate the Conda environment
 2. Install _gsplat_ to the same Conda environment `git clone https://github.com/nerfstudio-project/gsplat; cd gsplat; pip install -e .`
 3. Train as `ns-train gaussian-splatting --data /example/output/path/my-nerf`


