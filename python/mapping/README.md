# Spectacular AI mapping scripts

These scripts postprocess data recorded through the Spectacular AI SDK on supported devices, export the resulting 3D maps to various formats, and visualize the process. The scripts are powered by the Spectacular AI _Mapping API_.

## Installation

    pip install -r requirements.txt

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

## Gaussian Splatting

Currently only supports https://github.com/wanmeihuali/taichi_3d_gaussian_splatting.
See the repository for installation instructions. The arguments to the `replay_to_nerf` script are the same as above.

    python replay_to_nerf.py \
        --format=taichi \
        INPUT_PATH \
        /PATH/TO/data/taichi_3d_gaussian_splatting/data/example-splat \
        --preview \
        --key_frame_distance=0.05
    cd /PATH/TO/data/taichi_3d_gaussian_splatting
    # Then, create a config/my_config.yaml with the correponding paths
    # see the instructions printed by replay_to_nerf.py
    python gaussian_point_train.py --train_config config/my_config.yaml


