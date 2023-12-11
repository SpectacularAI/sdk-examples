

![OAK-D NeRF](https://spectacularai.github.io/docs/gif/oak-d-nerf.gif)
<img height="253" alt="OAK-D fast mapping" src="https://spectacularai.github.io/docs/png/oak-d-fast-mapping.png">

# Spectacular AI mapping tools

This page has instructions for post-processing data recorded through the Spectacular AI SDK on supported devices, exporting to Nerfstudio, training NeRFs and 3DGS, and visualizing the process. The Spectacular AI mapping tool (`sai-cli process`) is powered by the Spectacular AI _Mapping API_ 
([documentation](https://spectacularai.github.io/docs/sdk/python/latest/#module-spectacularAI.mapping)).

## Installation

 1. [install Nerfstudio](https://github.com/nerfstudio-project/nerfstudio#1-installation-setup-the-environment) (**Requirement**: a good NVidia GPU + CUDA).
 2. Install FFmpeg
    * Linux `apt install ffmpeg` (or similar, if using another package manager)
    * Windows: [see here](https://www.ffmpeg.org/download.html#build-windows). FFmpeg must be in your `PATH` so that `ffmpeg` works on the command line).
       
 3. In Nerfstudio's Conda environment, install the Spectacular AI Python library with all recommended dependencies:

        pip install spectacularAI[full]

## Recording data

Choose your device below to see more detailed instructions for creating Spectacular AI recordings (folders or zip files):

<details><summary><b>iPhone with LiDAR</b></summary><p>
   
 1. Download [Spectacular Rec](https://apps.apple.com/us/app/spectacular-rec/id6473188128) from App Store.
 2. See our [instruction video on YouTube](https://youtu.be/d77u-E96VVw) on how to create recording files and transfer them to your computer.

</p></details>

<details><summary><b>OAK-D</b></summary><p>

 1. Plug in the OAK-D to your laptop (or directly the computer with the heavy GPU)
 2. Run `sai-cli record oak --no_feature_tracker --resolution=800p`.

If the above settings cause issues, try running `sai-cli record oak` instead. Coming soon: 🌈 colors.

</p></details>

<details><summary><b>RealSense D455/D435i</b></summary><p>
See the <i>Recording data</i> item under the <a href="https://github.com/SpectacularAI/sdk-examples/tree/main/cpp/realsense#quick-start">RealSense example folder</a>

</p></details>

<details><summary><b>Azure Kinect DK</b></summary><p>

Download our binary recorder [here](https://github.com/SpectacularAI/sdk/releases/download/v1.24.0/spectacularAI_k4aPlugin_cpp_non-commercial_1.24.0.zip) and see the README within for recording instructions.

</p></details>

<details><summary><b>Coming soon</b></summary><p>

 * Android phones
 * Orbbec Femto

</p></details>

With OAK-D or RealSense devices, you can currently expect to be able to map "table-sized" scenes
quite fast and accurately. Move slow while mapping and shoot from different angles to increase quality.

## Nerfstudio export and training

First run our conversion script and then Nerstudio training as

    sai-cli process INPUT_PATH --preview3d --key_frame_distance=0.05 /example/output/path/my-nerf
    ns-train nerfacto --data /example/output/path/my-nerf

Where

 * `INPUT_PATH` is the dataset folder recorded using _Spectacular Rec_ or our other recording tools (the value of `recordingFolder` if using the SDK directly)
 * `/example/output/path/my-nerf` (placeholder) is the output folder of this script and the input to Nerfstudio
 * `--key_frame_distance` should be set based on the recorded scene size: `0.05` (5cm) is good for small scans and `0.15` for room-sized scans.
 * `--preview3d` (optional flag) shows you a 3D preview of the point cloud and estimated trajectory (not the final ones).

If the processing gets slow, you can also try adding a `--fast` flag to `sai-cli process` to trade off quality for speed.
Without the `--fast` flag, the processing should take around 10 minutes tops.

### Gaussian Splatting

**2023-11-14**. Currently supports the Nerfstudio [branch](https://github.com/nerfstudio-project/nerfstudio/pull/2521) powered by [gsplat](https://github.com/nerfstudio-project/gsplat).
Installation instructions: 

 1. Create and activate a Conda environment for Nerfstudio (see [instructions](https://github.com/nerfstudio-project/nerfstudio#create-environment))
 2. Install _gsplat_ to the same Conda environment:
    
        git clone https://github.com/nerfstudio-project/gsplat
        cd gsplat
        pip install -e .
    
 4. Install the `gaussian-splatting` branch of Nerfstudio:
    
        git clone https://github.com/nerfstudio-project/nerfstudio.git
        cd nerfstudio
        git checkout gaussian-splatting  # <--- Different from standard instructions
        pip install --upgrade pip setuptools
        pip install -e .

 5. Train as `ns-train gaussian-splatting --data /example/output/path/my-nerf`

## License note

Spectacular AI SDK is free to use for non-commercial purposes. [Contact us](https://www.spectacularai.com/#contact) for commercial licensing (e.g., running this in your own cloud service).
Nerfstudio and FFMpeg are used under their own licenses.
