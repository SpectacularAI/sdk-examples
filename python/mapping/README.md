

![OAK-D NeRF](https://spectacularai.github.io/docs/gif/oak-d-nerf.gif)

# Gaussian Splatting & NeRFs

This page has instructions for post-processing data recorded through the Spectacular AI SDK on supported devices, exporting to Nerfstudio, training NeRFs and 3DGS, and visualizing the process. The Spectacular AI mapping tool (`sai-cli process`) is powered by the Spectacular AI [Mapping API](https://spectacularai.github.io/docs/sdk/mapping.html).

## Installation

These instructions assume you want to train NeRFs or 3DGS using Nerfstudio. For other uses, Nerfstudio and CUDA are not required, but you simply need Python, `pip` and FFmpeg.

 1. [install Nerfstudio](https://github.com/nerfstudio-project/nerfstudio#1-installation-setup-the-environment) (**Requirement**: a good NVidia GPU + CUDA).
 2. Install FFmpeg. Linux `apt install ffmpeg` (or similar, if using another package manager). Windows: [see here](https://www.editframe.com/guides/how-to-install-and-start-using-ffmpeg-in-under-10-minutes). FFmpeg must be in your `PATH` so that `ffmpeg` works on the command line.
 3. In Nerfstudio's Conda environment, install the Spectacular AI Python library with all recommended dependencies:

        pip install spectacularAI[full]

## Recording data

Choose your device below to see more detailed instructions for creating Spectacular AI recordings (folders or zip files):

<details><summary><b>iPhone</b> (with or without LiDAR)</summary><p>

 1. Download [Spectacular Rec](https://apps.apple.com/us/app/spectacular-rec/id6473188128) from App Store.
 2. See our [instruction video on YouTube](https://youtu.be/d77u-E96VVw) on how to create recording files and transfer them to your computer.

</p></details>

<details><summary><b>Android</b> (with or without ToF)</summary><p>

 1. Download [Spectacular Rec](https://play.google.com/store/apps/details?id=com.spectacularai.rec) from Play Store.
 2. Use like the iPhone version (tutorial here [here](https://youtu.be/d77u-E96VVw))

</p></details>

<details><summary><b>OAK-D</b></summary><p>

 1. Plug in the OAK-D to your laptop (or directly the computer with the heavy GPU)
 2. Run `sai-cli record oak --no_feature_tracker --resolution=800p`.

If the above settings cause issues, try running `sai-cli record oak` instead.

</p></details>

<details><summary><b>RealSense D455/D435i</b></summary><p>
See the <a href="https://spectacularai.github.io/docs/sdk/wrappers/realsense.html#recording-data">Recording data</a> section under the RealSense wrapper instructions

</p></details>

<details><summary><b>Azure Kinect DK</b></summary><p>

See the <a href="https://spectacularai.github.io/docs/sdk/wrappers/k4a.html">Kinect wrapper page</a> for more information

</p></details>

<details><summary><b>Orbbec</b></summary><p>

See the <a href="https://spectacularai.github.io/docs/sdk/wrappers/orbbec.html#recording-data">Recording data</a> section under the Orbbec wrapper instructions

</p></details>

---

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

Update Nerfstudio and train as

    ns-train gaussian-splatting --data /example/output/path/my-nerf

To use the resulting "splats" in other tools, first export as PLY

    ns-export gaussian-splat \
        --load-config outputs/my-nerf/gaussian-splatting/DATE/config.yaml
        --output-dir exports/splats

Then copy the the file `exports/point_cloud.ply`. Examples:

 * Edit in [Super Splat](https://playcanvas.com/super-splat) (splat colors may look wrong here)
 * Export to `.splat` or [stand-alone HTML](https://spectacularai.github.io/docs/other/android-3dgs-example-ramen.html)
   using [SpectacularAI/point-cloud-tools](https://github.com/SpectacularAI/point-cloud-tools#gaussian-splatting)
 * View or embed `.splat` to a web page using [gsplat.js](https://github.com/huggingface/gsplat.js)

The export process can also be customized by modifying the source code of [`sai-cli process`](https://github.com/SpectacularAI/sdk/blob/main/python/cli/process/process.py)
which can also be used as a standalone Python script.

## License note

Spectacular AI SDK is free to use for non-commercial purposes. [Contact us](https://www.spectacularai.com/#contact) for commercial licensing (e.g., running this in your own cloud service).
Nerfstudio and FFMpeg are used under their own licenses.
