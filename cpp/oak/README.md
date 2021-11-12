# Spectacular AI C++ SDK for OAK-D

For access to the C++ SDK, contact us at https://www.spectacularai.com/#contact .
Available for multiple OSes and CPU architectures.

## Running on specific hardware

## Jetson

Running on Jetson (Nano) requires some additional steps in order to get the SDK working. Luxonis has more detailed [instructions](https://docs.luxonis.com/projects/api/en/latest/install/#jetson) on their website, but below are the minimum required steps.

Open a terminal window and run the following commands:
```bash
    sudo apt update && sudo apt upgrade
    sudo reboot now
```

Change the size of your SWAP. These instructions come from the Getting Started with AI on Jetson from Nvidia:
```bash
    # Disable ZRAM:
    sudo systemctl disable nvzramconfig

    # Create 4GB swap file
    sudo fallocate -l 4G /mnt/4GB.swap
    sudo chmod 600 /mnt/4GB.swap
    sudo mkswap /mnt/4GB.swap
```

(Optional) If you have an issue with the final command, you can try the following:
```bash
    sudo vi /etc/fstab

    # Add this line at the bottom of the file
    /mnt/4GB.swap swap swap defaults 0 0

    # Reboot
    sudo reboot now
```
