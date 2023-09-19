# Deformable Dataset

This project generates datasets for deformable objects using PyBullet.

## Quickstart Guide

1. **Installation**:
    - Install `alrsim` by executing the following command in the base directory:
    ```
    ./SimulationFramework/install.sh
    ```
    - During the installation you will be asked if you want to install Mujoco 2.1 Support. You can skip that since we only use pybullet 
    - For detailed installation instructions, refer to the [alrsim installation guide](./SimulationFramework/doc/01_installation.md).

2. **Generate Dataset**:
    - Once the installation is complete, generate the dataset by running:
    ```
    ./start.sh
    ```

## File Descriptions

### create_samples.py
*Description coming soon.*

### run_trajectory.py
*Description coming soon.*

### aggregate_trajectories.py
*Description coming soon.*

### deformable_utils.py
*Description coming soon.*

### start.sh
*Description coming soon.*

## Tips and Tricks

1. **Use Deformable World in PyBullet**: When working with deformable objects, it's recommended to use the deformable world feature in PyBullet.
2. **PyBullet Deformable Documentation**: Documentation on PyBullet's deformable features can be found in the [pybullet quickstart guide](https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/pybullet_quickstartguide.pdf).
