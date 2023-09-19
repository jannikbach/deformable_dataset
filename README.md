# Deformable Dataset

This project generates datasets for deformable objects using PyBullet.

## Quickstart Guide

1. **Installation**:
    - Install `alrsim` by executing the following command in the base directory:
    ```
    ./SimulationFramework/install.sh
    ```
    - During the installation you will be asked if you want to install Mujoco 2.1 Support. You can skip that since we only use pybullet 
    - For detailed installation instructions, refer to the ```/SimulationFramework/doc/01_installation.md```.

2. **Generate Dataset**:
    - Once the installation is complete, generate the dataset by running:
    ```
    ./start.sh
    ```

## File Descriptions

### create_samples.py

This script generates a dataset of sample configurations based on predefined parameters.

#### Overview

The `main` function of the script carries out the following tasks:

1. Define the number of samples and spring constants.
2. Generate random values for the x and z coordinates within specified ranges that match the franka robots range and the deformable size.
3. Assign a constant value for the y coordinate. Always start with a distance to the deformable before poking it.
4. Assign a spring constant for each sample. Because this is the parameter to adapt the behaviour of the deformable.
5. Combine the generated values into a single dataset.
6. Save the dataset to a CSV file named `samples.csv`.

#### Parameters

- `x`: Randomly generated values between `x_min` (0.2) and `x_max` (0.6).
- `y`: Constant value of -0.3.
- `z`: Randomly generated values between `z_min` (0.2) and `z_max` (0.65).
- `spring`: One of the predefined spring constants repeated for the number of samples.

#### Usage

To run the script and generate the sample configurations, execute:

```python create_samples.py```

After execution, a CSV file named `samples.csv` will be created in the current directory containing the generated samples.



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
