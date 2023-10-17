# Deformable Dataset

This project generates datasets for deformable objects using PyBullet.

## Quickstart Guide

1. **Installation**:
    - Install `alrsim` by executing the following command in the base directory:
    ```
    ./SimulationFramework/install.sh
    ```
    - During the installation you will be asked if you want to install Mujoco 2.1 Support. You can skip that since we only use `pybullet`.
    - For detailed installation instructions, refer to the ```/SimulationFramework/doc/01_installation.md```.
    - Make sure to use the conda environment created during the installation.

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

#### Description

The Python script focuses on generating and saving trajectories for deformable simulations using the PyBullet physics engine. The main objective is to create a deformable button cloth and move a robot's end effector with an attached stick to simulate interactions, thereby generating a dataset of these interactions.

#### Getting Started

##### Requirements:

- `alr_sim` library
- Additional custom modules: `Logger`, `deformable_utils`

##### Execution

To run the script, you need to provide four float arguments: x, y, z coordinates, and spring constant:
```python run_trajectory.py <x> <y> <z> <spring_constant>```
For example:
```python trajectory_generation.py 0.5 0.2 0.3 100```


#### Functional Overview

##### Main Function

- Initializes the simulation environment.
- Loads a deformable button cloth into the scene.
- Calculates anchor positions on the cloth.
- Attaches anchors to the deformable cloth.
- Adds a stick to the robot's end effector.
- Calls the `create_and_save_trajectory` function to generate the trajectory and log the data.

##### `create_and_save_trajectory` Function

- Moves the robot's end effector to a starting position.
- Initializes a logger to extract and save deformable data.
- Starts logging the simulation data.
- Directs the robot to go to a certain position in the simulation, simulating the interaction.
- Stops logging and saves the trajectories of the interacting objects.

#### Important Notes

1. The script utilizes the `PyBulletDeformableDatasetLogger` from `Logger.py` for logging purposes. This class inherits from `alr_sim.core.logger.LoggerBase`.
2. The cloth is loaded as a soft body with various parameters like spring stiffness, damping, and Neo-Hookean parameters, which can be adjusted as needed.
3. The anchors created are static objects used to attach specific vertices of the cloth. They act as fixed points.
4. The robot's end effector is equipped with a stick to interact with the cloth.
5. Trajectory data is saved in the `./data` directory by default.

#### Limitations

- Only implements straight forward interactions.
- Only integrates one deformable object (button cloth).

#### Tips & Troubleshooting
- Adjust the simulation's `time_step` parameter to speed up or slow down the simulation. Making it to big will lead to unrealistic simulations.
- For different simulation behaviors, tweak the cloth's properties, such as the spring constant or damping values.




### aggregate_trajectories.py

#### Description

This script processes multiple `.npz` files from the directory `./data` to aggregate their data based on the `spring_elastic_stiffness` value. The primary objective is to group the data from these files, and store the aggregated data into a new `.npz` file named `dataset.npz`.

#### How it works

1. **Directory Initialization**: The script looks for `.npz` files in the directory `./data`.
2. **Data Processing**:
    - Each `.npz` file's data is loaded into a dictionary, `data_dict`, indexed by its `spring_elastic_stiffness` value.
    - For each unique spring value, the script attempts to process up to 300 samples.
    - The data from each file is appended to a list associated with its spring value in the dictionary.
3. **Data Aggregation**:
    - The lists in the dictionary are converted to numpy arrays.
    - A new dictionary, `final_data`, is created to store the aggregated data arrays.
    - The arrays from `data_dict` are stacked along a new dimension, with the order determined by the sorted unique spring values.
    - The spring values themselves are reshaped and stored in `final_data`.
4. **Data Saving**: The `final_data` dictionary is saved to a new `.npz` file named `dataset.npz`.

#### Requirements

- Python 3.9
- Libraries: `os`, `numpy`

#### Usage

1. Ensure that the directory `./data` contains the `.npz` files you want to process.
2. Run the script.

After the script completes its execution, you should find a new `.npz` file named `dataset.npz` in the current directory.



### start.sh

#### Description

This bash script orchestrates the process of sample creation, trajectory execution, and data aggregation. It reads parameters from a CSV file and uses them to execute Python scripts for trajectory generation and data aggregation. The workflow can be summarized as follows:

1. Invoke the `create_samples.py` script to generate sample parameters.
2. Process each line of the generated `samples.csv` file as parameters for trajectory generation.
3. For each line in `samples.csv`, invoke the `run_trajectory.py` script with the extracted parameters.
4. After processing a line from the CSV, the line is removed.
5. After all lines are processed and `samples.csv` is empty, the file is deleted.
6. Invoke the `aggregate_trajectories.py` script to collect and aggregate trajectory data.

#### How it works

1. **Sample Creation**: 
    - The script starts by running `create_samples.py`.
    - This is expected to produce a `samples.csv` file with parameters for trajectories.
2. **Trajectory Execution**:
    - For each line in `samples.csv`, the script extracts four parameters.
    - It then calls `run_trajectory.py` with these parameters.
    - After a line is processed, it's removed from the CSV file.
3. **Data Aggregation**:
    - Once all lines from the CSV are processed and the file is empty, it's deleted.
    - The script then runs `aggregate_trajectories.py` to process and aggregate the trajectory data.

#### Usage

1. Ensure the Python scripts (`create_samples.py`, `run_trajectory.py`, and `aggregate_trajectories.py`) are in the same directory as this bash script.
2. Make the bash script executable: `chmod +x start.sh`
3. Run the script: `./start.sh`



## Tips and Tricks

1. **Use Deformable World in PyBullet**: When working with deformable objects, it's recommended to use the deformable world feature in PyBullet.
2. **PyBullet Deformable Documentation**: Documentation on PyBullet's deformable features can be found in the [pybullet quickstart guide](https://raw.githubusercontent.com/bulletphysics/bullet3/master/docs/pybullet_quickstartguide.pdf).
3. **Resetting the pybullet Simulation**: Resetting the deformable simulation is buggy. It only detects one trajectory, even if you run the experiment several times. The simulation has to be rerun for every trajectory.