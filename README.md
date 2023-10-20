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

This Python script is designed to generate and save trajectories for deformable simulations using the PyBullet physics engine. Its primary objective is to simulate interactions between a deformable button cloth and a robot's end effector equipped with a stick, thereby creating a dataset of these interactions.

#### Getting Started

##### Requirements:

- `pybullet`
- `alr_sim` library
- Additional custom modules: `DeformableObject`, `RobotObject`, `Logger`
  
##### Execution

To execute the script, provide four float arguments: x, y, z coordinates, and spring constant:
```
python run_trajectory.py <x> <y> <z> <spring_constant>
```
For example:
```
python run_trajectory.py 0.5 0.2 0.3 100
```

#### Functional Overview

##### Main Function

- Sets up the PyBullet simulation environment with the specified time step.
- Loads a deformable button cloth into the scene and creates anchor positions.
- Loads a stick to the robot's end effector.
- Constructs a trajectory object to handle the simulation and logging.
- Executes the trajectory to generate and save the interaction data.

##### `Trajectory` Class

- Initializes various parameters for the simulation, including spring stiffness, damping, and Neo-Hookean parameters.
- Has a `create_and_save` method which:
  - Moves the robot's end effector to a starting position.
  - Initializes a logger (using `PyBulletDeformableDatasetLogger`) to extract and save deformable interaction data.
  - Begins the simulation, logs the data, and guides the robot to interact with the cloth.
  - Ends the logging and saves the trajectory data.

#### Important Notes

1. The logger, `PyBulletDeformableDatasetLogger`, is used to save the trajectory data and inherits functionalities from the `alr_sim` library.
2. The cloth is loaded as a soft body with adjustable parameters including spring stiffness, damping, and Neo-Hookean attributes.
3. The anchor objects on the cloth are static and used to attach specific vertices of the cloth, functioning as fixed points.
4. The trajectory data is saved in the `./data` directory by default.

#### Limitations

- The interactions are straightforward and linear.
- Only one deformable object, the button cloth, is integrated into the simulation.

#### Tips & Troubleshooting

- Adjusting the simulation's `time_step` parameter can speed up or slow down the simulation. However, setting it too high can lead to unrealistic results.
- Modify the cloth's properties, such as the spring constant or damping values, to achieve different simulation behaviors.



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