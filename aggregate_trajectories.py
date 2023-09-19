import os

import numpy as np

# Initialize an empty dictionary to store the values
data_dict = {}

# Directory where the .npz files are located
directory_path = "./log_cloth"

num_samples = 300

for filename in os.listdir(directory_path):
    if filename.endswith(".npz"):
        file_path = os.path.join(directory_path, filename)

        with np.load(file_path) as data:
            spring_value = data["spring_elastic_stiffness"].item()

            # If the spring value is not in the dictionary, initialize a new entry
            if spring_value not in data_dict:
                data_dict[spring_value] = {key: [] for key in data.files}

            # Check if already processed 300 files for this spring_value
            if len(data_dict[spring_value]["spring_elastic_stiffness"]) < num_samples:
                # Append the arrays to the respective lists
                for key in data.files:
                    data_dict[spring_value][key].append(data[key])

# Convert the lists to numpy arrays and append a new dimension
for spring_value in data_dict:
    for key in data_dict[spring_value]:
        data_dict[spring_value][key] = np.array(data_dict[spring_value][key])

# Create a final dictionary to store the aggregated data
final_data = {}
unique_spring_values = sorted(list(data_dict.keys()))

for key in data_dict[unique_spring_values[0]]:
    final_data[key] = np.stack([data_dict[spring_value][key] for spring_value in unique_spring_values])

# Use np.array and np.newaxis to reshape it to [3,1]
final_data["spring_elastic_stiffness"] = np.array(unique_spring_values)[:, np.newaxis]


# Save the final data to a new .npz file
np.savez("dataset3x300_long.npz", **final_data)