#!/bin/bash

python create_samples.py


# Name of the csv file
file="samples.csv"

# Loop while the csv file is not empty
while [ -s $file ]
do
    # Read the first line of the file
    line=$(head -n 1 $file)

    # Split the line into four parameters
    IFS=',' read -ra params <<< "$line"

    # Call the Python script with the parameters
    python run_trajectory.py ${params[0]} ${params[1]} ${params[2]} ${params[3]}

    # Remove the first line from the file
    sed -i '1d' $file
done

# Remove the csv file
rm $file

python aggregate_trajectories.py
