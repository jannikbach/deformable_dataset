import numpy as np
import sys


def main(index=0):
    # create sample configs

    # Define the number of samples
    num_samples = 300
    spring_constant = [100, 500, 1000]

    # Generate random data for each column with specified ranges
    x_min = 0.2
    x_max = 0.6
    z_min = 0.2
    z_max = 0.65

    x = np.random.uniform(x_min, x_max, len(spring_constant) * num_samples).reshape(
        -1, 1
    )
    y = np.full((len(spring_constant) * num_samples, 1), -0.3)
    z = np.random.uniform(z_min, z_max, len(spring_constant) * num_samples).reshape(
        -1, 1
    )
    # spring = np.full((num_samples, 1), spring_constant).reshape(-1, 1)
    spring = np.zeros((len(spring_constant) * num_samples, 1))
    border = 0
    for springs in spring_constant:
        spring[border : border + num_samples] = springs
        border = border + num_samples

    samples = np.concatenate((x, y, z, spring), axis=1)

    np.savetxt("samples.csv", samples, delimiter=",")

    print("Goodbye")


if __name__ == "__main__":
    main()
