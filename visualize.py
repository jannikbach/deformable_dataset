import os
import sys

import numpy as np


def main(index=0):
    def visualize(dataset, plot_edges=True):
        import numpy as np
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        # Generate random data for demonstration

        # Plot data shape
        print("deformable mesh shape: ", dataset["deformable_mesh"].shape)
        print("deformable edges shape: ", dataset["deformable_edges"].shape)
        print("deformable faces shape: ", dataset["deformable_faces"].shape)
        print("stick mesh shape: ", dataset["stick_mesh"].shape)
        print("collider: ", dataset["collider"].shape)
        # print("start position: ", dataset["start_pos_stick_xz"][1][0])

        # is_duplicate = len(np.unique(dataset["spring_elastic_stiffness"])) < len(
        #     dataset["spring_elastic_stiffness"]
        # )
        #
        # if is_duplicate:
        #     print("The array contains duplicate values.")
        # else:
        #     print("The array does not contain duplicate values.")

        # bins = np.linspace(100, 1050, 20)
        # data = dataset["spring_elastic_stiffness"].flatten()

        # bins = np.linspace(0.3, 0.5, 40)
        # data = dataset["start_pos_stick_xz"][..., 0].flatten()
        #
        # # bins = np.linspace(0.3, 0.75, 40)
        # # data = dataset["start_pos_stick_xz"][..., 1].flatten()
        #
        # plt.hist(data, bins=bins)
        # plt.xlabel("Range")
        # plt.ylabel("Frequency")
        # plt.title("Histogram")
        # plt.show()

        # Create the figure and 3D axes
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        plt.ion()

        # for t in range(0, len(dataset["deformable_mesh"][1][0]), 1):
        #     ax.clear()
        #     pos = dataset["deformable_mesh"][1][0][t]
        #     x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]
        #
        #     stick_pos = dataset["stick_mesh"][1][0][t]
        #     # print(np.max(stick_pos, axis=0)[1])
        #
        #     sx, sy, sz = stick_pos[:, 0], stick_pos[:, 1], stick_pos[:, 2]
        #
        #     # Plot the scatter points
        #     ax.scatter(x, y, z, c="b", marker="o", s=2)
        #     ax.scatter(sx, sy, sz, c="r", marker="o", s=2)
        #
        #     # Plot the lines connecting selected points
        #     if plot_edges == True:
        #         line_indices = dataset["deformable_edges"][1][0]
        #         for idx1, idx2 in line_indices[:]:
        #             ax.plot(
        #                 [x[idx1], x[idx2]],
        #                 [y[idx1], y[idx2]],
        #                 [z[idx1], z[idx2]],
        #                 c="black",
        #             )
        #
        #     # Set labels and title
        #     ax.set_xlabel("X")
        #     ax.set_ylabel("Y")
        #     ax.set_zlabel("Z")
        #     # ax.set_aspect("equal")  # Setting aspect ratio
        #     ax.set_title(f"Deformable Mesh Visualization for t={t}")
        #     ax.set_ylim3d(-0.0, 0.5)
        #     ax.set_xlim3d(-0, 1.0)
        #     # Show the plot
        #     plt.pause(0.1)

        for t in range(0, len(dataset["deformable_mesh"]), 1):
            ax.clear()
            pos = dataset["deformable_mesh"][t]
            x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]

            stick_pos = dataset["stick_mesh"][t]
            # print(np.max(stick_pos, axis=0)[1])

            sx, sy, sz = stick_pos[:, 0], stick_pos[:, 1], stick_pos[:, 2]

            # Plot the scatter points
            ax.scatter(x, y, z, c="b", marker="o", s=2)
            ax.scatter(sx, sy, sz, c="r", marker="o", s=2)

            # Plot the lines connecting selected points
            if plot_edges == True:
                line_indices = dataset["deformable_edges"]
                for idx1, idx2 in line_indices[:]:
                    ax.plot(
                        [x[idx1], x[idx2]],
                        [y[idx1], y[idx2]],
                        [z[idx1], z[idx2]],
                        c="black",
                    )

            # Set labels and title
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            # ax.set_aspect("equal")  # Setting aspect ratio
            ax.set_title(f"Deformable Mesh Visualization for t={t}")
            ax.set_ylim3d(-0.0, 0.5)
            ax.set_xlim3d(-0, 1.0)
            # Show the plot
            plt.pause(0.1)

        print("done")

    dataset1 = np.load(
        "./data/deformable_dataset_19-09-2023_19-01-16-828407650.npz"
    )

    print(dataset1["stick_edges"])
    print(dataset1["stick_faces"].shape)

    print(dataset1["spring_elastic_stiffness"])

    visualize(dataset1, plot_edges=False)


if __name__ == "__main__":
    index = 0
    if len(sys.argv) > 1:
        index = int(sys.argv[1])

    main(index)
