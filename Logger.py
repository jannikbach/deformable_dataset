import abc
import datetime
import os
import time
from enum import Flag, auto

import numpy as np

from alr_sim.core.logger import LoggerBase
from deformable_utils import get_mesh_data, get_mesh_obj_vertices


class PyBulletDeformableDatasetLogger(LoggerBase):
    """
    Logger for the
    - deformable mesh positions
    - deformable edges
    - stick mesh positions

    Resets all logs by calling :func:`start_logging`. Appends current state of each quantity by calling :func:`log_data`.
    Stops logging by calling :func:`stop_logging`.

    :return: no return value
    """

    def __init__(
            self,
            scene,
            deformable_id,
            stick_id,
            deformable_obj_path,
            stick_obj_path,
            anchor_pos,
            start_pos_stick_xz,
            spring_elastic_stiffness,
            spring_damping_stiffness,
            spring_damping_all_directions,
            use_neo_hookean,
            neo_hookean_mu,
            neo_hookean_lambda,
            neo_hookean_damping,
            output_file_path=None,
            save_data=True,
            final_time_stamp_count=100,
            min_y_for_logging=0.15,
    ):
        super().__init__()
        """
        Initialization of the quantities to log.
        """
        from datetime import datetime

        now = datetime.now()
        # dd-mm-YYYY_H-M-S
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")

        if output_file_path is None:
            def_path = os.path.join(
                os.path.split(__file__)[0],
                "..",
                "..",
                "demos",
                "pybullet",
                "robot_control_scene_creation",
            )
            self.output_file_path = os.path.join(
                def_path, "deformable_dataset_" + dt_string + ".npz"
            )
        else:
            self.output_file_path = os.path.join(
                output_file_path,
                "deformable_dataset_"
                + dt_string
                + "-"
                + str(np.random.randint(0, np.iinfo(np.int32).max))
                + ".npz",
            )

        self.scene = scene
        self.deformable_id = deformable_id
        self.deformable_obj_path = deformable_obj_path
        self.stick_obj_path = stick_obj_path
        self.stick_id = stick_id
        self.log_dict_full = None
        self.save_data = save_data
        self.final_time_stamp_count = final_time_stamp_count
        self.min_y_for_logging = min_y_for_logging
        self.anchor_pos = anchor_pos
        self.start_pos_stick_xz = start_pos_stick_xz
        self.spring_elastic_stiffness = spring_elastic_stiffness
        self.spring_damping_stiffness = spring_damping_stiffness
        self.spring_damping_all_directions = spring_damping_all_directions
        self.neo_hookean_mu = neo_hookean_mu
        self.neo_hookean_lambda = neo_hookean_lambda
        self.neo_hookean_damping = neo_hookean_damping
        self.use_neo_hookean = use_neo_hookean

    @property
    def log_dict(self):
        import pybullet as p

        """
        A dictionary of robot properties to log
        """
        log_dict = dict()

        vertices = get_mesh_obj_vertices(self.stick_id)  # rigid object
        _, mesh_deformable = get_mesh_data(p, self.deformable_id)  # deformable object

        log_dict.update(
            {
                "deformable_mesh": mesh_deformable,
                "stick_mesh": vertices,
            }
        )

        return log_dict

    def _start(self, duration=600, log_interval=None, **kwargs):
        """
        Starts logging.

        :return: no return value
        """

        if log_interval is None:
            self.log_interval = self.scene.dt
        else:
            self.log_interval = log_interval

        assert self.log_interval >= self.scene.dt

        self.max_time_steps = int(duration / self.log_interval)

        # limit max time steps to 30m at 1kHz
        if self.max_time_steps > 1800000:
            self.max_time_steps = 1800000

        self.log_dict_full = {
            k: [None] * self.max_time_steps for k, v in self.log_dict.items()
        }

    def _log(self):
        """
        Appends current state of each logged value.

        :return: no return value
        """

        log_properties = self.log_dict

        for k, v in log_properties.items():
            self.log_dict_full[k][self.logged_time_steps] = v

    def _stop(self):
        """
        Stops logging.
        Adds the edge info of the deformable to self.log_dict_full

        maybe it should also save the dict as a pickle file.

        :return: No return value
        """

        if self.logged_time_steps < self.max_time_steps:
            for v in self.log_dict_full.values():
                del v[self.logged_time_steps:]

        def extract_edges_from_obj(obj_file_path):
            faces = []

            # Read the .obj file and extract vertex and face data
            with open(obj_file_path, "r") as obj_file:
                for line in obj_file:
                    if line.startswith("f "):
                        # Extract face vertex indices
                        _, *vertex_indices = line.split()
                        face = []
                        for index in vertex_indices:
                            # Subtract 1 because vertex indices in .obj files are 1-based
                            face.append(int(index.split("/")[0]) - 1)
                        faces.append(face)

            # Extract edges from face definitions
            edges = set()
            for face in faces:
                num_vertices = len(face)
                for i in range(num_vertices):
                    v0 = face[i]
                    v1 = face[(i + 1) % num_vertices]  # Wrap around for the last edge
                    if v0 < v1:
                        edges.add((v0, v1))
                    else:
                        edges.add((v1, v0))

            # Convert the set of edges to a list if desired
            edge_list = list(edges)
            return edge_list, faces

        edge_list, face_list = extract_edges_from_obj(self.deformable_obj_path)
        self.log_dict_full.update({"deformable_edges": edge_list})
        self.log_dict_full.update({"deformable_faces": face_list})

        stick_edge_list, stick_face_list = extract_edges_from_obj(self.stick_obj_path)
        self.log_dict_full.update({"stick_edges": stick_edge_list})
        self.log_dict_full.update({"stick_faces": stick_face_list})

        # save params
        self.log_dict_full.update(
            {"spring_elastic_stiffness": self.spring_elastic_stiffness}
        )
        self.log_dict_full.update(
            {"spring_damping_stiffness": self.spring_damping_stiffness}
        )
        self.log_dict_full.update(
            {"spring_damping_all_directions": self.spring_damping_all_directions}
        )
        self.log_dict_full.update({"use_neo_hookean": self.use_neo_hookean})
        self.log_dict_full.update({"neo_hookean_mu": self.neo_hookean_mu})
        self.log_dict_full.update({"neo_hookean_lambda": self.neo_hookean_lambda})
        self.log_dict_full.update({"neo_hookean_damping": self.neo_hookean_damping})

        for key in self.log_dict_full:
            self.log_dict_full[key] = np.array(self.log_dict_full[key])

        # drop early time steps #all data points where the tip of the stick is not over y coordinate 0.54 are dropped.
        # this means there are 2135 time steps remaining
        mask = (
                np.max(self.log_dict_full["stick_mesh"][:, :, 1], axis=1)
                >= self.min_y_for_logging
        )

        assert (
                self.log_dict_full["stick_mesh"].shape[0]
                == self.log_dict_full["deformable_mesh"].shape[0]
        )

        for key in ["stick_mesh", "deformable_mesh"]:
            self.log_dict_full[key] = self.log_dict_full[key][mask]

        assert (
                self.log_dict_full["stick_mesh"].shape[0]
                == self.log_dict_full["deformable_mesh"].shape[0]
        )

        # now we do a subsampling to only get 100 distinct timesteps
        indices = np.linspace(
            0,
            self.log_dict_full["stick_mesh"].shape[0] - 1,
            self.final_time_stamp_count,
            dtype=int,
        )
        for key in ["stick_mesh", "deformable_mesh"]:
            self.log_dict_full[key] = self.log_dict_full[key][indices]

        assert self.log_dict_full["stick_mesh"].shape[0] == self.final_time_stamp_count
        assert (
                self.log_dict_full["deformable_mesh"].shape[0]
                == self.final_time_stamp_count
        )

        # don't know if this is necessary
        # self.__dict__.update({k: v for k, v in self.log_dict_full.items()})

        # save dict to file
        if self.save_data:
            np.savez(
                self.output_file_path,
                deformable_mesh=self.log_dict_full["deformable_mesh"],
                deformable_edges=self.log_dict_full["deformable_edges"],
                deformable_faces=self.log_dict_full["deformable_faces"],
                stick_mesh=self.log_dict_full["stick_mesh"],
                stick_edges=self.log_dict_full["stick_edges"],
                stick_faces=self.log_dict_full["stick_faces"],
                spring_elastic_stiffness=self.log_dict_full["spring_elastic_stiffness"],
                spring_damping_stiffness=self.log_dict_full["spring_damping_stiffness"],
                spring_damping_all_directions=self.log_dict_full[
                    "spring_damping_all_directions"
                ],
                use_neo_hookean=self.log_dict_full["use_neo_hookean"],
                neo_hookean_mu=self.log_dict_full["neo_hookean_mu"],
                neo_hookean_lambda=self.log_dict_full["neo_hookean_lambda"],
                neo_hookean_damping=self.log_dict_full["neo_hookean_damping"],
                collider=self.anchor_pos,
                start_pos_stick_xz=self.start_pos_stick_xz,
            )

    def _check_log_interval(self) -> bool:
        if (
                self.last_log_time is None
                or self.robot.time_stamp >= self.last_log_time + self.log_interval
        ):
            return True
        return False
