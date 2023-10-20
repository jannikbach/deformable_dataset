import os
from abc import ABC, abstractmethod

import numpy as np
import pybullet as p

from alr_sim.utils import euler2quat
from deformable_utils import get_mesh_data, get_closest, create_anchor_geom, attach_anchor


# Abstract Base Class (ABC) for Deformable Objects
class DeformableObject(ABC):
    @abstractmethod
    def load(self):
        """
        Abstract method to be implemented by subclasses for loading deformable object into the simulation.
        """
        pass

    @abstractmethod
    def create_anchors(self):
        """
        Abstract method to be implemented by subclasses for creating anchors on the deformable object.
        """
        pass


# Class for simulating a Deformable Cloth object
class DeformableCloth(DeformableObject):

    def __init__(self, spring_elastic_stiffness):
        """
        Initializes a DeformableCloth object.

        Args:
        - spring_elastic_stiffness (float): Stiffness of the spring connections in the cloth.

        Attributes:
        - file_path (str): Path to the cloth model file.
        - base_position (list): Initial position of the cloth object in the simulation.
        - base_orientation (np.array): Initial orientation of the cloth object, converted to quaternion.
        - scale (float): Scale factor for the cloth object.
        - mass (float): Mass of the cloth object. Should be at least 1kg.
        - spring_elastic_stiffness (float): Stiffness of the spring connections in the cloth.
        - cloth_id (int): ID of the cloth object once loaded into the simulation, initialized to None.
        """
        def_path = os.path.join(
            os.path.split(__file__)[0],
            "models",
            "deformables",
        )
        self.file_path = os.path.join(def_path, "button_cloth.obj")
        self.base_position = [0.8, 0.21, 0.5]
        self.base_orientation = euler2quat(np.array([np.pi / 2, 0, 0]))
        self.scale = 0.25
        self.mass = 1
        self.spring_elastic_stiffness = spring_elastic_stiffness
        self.cloth_id = None

    def load(self):
        """
        Loads the cloth object into the simulation using the pybullet loadSoftBody function.
        Sets the cloth_id attribute to the ID returned by pybullet.
        """
        self.cloth_id = p.loadSoftBody(
            self.file_path,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
            scale=self.scale,
            mass=self.mass,
            useBendingSprings=1,
            useMassSpring=1,
            springElasticStiffness=self.spring_elastic_stiffness,
            useSelfCollision=0,
            frictionCoeff=0.5,
            useFaceContact=1,
            springDampingStiffness=1,
            springDampingAllDirections=1,
            useNeoHookean=True,
            NeoHookeanMu=1,
            NeoHookeanLambda=2,
            NeoHookeanDamping=1,
        )

    def create_anchors(self, positions):
        """
        Creates anchors on the cloth at specified positions.

        Args:
        - positions (list): List of positions where anchors will be created.

        Returns:
        - anchors (list): List of anchor IDs created on the cloth.
        """
        _, mesh = get_mesh_data(p, self.cloth_id)
        anchors = []
        for pos in positions:
            new_anc_pos, anchor_vertices = get_closest(pos, mesh)
            anchor_id = create_anchor_geom(p, new_anc_pos, mass=0, radius=0.01, rgba=(1, 0, 1, 1.0))
            attach_anchor(p, anchor_id, anchor_vertices, self.cloth_id)
            anchors.append(anchor_id)
        return anchors