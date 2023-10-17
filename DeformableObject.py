import os
from abc import ABC, abstractmethod

import numpy as np
import pybullet as p

from alr_sim.utils import euler2quat
from deformable_utils import get_mesh_data, get_closest, create_anchor_geom, attach_anchor


class DeformableObject(ABC):
    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def create_anchors(self):
        pass


class DeformableCloth(DeformableObject):

    def __init__(self, spring_elastic_stiffness):
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
        _, mesh = get_mesh_data(p, self.cloth_id)
        anchors = []
        for pos in positions:
            new_anc_pos, anchor_vertices = get_closest(pos, mesh)
            anchor_id = create_anchor_geom(p, new_anc_pos, mass=0, radius=0.01, rgba=(1, 0, 1, 1.0))
            attach_anchor(p, anchor_id, anchor_vertices, self.cloth_id)
            anchors.append(anchor_id)
        return anchors

