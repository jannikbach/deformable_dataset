import os
import pybullet as p
import sys
import numpy as np
from abc import ABC, abstractmethod
from Logger import PyBulletDeformableDatasetLogger
from alr_sim.sims import PyBulletRobot
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils import xyzw_to_wxyz, euler2quat
from deformable_utils import get_mesh_data, get_closest, create_anchor_geom, attach_anchor, add_stick_to_robot


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


class RobotObject(ABC):

    @abstractmethod
    def load(self):
        pass


class Stick(RobotObject):

    def __init__(self, robot):
        self.robot = robot
        self.stick_id = None
        self.stick_constraint = None
        self.stick_path_obj = os.path.join(os.path.split(__file__)[0], "models", "objects")
        self.stick_urdf = os.path.join(self.stick_path_obj, "stick.urdf")
        self.stick_obj = os.path.join(self.stick_path_obj, "stick.obj")
        self.mesh_scale = [0.05, 0.05, 0.2]

    def load(self):

        # Get end effector link index
        def get_end_effector_link_index(robot_id):
            num_joints = p.getNumJoints(robot_id)
            for i in range(num_joints):
                joint_info = p.getJointInfo(robot_id, i)
                joint_name = joint_info[1].decode("utf-8")
                if joint_name == "panda_grasptarget_hand":
                    return joint_info[0]
            return None

        ee_link_index = get_end_effector_link_index(self.robot.robot_id)
        link_state = p.getLinkState(self.robot.robot_id, ee_link_index)
        link_world_position = link_state[0]

        # Create collision shape for the stick
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=self.stick_obj,
            meshScale=self.mesh_scale,
            flags=p.GEOM_CONCAVE_INTERNAL_EDGE
        )

        # Create a multibody for the stick
        stick_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=link_world_position,
        )

        # Create a constraint to attach the stick to the end effector
        stick_pos = [0, 0, 0]
        stick_orient = p.getQuaternionFromEuler([0, 0, 0])
        constraint_id = p.createConstraint(
            self.robot.robot_id,
            ee_link_index,
            stick_id,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=stick_pos,
            childFrameOrientation=stick_orient,
        )

        return stick_id, constraint_id, self.stick_obj


class Trajectory:

    def __init__(self, pb_scene, replica_robot, time_step, cloth, stick, start_pos_end_effector, anchor_pos):
        self.pb_scene = pb_scene
        self.replica_robot = replica_robot
        self.time_step = time_step
        self.cloth = cloth
        self.stick = stick
        self.start_pos_end_effector = start_pos_end_effector
        self.anchor_pos = anchor_pos

    def create_and_save(self, spring_elastic_stiffness=100.0, **kwargs):
        self.replica_robot.beam_to_cart_pos_and_quat(
            self.start_pos_end_effector,
            [0.70610327, -0.70709914, -0.03779343, -0.00081645],  # calculated by hand from euler2quat
        )

        start_pos, _, _ = self.replica_robot.get_x()
        print("actual pos start: ", start_pos)

        _, _, quat = self.replica_robot.get_x()
        quat = xyzw_to_wxyz(quat)

        # Adding a logger to extract and save deformable data
        logger = PyBulletDeformableDatasetLogger(
            scene=self.pb_scene,
            deformable_id=self.cloth.cloth_id,
            stick_id=self.stick.stick_id,
            deformable_obj_path=self.cloth.file_path,
            stick_obj_path=self.stick.stick_path_obj,
            save_data=True,
            start_pos_stick_xz=[self.start_pos_end_effector[0], self.start_pos_end_effector[2]],
            anchor_pos=self.anchor_pos,
            spring_elastic_stiffness=spring_elastic_stiffness,
            **kwargs
        )
        self.pb_scene.add_logger(logger=logger)

        self.pb_scene.start_logging(duration=10.0)
        self.replica_robot.gotoCartPositionAndQuat(
            [
                self.start_pos_end_effector[0],
                self.start_pos_end_effector[1] + 0.3,
                self.start_pos_end_effector[2],
            ],
            quat,
            duration=4.0,
        )
        self.pb_scene.stop_logging()


def main(x, y, z, spring_constant):
    time_step = 0.001
    factory = SimRepository.get_factory("pybullet")
    rm = factory.RenderMode.OFFSCREEN
    obj_list = None

    pb_scene = factory.create_scene(object_list=obj_list, render=rm, dt=time_step)
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    replica_robot = PyBulletRobot(scene=pb_scene)

    pb_scene.start()

    cloth = DeformableCloth(spring_constant)
    cloth.load()
    anchor_positions = [[-0.5, 0, 1], [1, 0, 1]]
    anchors = cloth.create_anchors(anchor_positions)

    stick = Stick(replica_robot)
    stick.load()

    trajectory = Trajectory(pb_scene, replica_robot, time_step, cloth, stick, [x, y, z], anchors)
    trajectory.create_and_save(spring_elastic_stiffness=spring_constant)

    print('Trajectory finished.')


if __name__ == "__main__":
    float_args = [float(arg) for arg in sys.argv[1:]]
    main(x=float_args[0], y=float_args[1], z=float_args[2], spring_constant=float_args[3])
