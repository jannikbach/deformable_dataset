import os
from abc import ABC, abstractmethod
import pybullet as p


# Abstract Base Class for Robot Objects
class RobotObject(ABC):

    @abstractmethod
    def load(self):
        """
        Abstract method to be implemented by subclasses for loading the robot object into the simulation.
        """
        pass

    @abstractmethod
    def update_y(self, y):
        """
        Abstract method to be implemented by subclasses for updating the y-coordinate of the robot object.

        Args:
        - y (float): The y-coordinate value.
        """
        pass

# Class for simulating a Standard Stick object attached to a robot
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
        self.stick_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=link_world_position,
        )

        # Create a constraint to attach the stick to the end effector
        stick_pos = [0, 0, 0]
        stick_orient = p.getQuaternionFromEuler([0, 0, 0])
        self.constraint_id = p.createConstraint(
            self.robot.robot_id,
            ee_link_index,
            self.stick_id,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=stick_pos,
            childFrameOrientation=stick_orient,
        )

        return self.stick_id, self.constraint_id, self.stick_obj

    def update_y(self, y):
        """
        Overrides the abstract method from RobotObject.
        In this implementation, the method simply returns the input y-coordinate value without modification.

        Args:
        - y (float): The y-coordinate value.

        Returns:
        - float: The unmodified y-coordinate value.
        """
        return y


# Class for simulating a Long Stick object attached to a robot
class LongStick(RobotObject):

    def __init__(self, robot):
        self.robot = robot
        self.stick_id = None
        self.stick_constraint = None
        self.stick_path_obj = os.path.join(os.path.split(__file__)[0], "models", "objects")
        self.stick_urdf = os.path.join(self.stick_path_obj, "stick.urdf")
        self.stick_obj = os.path.join(self.stick_path_obj, "stick.obj")
        self.mesh_scale = [0.05, 0.05, 0.4]

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
        self.stick_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=link_world_position,
        )

        # Create a constraint to attach the stick to the end effector
        stick_pos = [0, 0, 0]
        stick_orient = p.getQuaternionFromEuler([0, 0, 0])
        self.constraint_id = p.createConstraint(
            self.robot.robot_id,
            ee_link_index,
            self.stick_id,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=stick_pos,
            childFrameOrientation=stick_orient,
        )

        return self.stick_id, self.constraint_id, self.stick_obj

    def update_y(self, y):
        """
        Overrides the abstract method from RobotObject.
        In this implementation, the method reduces the y values by 0.2, because of the increased length of the long stick.

        Args:
        - y (float): The y-coordinate value.

        Returns:
        - float: The unmodified y-coordinate value.
        """
        y -= 0.2
        return y


# Class for simulating a Convex Stick object attached to a robot
class ConvexStick(RobotObject):

    def __init__(self, robot):
        self.robot = robot
        self.stick_id = None
        self.stick_constraint = None
        self.stick_path_obj = os.path.join(os.path.split(__file__)[0], "models", "objects")
        self.stick_urdf = os.path.join(self.stick_path_obj, "sticky.urdf")
        self.stick_obj = os.path.join(self.stick_path_obj, "sticky.obj")
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
        self.stick_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=link_world_position,
        )

        # Create a constraint to attach the stick to the end effector
        stick_pos = [0, 0, 0]
        stick_orient = p.getQuaternionFromEuler([0, 0, 0])
        self.constraint_id = p.createConstraint(
            self.robot.robot_id,
            ee_link_index,
            self.stick_id,
            -1,
            p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=stick_pos,
            childFrameOrientation=stick_orient,
        )

        return self.stick_id, self.constraint_id, self.stick_obj

    def update_y(self, y):
        """
        Overrides the abstract method from RobotObject.
        In this implementation, the method simply returns the input y-coordinate value without modification.

        Args:
        - y (float): The y-coordinate value.

        Returns:
        - float: The unmodified y-coordinate value.
        """
        return y



