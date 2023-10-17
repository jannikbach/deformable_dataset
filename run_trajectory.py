import pybullet as p
import sys

from DeformableObject import DeformableCloth
from RobotObject import Stick, LongStick, ConvexStick
from Logger import PyBulletDeformableDatasetLogger
from alr_sim.sims import PyBulletRobot
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils import xyzw_to_wxyz, euler2quat


class Trajectory:

    def __init__(self, pb_scene, replica_robot, time_step, cloth, stick, start_pos_end_effector, anchor_pos):
        self.pb_scene = pb_scene
        self.replica_robot = replica_robot
        self.time_step = time_step
        self.cloth = cloth
        self.stick = stick
        self.start_pos_end_effector = start_pos_end_effector
        self.anchor_pos = anchor_pos
        self.spring_elastic_stiffness = 100.0,
        self.spring_damping_stiffness = 1,
        self.spring_damping_all_directions = 1,
        self.use_neo_hookean = True,
        self.neo_hookean_mu = 1,
        self.neo_hookean_lambda = 2,
        self.neo_hookean_damping = 1,

    def create_and_save(self, **kwargs):
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
            stick_obj_path=self.stick.stick_obj,
            save_data=True,
            start_pos_stick_xz=[self.start_pos_end_effector[0], self.start_pos_end_effector[2]],
            anchor_pos=self.anchor_pos,
            spring_elastic_stiffness=self.spring_elastic_stiffness,
            spring_damping_stiffness=self.spring_damping_stiffness,
            spring_damping_all_directions=self.spring_damping_all_directions,
            use_neo_hookean=self.use_neo_hookean,
            neo_hookean_mu=self.neo_hookean_mu,
            neo_hookean_lambda=self.neo_hookean_lambda,
            neo_hookean_damping=self.neo_hookean_damping,
            final_time_stamp_count=100,
            min_y_for_logging=0.15,
            output_file_path="./data",
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
    y = stick.update_y(y)  # we have to update the base position if the stick is longer

    trajectory = Trajectory(pb_scene, replica_robot, time_step, cloth, stick, [x, y, z], anchors)
    trajectory.create_and_save(spring_elastic_stiffness=spring_constant)

    print('Trajectory finished.')


if __name__ == "__main__":
    float_args = [float(arg) for arg in sys.argv[1:]]
    main(x=float_args[0], y=float_args[1], z=float_args[2], spring_constant=float_args[3])
