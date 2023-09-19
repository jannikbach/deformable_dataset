import os
import sys
import pybullet as p
import numpy as np

from Logger import PyBulletDeformableDatasetLogger
from alr_sim.sims import PyBulletRobot
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils import xyzw_to_wxyz, euler2quat
from deformable_utils import get_mesh_data, get_closest, create_anchor_geom, attach_anchor, add_stick_to_robot


def create_and_save_trajectory(
    pb_scene,
    replica_robot,
    time_step,
    deformable_id,
    stick_id,
    anchor_pos,
    deformable_obj_path,
    stick_obj_path,
    start_pos_end_effector=[0.4388102, 0.21632394, -0.3],
    spring_elastic_stiffness=100.0,
    spring_damping_stiffness=1,
    spring_damping_all_directions=1,
    use_neo_hookean=True,
    neo_hookean_mu=1,
    neo_hookean_lambda=2,
    neo_hookean_damping=1,
):

    replica_robot.beam_to_cart_pos_and_quat(
        start_pos_end_effector,
        [0.70610327, -0.70709914, -0.03779343, -0.00081645], # claculated by hand from euler2quat
    )

    start_pos, _, start_quat = replica_robot.get_x()
    print("actual pos start: ", start_pos)

    _, _, quat = replica_robot.get_x()
    quat = xyzw_to_wxyz(quat)

    # Adding a logger to extract and save deformable data
    logger = PyBulletDeformableDatasetLogger(
        scene=pb_scene,
        deformable_id=deformable_id,
        stick_id=stick_id,
        deformable_obj_path=deformable_obj_path,
        stick_obj_path=stick_obj_path,
        save_data=True,
        start_pos_stick_xz=[start_pos_end_effector[0], start_pos_end_effector[2]],
        anchor_pos=anchor_pos,
        spring_elastic_stiffness=spring_elastic_stiffness,
        spring_damping_stiffness=spring_damping_stiffness,
        spring_damping_all_directions=spring_damping_all_directions,
        use_neo_hookean=use_neo_hookean,
        neo_hookean_mu=neo_hookean_mu,
        neo_hookean_lambda=neo_hookean_lambda,
        neo_hookean_damping=neo_hookean_damping,
        final_time_stamp_count=100,
        min_y_for_logging=0.15,
        output_file_path="./data",
    )
    pb_scene.add_logger(logger=logger)

    pb_scene.start_logging(duration=10.0)
    replica_robot.gotoCartPositionAndQuat(
        [
            start_pos_end_effector[0],
            start_pos_end_effector[1] + 0.3,
            start_pos_end_effector[2],
        ],
        quat,
        duration=4.0,
    )
    pb_scene.stop_logging()

    (
        start_pos,
        _,
        start_quat,
    ) = replica_robot.get_x()
    print("pos end: ", start_pos)


def main(x, y, z, spring_constant):
    time_step = 0.001
    factory = SimRepository.get_factory("pybullet")
    rm = factory.RenderMode.OFFSCREEN
    obj_list = None

    pb_scene = factory.create_scene(object_list=obj_list, render=rm, dt=time_step)
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    replica_robot = PyBulletRobot(scene=pb_scene)

    pb_scene.start()

    def_path = os.path.join(
        os.path.split(__file__)[0],
        "models",
        "deformables",
    )

    def_path_obj = os.path.join(def_path, "button_cloth.obj")

    # Start Scenes and Sync Positions
    # rotate by 90 degree around x-axis (euler)
    quat = euler2quat(
        np.array(
            [
                np.pi / 2,
                0,
                0,
            ]
        )
    )
    print(def_path_obj)
    cloth_id = p.loadSoftBody(
        def_path_obj,
        basePosition=[0.8, 0.21, 0.5],
        baseOrientation=quat,  # wxyz
        scale=0.25,
        mass=1,
        useBendingSprings=1,
        useMassSpring=1,
        springElasticStiffness=spring_constant,
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

    ### Create anchor objects
    _, mesh = get_mesh_data(p, cloth_id)
    new_anc_pos1, anchor_vertices1 = get_closest([-0.5, 0, 1], mesh)
    new_anc_pos2, anchor_vertices2 = get_closest([1, 0, 1], mesh)

    anchor_id1 = create_anchor_geom(
        p,
        new_anc_pos1,
        mass=0,
        radius=0.01,
        rgba=(1, 0, 1, 1.0)
    )
    anchor_id2 = create_anchor_geom(
        p,
        new_anc_pos2,
        mass=0,
        radius=0.01,
        rgba=(1, 0, 1, 1.0)
    )

    # Attach anchor object to deformable
    attach_anchor(p, anchor_id1, anchor_vertices1, cloth_id)
    attach_anchor(p, anchor_id2, anchor_vertices2, cloth_id)

    stick_id, stick_constraint, stick_path_obj = add_stick_to_robot(replica_robot)

    print("stiffness: ", str(spring_constant), ", coordinates: ", str([x, y, z]))
    create_and_save_trajectory(
        pb_scene=pb_scene,
        replica_robot=replica_robot,
        time_step=time_step,
        spring_elastic_stiffness=spring_constant,
        deformable_id=cloth_id,
        deformable_obj_path=def_path_obj,
        stick_obj_path=stick_path_obj,
        stick_id=stick_id,
        anchor_pos=[new_anc_pos1, new_anc_pos2],
        start_pos_end_effector=[x, y, z],
    )
    # Shutdown Procedure
    print('Trajectory finished.')


if __name__ == "__main__":
    float_args = [float(arg) for arg in sys.argv[1:]]
    main(x=float_args[0], y=float_args[1], z=float_args[2], spring_constant=float_args[3])
