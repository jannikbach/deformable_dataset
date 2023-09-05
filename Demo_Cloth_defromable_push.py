import math
import os
import sys
import time

import numpy as np
import pybullet as p

from Logger import PyBulletDeformableDatasetLogger
from alr_sim.sims import PyBulletRobot
from alr_sim.utils import xyzw_to_wxyz, wxyz_to_xyzw, euler2quat

from alr_sim.sims.SimFactory import SimRepository



ANCHOR_MIN_DIST = 0.02  # 2cm
ANCHOR_MASS = 0.100  # 100g
ANCHOR_RADIUS = 0.05  # 5cm
ANCHOR_RGBA_ACTIVE = (1, 0, 1, 1)  # magenta
ANCHOR_RGBA_INACTIVE = (0.5, 0.5, 0.5, 1)  # gray
ANCHOR_RGBA_PEACH = (0.9, 0.75, 0.65, 1)  # peach
# Gains and limits for a simple controller for the anchors.
CTRL_MAX_FORCE = 10  # 10
CTRL_PD_KD = 50.0  # 50
MAX_ACT_VEL = 10.0


def get_closest(init_pos, mesh, max_dist=None):
    """Find mesh points closest to the given point."""
    init_pos = np.array(init_pos).reshape(1, -1)
    mesh = np.array(mesh)
    # num_pins_per_pt = max(1, mesh.shape[0] // 50)
    # num_to_pin = min(mesh.shape[0], num_pins_per_pt)
    num_to_pin = 1  # new pybullet behaves well with 1 vertex per anchor
    dists = np.linalg.norm(mesh - init_pos, axis=1)
    anchor_vertices = np.argpartition(dists, num_to_pin)[0:num_to_pin]
    if max_dist is not None:
        anchor_vertices = anchor_vertices[dists[anchor_vertices] <= max_dist]
    new_anc_pos = mesh[anchor_vertices].mean(axis=0)
    return new_anc_pos, anchor_vertices


def create_anchor_geom(
    sim,
    pos,
    mass=ANCHOR_MASS,
    radius=ANCHOR_RADIUS,
    rgba=ANCHOR_RGBA_INACTIVE,
    use_collision=True,
):
    """Create a small visual object at the provided pos in world coordinates.
    If mass==0: the anchor will be fixed (not moving)
    If use_collision==False: this object does not collide with any other objects
    and would only serve to show grip location.
    input: sim (pybullet sim), pos (list of 3 coords for anchor in world frame)
    output: anchorId (long) --> unique bullet ID to refer to the anchor object
    """
    anchor_visual_shape = sim.createVisualShape(
        p.GEOM_SPHERE, radius=radius, rgbaColor=rgba
    )
    if mass > 0 and use_collision:
        anchor_collision_shape = sim.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    else:
        anchor_collision_shape = -1
    anchor_id = sim.createMultiBody(
        baseMass=mass,
        basePosition=pos,
        baseCollisionShapeIndex=anchor_collision_shape,
        baseVisualShapeIndex=anchor_visual_shape,
        useMaximalCoordinates=True,
    )
    return anchor_id


def create_anchor(
    sim,
    anchor_pos,
    anchor_idx,
    preset_vertices,
    mesh,
    mass=0.1,
    radius=ANCHOR_RADIUS,
    rgba=(1, 0, 1, 1.0),
    use_preset=True,
    use_closest=True,
):
    """
    Create an anchor in Pybullet to grab or pin an object.
    :param sim: The simulator object
    :param anchor_pos: initial anchor position
    :param anchor_idx: index of the anchor (0:left, 1:right ...)
    :param preset_vertices: a preset list of vertices for the anchors
                            to grab on to (if use_preset is enabled)
    :param mesh: mesh of the deform object
    :param mass: mass of the anchor
    :param radius: visual radius of the anchor object
    :param rgba: color of the anchor
    :param use_preset: Use preset of anchor vertices
    :param use_closest: Use closest vertices to anchor as grabbing vertices
           (if no preset is used), ensuring anchors
    has something to grab on to
    :return: Anchor's ID, anchor's position, anchor's vertices
    """
    anchor_vertices = None
    mesh = np.array(mesh)
    if use_preset and preset_vertices is not None:
        anchor_vertices = preset_vertices[anchor_idx]
        anchor_pos = mesh[anchor_vertices].mean(axis=0)
    elif use_closest:
        anchor_pos, anchor_vertices = get_closest(anchor_pos, mesh)
    anchor_geom_id = create_anchor_geom(sim, anchor_pos, mass, radius, rgba)
    return anchor_geom_id, anchor_pos, anchor_vertices


def command_anchor_velocity(sim, anchor_bullet_id, tgt_vel):
    anc_linvel, _ = sim.getBaseVelocity(anchor_bullet_id)
    vel_diff = tgt_vel - np.array(anc_linvel)
    raw_force = CTRL_PD_KD * vel_diff
    force = np.clip(raw_force, -1.0 * CTRL_MAX_FORCE, CTRL_MAX_FORCE)
    sim.applyExternalForce(
        anchor_bullet_id, -1, force.tolist(), [0, 0, 0], p.LINK_FRAME
    )
    return raw_force


def attach_anchor(sim, anchor_id, anchor_vertices, deform_id, change_color=False):
    if change_color:
        sim.changeVisualShape(anchor_id, -1, rgbaColor=ANCHOR_RGBA_ACTIVE)
    for v in anchor_vertices:
        sim.createSoftBodyAnchor(deform_id, v, anchor_id, -1)


def release_anchor(sim, anchor_id):
    sim.removeConstraint(anchor_id)
    sim.changeVisualShape(anchor_id, -1, rgbaColor=ANCHOR_RGBA_INACTIVE)


def change_anchor_color_gray(sim, anchor_id):
    sim.changeVisualShape(anchor_id, -1, rgbaColor=ANCHOR_RGBA_INACTIVE)


def pin_fixed(sim, deform_id, vert_ids):
    _, v_pos_list = get_mesh_data(sim, deform_id)
    for v_idx in vert_ids:
        v_pos = v_pos_list[v_idx]
        anc_id = create_anchor_geom(
            sim, v_pos, mass=0.0, radius=0.002, rgba=ANCHOR_RGBA_PEACH
        )
        sim.createSoftBodyAnchor(deform_id, v_idx, anc_id, -1)


def get_mesh_data(sim, deform_id):
    """Returns num mesh vertices and vertex positions."""
    kwargs = {}
    if hasattr(p, "MESH_DATA_SIMULATION_MESH"):
        kwargs["flags"] = p.MESH_DATA_SIMULATION_MESH
    num_verts, mesh_vert_positions = sim.getMeshData(deform_id, **kwargs)
    # print(mesh_vert_positions)
    return num_verts, mesh_vert_positions


def get_mesh_obj_vertices(obj_id):
    obj_mesh = p.getMeshData(obj_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    pos, orn = p.getBasePositionAndOrientation(obj_id)
    return [p.multiplyTransforms(pos, orn, v, orn) for v in obj_mesh[1]]


def add_stick_to_robot(replica_robot):
    obj_path = os.path.join(
        os.path.split(__file__)[0],
        "models",
        "objects",
    )

    stick_type = "long"
    if stick_type == "convex":
        path = os.path.join(obj_path, "sticky.urdf")
        stick_path_obj = os.path.join(obj_path, "sticky.obj")
    if stick_type == "long":
        path = os.path.join(obj_path, "stick.urdf")
        stick_path_obj = os.path.join(obj_path, "stick.obj")

    print(stick_path_obj)

    # Attach the stick to the end-effector link
    num_joints = p.getNumJoints(replica_robot.robot_id)
    ee_link_index = None
    for i in range(num_joints):
        joint_info = p.getJointInfo(replica_robot.robot_id, i)
        joint_name = joint_info[1].decode("utf-8")
        if joint_name == "panda_grasptarget_hand":
            ee_link_index = joint_info[0]
            break
    link_state = p.getLinkState(replica_robot.robot_id, ee_link_index)
    link_world_position = link_state[0]
    # print("world pos of joint: ", link_world_position)

    # Load your mesh as a concave shape
    if stick_type == "convex":
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=stick_path_obj,
            meshScale=[0.05, 0.05, 0.2],
            flags=p.GEOM_CONCAVE_INTERNAL_EDGE,
        )
    if stick_type == "long":
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=stick_path_obj,
            meshScale=[0.05, 0.05, 0.4],
            flags=p.GEOM_CONCAVE_INTERNAL_EDGE,
        )

    # Create a multibody with the collision shape
    stick_id = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collisionShapeId,
        basePosition=link_world_position,
    )

    stick_pos = [0, 0, 0]
    stick_orient = p.getQuaternionFromEuler([0, 0, 0])

    constraint_id = p.createConstraint(
        replica_robot.robot_id,
        ee_link_index,
        stick_id,
        -1,
        p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=stick_pos,
        childFrameOrientation=stick_orient,
    )

    # beam to cart position
    # replica_robot.beam_to_cart_pos_and_quat([0.45, 0, 0.55], [0, 1, 0, 0])
    # j, _, _ = replica_robot.get_qdq_J()
    # joint_number = 4
    # print(j[joint_number])
    # j[joint_number] = math.pi / 2
    # replica_robot.beam_to_joint_pos(j)
    return stick_id, constraint_id, stick_path_obj


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
    # print('start: ', start_pos_end_effector)
    # try to give pos and quat
    replica_robot.beam_to_cart_pos_and_quat(
        start_pos_end_effector,
        [0.70610327, -0.70709914, -0.03779343, -0.00081645],
    )

    # # wait for 10s
    # for _ in range(int(10 * (1 / time_step))):  # 10s
    #     pb_scene.next_step()

    (
        start_pos,
        _,
        start_quat,
    ) = replica_robot.get_x()
    print("actual pos start: ", start_pos)
    # print("quat start: ", xyzw_to_wxyz(start_quat))
    # [0.4388102  0.21632394 0.76646638], start position
    # [-0.70709914 -0.03779343 -0.00081645  0.70610327], start quaternion

    _, _, quat = replica_robot.get_x()
    quat = xyzw_to_wxyz(quat)

    mesh_stick = p.getMeshData(stick_id, -1)
    pos, orn = p.getBasePositionAndOrientation(stick_id)
    vertices = [p.multiplyTransforms(pos, orn, v, orn)[0] for v in mesh_stick[1]]
    print("stick mesh shape: ", len(vertices))

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
    # script schreiben, dass 10 trajectories mit 10 stiffnes params macht
    # parallel ausfuehren, damit schneller.
    # dateiname mit random number, damit nicht gleicher file name wenn parallel
    # bash skript mit & damit alles gleichzeitig laeuft

    # 3 verschiedene stiffnesses: 100,500,1000 mit jeweils 300 trajectorien (mit untercheidlichen positionen)

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

    # quat =euler2quat(np.array([np.pi/2, np.pi/2, 0]))
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
        # p, [0.5, 0.5, 1], mass = 0, radius = 0.01, rgba = (1, 0, 1, 1.0)
    )
    anchor_id2 = create_anchor_geom(
        p,
        new_anc_pos2,
        mass=0,
        radius=0.01,
        rgba=(1, 0, 1, 1.0)
        # p, [0.3, 0.5, 1], mass = 0, radius = 0.01, rgba = (1, 0, 1, 1.0)
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
    print("Goodbye")


if __name__ == "__main__":
    print(sys.argv[1:])
    float_args = [float(arg) for arg in sys.argv[1:]]
    print(float_args)
    main(float_args[0], float_args[1], float_args[2], float_args[3])
