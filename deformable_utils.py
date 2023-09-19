import os
import pybullet as p
import numpy as np

# Constants for the anchors and controllers
ANCHOR_MASS = 0.100  # 100g
ANCHOR_RADIUS = 0.05  # 5cm
ANCHOR_RGBA_ACTIVE = (1, 0, 1, 1)  # magenta
ANCHOR_RGBA_INACTIVE = (0.5, 0.5, 0.5, 1)  # gray
ANCHOR_RGBA_PEACH = (0.9, 0.75, 0.65, 1)  # peach
CTRL_MAX_FORCE = 10  # 10
CTRL_PD_KD = 50.0  # 50


def get_closest(init_pos, mesh, max_dist=None):
    """Find mesh points closest to the given point."""
    init_pos = np.array(init_pos).reshape(1, -1)
    mesh = np.array(mesh)
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


def get_mesh_data(sim, deform_id): # use this method for d objects
    """Returns num mesh vertices and vertex positions."""
    kwargs = {}
    if hasattr(p, "MESH_DATA_SIMULATION_MESH"):
        kwargs["flags"] = p.MESH_DATA_SIMULATION_MESH
    num_verts, mesh_vert_positions = sim.getMeshData(deform_id, **kwargs)
    # print(mesh_vert_positions)
    return num_verts, mesh_vert_positions


def get_mesh_obj_vertices(obj_id):  # use this method for rigid objects
    mesh_stick = p.getMeshData(obj_id, -1)
    pos, orn = p.getBasePositionAndOrientation(obj_id)
    vertices = [p.multiplyTransforms(pos, orn, v, orn)[0] for v in mesh_stick[1]]
    return vertices


def add_stick_to_robot(replica_robot, stick_type='long'):
    # Define paths based on stick type
    assert stick_type in ['default', 'convex', 'long']
    obj_path = os.path.join(os.path.split(__file__)[0], "models", "objects")
    # the problem right now is that the start position of the robot is defined in create_samples.py, but
    # the stick form is not. therefore the startpostion and stick form have to be matched by hand.
    # future action:
    # give stick type as parameter already created in create_samples.py and
    # feed the parameter through the functions to set the robot start position (particularly the y-axis param)
    stick_paths = {
        "convex": {
            "urdf": os.path.join(obj_path, "sticky.urdf"),
            "obj": os.path.join(obj_path, "sticky.obj"),
            "meshScale": [0.05, 0.05, 0.2]
        },
        "long": {
            "urdf": os.path.join(obj_path, "stick.urdf"),
            "obj": os.path.join(obj_path, "stick.obj"),
            "meshScale": [0.05, 0.05, 0.4]
        },
        "default": {
            "urdf": os.path.join(obj_path, "stick.urdf"),
            "obj": os.path.join(obj_path, "stick.obj"),
            "meshScale": [0.05, 0.05, 0.2]
        }
    }
    stick_data = stick_paths[stick_type]

    # Get end effector link index
    def get_end_effector_link_index(robot_id):
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            if joint_name == "panda_grasptarget_hand":
                return joint_info[0]
        return None

    ee_link_index = get_end_effector_link_index(replica_robot.robot_id)
    link_state = p.getLinkState(replica_robot.robot_id, ee_link_index)
    link_world_position = link_state[0]

    # Create collision shape for the stick
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=stick_data['obj'],
        meshScale=stick_data['meshScale'],
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

    return stick_id, constraint_id, stick_data['obj']
