from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage

from isaacsim.core.prims import SingleArticulation, Articulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation import interface_config_loader
from isaacsim.core.utils.rotations import lookat_to_quatf


import numpy as np

world = World()
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()

franka_dir = assets_root + "/Isaac/Robots/Franka"

# add robot
usd_path = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
prim_path = "/World/envs/env_0/panda"
add_reference_to_stage(usd_path, prim_path)

# add articulation controller
robot_articulation = SingleArticulation(prim_path=prim_path, name="franka_panda")

# reset world
world.reset()

# setup inverse kinematics
config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
urdf_path = config["urdf_path"]
config_path = config["robot_description_path"]
ee_name = config["end_effector_frame_name"]
kinematics_solver = LulaKinematicsSolver(
    robot_description_path=config_path,
    urdf_path=urdf_path
)

# Initialize the helper wrapper - used to convert directly to articulation actions
ik_solver = ArticulationKinematicsSolver(
    robot_articulation=robot_articulation,
    kinematics_solver=kinematics_solver,
    end_effector_frame_name=ee_name
)

# # initialize controller
robot_articulation.initialize()

circle_radius = 0.2
circle_center = np.array([0.6, 0, 0.5])
target_center = np.array([0.6, 0, 0])
up_axis = np.array([0.0, 0.0, 1.0])

target_positions = []
target_orientations = []
from pxr import Gf
for theta in [0, np.pi/2, np.pi, 3*np.pi/2]:

    pos = circle_center + np.array([circle_radius * np.cos(theta), circle_radius * np.sin(theta), 0])
    quat_gf = lookat_to_quatf(target=Gf.Vec3f(target_center.tolist()), camera=Gf.Vec3f(pos.tolist()), up=Gf.Vec3f(up_axis.tolist()))
    # quat = np.array([quat_gf.real, quat_gf.imag[0], quat_gf.imag[1], quat_gf.imag[2]])
    quat = np.array([quat_gf.real, *quat_gf.imaginary])
    target_positions.append(pos)
    target_orientations.append(quat)
# target_positions = [circle_center]
# target_ori = np.array([0.5, 0.5, 0.5, 0.5]) # Example quaternion

while simulation_app.is_running():
    for _ in range(120): # At 60Hz, 120 steps is 2 seconds
        world.step(render=True)
    for position_number in range(len(target_positions)):
        print(f"Moving to position {position_number}")
        target_pos = target_positions[position_number]
        target_ori = target_orientations[position_number]
        action, success = ik_solver.compute_inverse_kinematics(target_pos, target_ori)
        # print(f"target position #{position_number}: {target_pos}")
        if not success:
            print("IK did not converge to a solution.  No action is being taken")
            continue

        dist = np.inf
        threshold = 0.01
        while dist > threshold:
            
            robot_articulation.apply_action(action)
            world.step(render=True)
            current_joint_pos = robot_articulation.get_joint_positions()
            dist = np.linalg.norm(current_joint_pos[:7] - action.joint_positions)
            print(dist)

    # world.pause()

simulation_app.close()

# # Get current joint positions
# current_positions = robot_articulation.get_joint_positions()
# print(f"Current joint positions: {current_positions}")

# # Create target positions
# target_positions = np.array([0.0, -1.5, 0.0, -2.8, 0.0, 2.8, 1.2, 0.04, 0.04])
# # target_positions = joint_positions

# all_positions = np.vstack([target_positions, current_positions, target_positions, current_positions])

# # action = ArticulationAction(joint_positions=target_positions)

# while simulation_app.is_running():
#     for i in range(len(all_positions)):
#         action = ArticulationAction(joint_positions=all_positions[i])
#         dist = np.inf
#         threshold = 0.01
#         while dist > threshold:
#             robot_articulation.get_articulation_controller().apply_action(action)
#             world.step(render=True)
#             dist = np.linalg.norm(all_positions[i] - robot_articulation.get_joint_positions())
#     world.pause()
# simulation_app.close()


