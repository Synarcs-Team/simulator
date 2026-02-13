from isaacsim import SimulationApp

# instantiate the SimulationApp helper class
simulation_app = SimulationApp({"headless": True})

from isaacsim.robot_motion.motion_generation import interface_config_loader, LulaKinematicsSolver
from isaacsim.core.utils.rotations import lookat_to_quatf
# # Automatically load the config for Franka
# config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")

# # The config object contains the paths and parameters needed
# # You can inspect it to see where the files are coming from if needed
# print(config) 

# urdf_path = config["urdf_path"]
# config_path = config["robot_description_path"]
# ik_solver = LulaKinematicsSolver(
#     robot_description_path=config_path, 
#     urdf_path=urdf_path
# )