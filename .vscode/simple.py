from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False
    })

from isaacsim.core.api import World
from isaacsim.core.api.objects import *
from omni.isaac.sensor import Camera
from isaacsim.core.utils.bounds import compute_combined_aabb

from pxr import Sdf, UsdLux
import omni.usd

import numpy as np

# initialzie world - this connects to the physics engine
my_world = World(stage_units_in_meters=1.0)

# add light source
stage = omni.usd.get_context().get_stage()
distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/MainLight"))
distant_light.CreateIntensityAttr(1000)      # Brightness
distant_light.CreateColorAttr((1.0, 1.0, 1.0)) # White light (RGB)

# add ground plane
GroundPlane(prim_path="/World/GroundPlane", name="GroundPlane", z_position=0.0)

# Create a cube
big_scale = 0.5
big_cube = DynamicCuboid(
    prim_path="/World/BigCube", # Where it lives in the stage
    name="my_test_cube",     # A unique name for the API
    scale=(big_scale, big_scale, big_scale),    # Size (x,y,z)
    position=(0.0, 0.0, big_scale/2) # Initial position (x,y,z) sets the position of the centroid
)

# 3. Spawn the Small Cube exactly on that Z-height
small_scale = 0.2
small_cube = DynamicCuboid(
    prim_path="/World/SmallCube",
    name="small_cube",
    position=np.array([0, 0, big_scale + (small_scale / 2)]), 
    scale=np.array([small_scale, small_scale, small_scale]),
    color=np.array([255, 0, 0])
)

# create a camera looking down at the cube
my_cam = Camera(
    prim_path="/World/BigCube/Camera",
    name="top_down_cam",
    frequency=30,
    resolution=(640, 480),
    # Position it 1.5 meters above the cube's center
    translation=np.array([0, 0, 1.5]),
    # Rotate it -90 degrees around the Y-axis to look DOWN
    # Format is (w, x, y, z) for quaternions or Euler angles depending on version
    # Here we use orientation to point straight down
    orientation=np.array([0.70711, 0, 0.70711, 0]) 
)

# simulation loop
my_world.reset() # need this to get it started
my_cam.initialize()
while simulation_app.is_running():
    my_world.step(render=True)

# cleanup
simulation_app.close()