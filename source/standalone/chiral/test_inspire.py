# license?

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test inspire hand in Isaac Sim.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
# parser.add_argument("--replicate_physics", action="store_true", default=False, help="Replicate physics.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

##
# Pre-defined configs
##
from omni.isaac.lab_assets import INSPIRE_HAND_LEFT_CFG, INSPIRE_HAND_RIGHT_CFG  # isort:skip
from modify_urdf import add_forearm_dof

@configclass
class InspireTwoHandsSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # inspire hand 
    inspire_left: ArticulationCfg = INSPIRE_HAND_LEFT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandLeft")
    # inspire_right: ArticulationCfg = INSPIRE_HAND_RIGHT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandRight")

    # object
    # task_object = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/TaskObject",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="/home/mandi/IsaacLab/source/standalone/chiral/waffleiron.usd"
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(),
    # )
    replicate_physics = False

@torch.jit.script
def scale(x, lower, upper):
    return 0.5 * (x + 1.0) * (upper - lower) + lower

def sample_actuated_joints(robot, actuator_key="fingers"):
    actuated_indices = robot.actuators[actuator_key].joint_indices # list of int 
    actuated_names = robot.actuators[actuator_key].joint_names # list of str
    # get idx for joint "L_FF_J1"
    idxs = []
    for i, name in enumerate(actuated_names):
        if "F_J" in name:
            idxs.append(i) 

    dof_limits = robot.root_physx_view.get_dof_limits() # shape [num_envs, num_total_joints, 2]
    actuated_limits = dof_limits[:, idxs] # shape [num_envs, num_actuated_joints, 2]
    # actions are in the range [-1, 1], sample uniformly
    rand_actions = torch.rand(actuated_limits.shape[0], actuated_limits.shape[1])
    # set to max value
    # rand_actions = torch.ones_like(rand_actions)
    joint_pos = scale(rand_actions, actuated_limits[:, :, 0], actuated_limits[:, :, 1])

    # set joint positions
    robot.set_joint_position_target(joint_pos, idxs)
    return 

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    # robot = scene["inspire_left"]
    left_hand = scene["inspire_left"]
    robot = left_hand
    # right_hand = scene["inspire_right"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0: 
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            # root_state = left_hand.data.default_root_state.clone()
            # print(root_state.shape)
            # breakpoint()
            # root_state[:, :3] += scene.env_origins
            # robot.write_root_state_to_sim(root_state)
            # # set joint positions with some noise
            # joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            # joint_pos += torch.rand_like(joint_pos) * 0.1
            # robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            # scene.reset()
            # print("[INFO]: Resetting robot state...")
            # sample actuated joints
            print("Set new joint positions...   ")
        if count % 20 == 0:
            sample_actuated_joints(left_hand)
        # Apply random action
        # -- generate random joint efforts
        # efforts = torch.randn_like(left_hand.data.joint_pos) * 5.0
        # -- apply action to the robot
        # left_hand.set_joint_effort_target(efforts)
        # right_hand.set_joint_effort_target(efforts)
        # -- write data to sim
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device="cpu", use_gpu_pipeline=False)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    # Design scene
    scene_cfg = InspireTwoHandsSceneCfg(num_envs=args_cli.num_envs, env_spacing=1)
    scene = InteractiveScene(scene_cfg)
    print(scene) 
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
