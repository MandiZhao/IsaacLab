# license?

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Test inspire hand in Isaac Sim.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
parser.add_argument("--dof_choices", type=str, nargs="+", help="List of degree of freedom choices.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import os 
import torch
from copy import deepcopy
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

def write_hand_with_wrist_cfg(
    wrist_dof_choices,
    modified_urdf_name,  
    base_hand_cfg=INSPIRE_HAND_LEFT_CFG, 
    is_left_hand=True
):
    new_hand_cfg = deepcopy(base_hand_cfg)
    if wrist_dof_choices is None:
        return new_hand_cfg
    # add new degree of freedom to the URDF file
    input_fname = str(new_hand_cfg.spawn.asset_path)
    output_name = modified_urdf_name
    output_fname = os.path.join(os.path.dirname(input_fname), output_name)
    if os.path.exists(output_fname):
        print(f"WARNING - File: {output_fname} already exists. Overwriting...") 

    dof_choices = wrist_dof_choices
    base_link_name = "base_link"
    new_joint_names = add_forearm_dof(input_fname, output_fname, dof_choices, base_link_name, left_hand=is_left_hand)

    # update spawn asset path
    new_hand_cfg.spawn.asset_path = output_fname
    # update actuators
    wrist_actuators = ImplicitActuatorCfg(
        joint_names_expr=new_joint_names,
        effort_limit={joint: 1.0 for joint in new_joint_names},
        stiffness={joint: 1.0 for joint in new_joint_names},
        damping={joint: 5000 for joint in new_joint_names},
    )
    new_hand_cfg.actuators["wrists"] = wrist_actuators
    return new_hand_cfg


@configclass
class InspireTwoHandsSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # # inspire hand 
    # inspire_left: ArticulationCfg = INSPIRE_HAND_LEFT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandLeft")
    # # inspire_right: ArticulationCfg = INSPIRE_HAND_RIGHT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandRight")

    # object
    # task_object = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/TaskObject",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="/home/mandi/IsaacLab/source/standalone/chiral/waffleiron.usd"
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(),
    # )
    replicate_physics = False # due to mimic joints!

    def __init__(self, *args, wrist_dof_choices, new_urdf_fname, **kwargs):
        super().__init__(*args, **kwargs)
        left_hand_cfg = write_hand_with_wrist_cfg(
            wrist_dof_choices, new_urdf_fname,
            base_hand_cfg=INSPIRE_HAND_LEFT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandLeft"), 
            is_left_hand=True)
        # right_hand_cfg = write_hand_with_wrist_cfg(
        #     wrist_dof_choices, new_urdf_fname,
        #     base_hand_cfg=INSPIRE_HAND_RIGHT_CFG.replace(prim_path="{ENV_REGEX_NS}/HandRight"),
        #     is_left_hand=False)
        self.inspire_left = left_hand_cfg
        # self.inspire_right = right_hand_cfg
        # ground plane
        self.ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

        # lights
        self.dome_light = AssetBaseCfg(
            prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        )
        self.replicate_physics = False # due to mimic joints!

@torch.jit.script
def scale(x, lower, upper):
    return 0.5 * (x + 1.0) * (upper - lower) + lower

def sample_actuated_joints(robot, actuator_key="fingers", random_delta=0.1):
    """
    Sample a small delta centering the current joint states
    """
    actuated_indices = robot.actuators[actuator_key].joint_indices # list of int 
    actuated_names = robot.actuators[actuator_key].joint_names # list of str
    # get idx for joint "L_FF_J1"
    idxs = []
    for i, name in enumerate(actuated_names):
        if "F_J" in name:
            idxs.append(i) 
    
    dof_limits = robot.root_physx_view.get_dof_limits() # shape [num_envs, num_total_joints, 2]
    actuated_limits = dof_limits[:, idxs] # shape [num_envs, num_actuated_joints, 2]
    
    # sample a random delta 
    current_joint_pos = robot._data.joint_pos[:, idxs]  
    rand_actions = torch.randn_like(current_joint_pos) * random_delta
    new_joint_pos = current_joint_pos + rand_actions
    clamped_pos = torch.clamp(new_joint_pos, actuated_limits[:, :, 0], actuated_limits[:, :, 1])
    print(actuated_names, idxs, current_joint_pos, new_joint_pos, clamped_pos)
    # set joint positions
    robot.set_joint_position_target(clamped_pos, idxs)
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
        if count %  500 == 0: 
            # reset counter
            # count = 0
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
            print(f"Step count: {count}")
        # if count == 2000:
        #     sample_actuated_joints(left_hand)
        #     breakpoint()
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
    scene_cfg = InspireTwoHandsSceneCfg(
        num_envs=args_cli.num_envs, env_spacing=1,
        wrist_dof_choices=args_cli.dof_choices, 
        new_urdf_fname="new_wrist.urdf",
        )
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
