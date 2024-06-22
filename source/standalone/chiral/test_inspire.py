# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to use the interactive scene interface to setup a scene with multiple prims.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/03_scene/create_scene.py --num_envs 32

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the interactive scene interface.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
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
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

##
# Pre-defined configs
##
from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip


@configclass
class CartpoleSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # inspire hand 
    inspire_left = ArticulationCfg(
        # prim_path="/home/avi/IsaacLab",
        prim_path="{ENV_REGEX_NS}/Hand",
        # spawn=sim_utils.UsdFileCfg(
        #     usd_path="/home/avi/IsaacLab/inspire_left.usd",
        spawn=sim_utils.UrdfFileCfg(
            asset_path="/home/avi/Downloads/inspire_hand_isaac/handleft01151/urdf/handleft01151.urdf",
            usd_dir="/home/avi/IsaacLab",
            fix_base=True,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                disable_gravity=True,
                retain_accelerations=True,
                max_depenetration_velocity=1000.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.0005,
            ),
            # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
            fixed_tendons_props=sim_utils.FixedTendonPropertiesCfg(limit_stiffness=30.0, damping=0.1),    
        ),

        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 2.0), 
            rot=(0.0, 0.0, -0.7071, 0.7071),
            joint_pos={".*": 0.0},
        ),
        actuators={
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=["joint011", "joint1112", "joint015", "joint017", "joint019", "joint021"],
                effort_limit={
                    "joint011": 1.30,
                    "joint1112": 0.68,
                    "joint015": 1.62,
                    "joint017": 1.62,
                    "joint019": 1.62,
                    "joint021": 1.62,    
                },
                stiffness={
                    "joint011": 1.0,
                    "joint1112": 1.0,
                    "joint015": 1.0,
                    "joint017": 1.0,
                    "joint019": 1.0,
                    "joint021": 1.0,
                },
                damping={
                    "joint011": 0.1,
                    "joint1112": 0.1,
                    "joint015": 0.1,
                    "joint017": 0.1,
                    "joint019": 0.1,
                    "joint021": 0.1,
                },
            )
        },
        soft_joint_pos_limit_factor=1.0,
    )
    
    # inspire_right = ArticulationCfg(
    #     prim_path="/home/avi/IsaacLab", 
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path="/home/avi/IsaacLab/inspire_right.usd",
    #         activate_contact_sensors=False,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             disable_gravity=True,
    #             retain_accelerations=True,
    #             max_depenetration_velocity=1000.0,
    #         ),
    #         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    #             enabled_self_collisions=True,
    #             solver_position_iteration_count=8,
    #             solver_velocity_iteration_count=0,
    #             sleep_threshold=0.005,
    #             stabilization_threshold=0.0005,
    #         ),
    #         # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    #         joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
    #         fixed_tendons_props=sim_utils.FixedTendonPropertiesCfg(limit_stiffness=30.0, damping=0.1),    
    #     ),
    #     init_state=ArticulationCfg.InitialStateCfg(
    #         pos=(1.0, 1.0, 2.0), 
    #     ),
    #     actuators={
    #         "fingers": ImplicitActuatorCfg(
    #             joint_names_expr=["joint_RFinger0", "joint_RFinger1", "joint_RFinger2"],
    #             effort_limit={
    #                 "joint_RFinger0": 0.9,
    #                 "joint_RFinger1": 0.9,
    #                 "joint_RFinger2": 0.9,
    #             },
    #             stiffness={
    #                 "joint_RFinger0": 1.0,
    #                 "joint_RFinger1": 1.0,
    #                 "joint_RFinger2": 1
    #             },
    #             damping={
    #                 "joint_RFinger0": 0.1,
    #                 "joint_RFinger1": 0.1,
    #                 "joint_RFinger2": 0.1,
    #             },
    #         )
    #     },
    #     soft_joint_pos_limit_factor=1.0,
    #     )

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["inspire_left"]
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
            root_state = robot.data.default_root_state.clone()
            # print(root_state.shape)
            # breakpoint()
            # root_state[:, :3] += scene.env_origins
            # robot.write_root_state_to_sim(root_state)
            # # set joint positions with some noise
            # joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            # joint_pos += torch.rand_like(joint_pos) * 0.1
            # robot.write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")
        # Apply random action
        # -- generate random joint efforts
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts)
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
    scene_cfg = CartpoleSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
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
