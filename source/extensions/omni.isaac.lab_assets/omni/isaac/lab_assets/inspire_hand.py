"""
# Rights and License?

"""


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

INSPIRE_HAND_LEFT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path="/home/mandi/IsaacLab/handleft01151.usd", # TODO: replace this with NUCLEUS path
        fix_base=True,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
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
        pos=(0.0, 0.2, 0.5),
        # rot=(0.0, 0.0, -0.7071, 0.7071),
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

INSPIRE_HAND_RIGHT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path="/home/mandi/IsaacLab/handright01091.usd", #TODO: replace this with NUCLEUS path
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
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
        pos=(0.0, -0.2, 0.5),
        # rot=(0.0, 0.0, -0.7071, 0.7071),
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
"""Configuration of Shadow Hand robot."""
