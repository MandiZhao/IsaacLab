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
        asset_path="inspire_hand_isaac/handleft01151/urdf/handleft01151_test.urdf",
        fix_base=True,
        import_inertia_tensor=True,
        self_collision=False,
        default_drive_type="position",
        default_drive_stiffness=1.0,
        default_drive_damping=1000,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=True,
            max_depenetration_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"), 
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.2, 0.2, 0.5),
        # rot=(0.0, 0.0, -0.7071, 0.7071),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_TH_J1", "L_TH_J2", 
                "L_FF_J1", "L_MF_J1", "L_RF_J1", "L_LF_J1"], # "L_LF_J2", "L_LF_J3", "L_TH_J3", "L_TH_J4"],
            effort_limit={
                "L_TH_J1": 1.30,
                "L_TH_J2": 0.68,
                "L_FF_J1": 1.62,
                "L_MF_J1": 1.62,
                "L_LF_J1": 1.62,
                "L_RF_J1": 1.62,
            },
            stiffness={
                "L_TH_J1": 1.0,
                "L_TH_J2": 1.0,
                "L_FF_J1": 1.0,
                "L_MF_J1": 1.0,
                "L_LF_J1": 1.0,
                "L_RF_J1": 1.0,
            },
            damping={
                "L_TH_J1": 5000,
                "L_TH_J2": 5000,
                "L_FF_J1": 5000,
                "L_MF_J1": 5000,
                "L_LF_J1": 5000,
                "L_RF_J1": 5000,
            },
        )
    },
    soft_joint_pos_limit_factor=1.0,
)
 
 
INSPIRE_HAND_RIGHT_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        # usd_path="/home/mandi/IsaacLab/handright01091.usd", #TODO: replace this with NUCLEUS path
        asset_path="inspire_hand_isaac/handright01091/urdf/handright01091_test.urdf",
        fix_base=False,
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
            joint_names_expr=[
                "R_TH_J1", "R_TH_J2",
                "R_FF_J1", "R_MF_J1", "R_RF_J1", "R_LF_J1"], # "R_LF_J2", "R_LF_J3", "R_TH_J3", "R_TH_J4"],
            effort_limit={
                "R_TH_J1": 1.30,
                "R_TH_J2": 0.68,
                "R_FF_J1": 1.62,
                "R_MF_J1": 1.62,
                "R_LF_J1": 1.62,
                "R_RF_J1": 1.62,
            },
            stiffness={
                "R_TH_J1": 1.0,
                "R_TH_J2": 1.0,
                "R_FF_J1": 1.0,
                "R_MF_J1": 1.0,
                "R_LF_J1": 1.0,
                "R_RF_J1": 1.0,
            },
            damping={
                "R_TH_J1": 0.1,
                "R_TH_J2": 0.1,
                "R_FF_J1": 0.1,
                "R_MF_J1": 0.1,
                "R_LF_J1": 0.1,
                "R_RF_J1": 0.1,
            },
        )
    },
    soft_joint_pos_limit_factor=1.0,
) 
