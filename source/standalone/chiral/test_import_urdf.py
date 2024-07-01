import argparse

from omni.isaac.lab.app import AppLauncher
app_launcher = AppLauncher()
simulation_app = app_launcher.app

"""Rest everything follows."""

import contextlib
import os

import carb
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.app

from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg
from omni.isaac.lab.utils.assets import check_file_path
from omni.isaac.lab.utils.dict import print_dict

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.importer.urdf")

def main():
    # Determine if there is a GUI to update:
    # acquire settings interface
    carb_settings_iface = carb.settings.get_settings()
    # read flag for whether a local GUI is enabled
    local_gui = carb_settings_iface.get("/app/window/enabled")
    # read flag for whether livestreaming GUI is enabled
    livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

    # Simulate scene (if not headless)
    if local_gui or livestream_gui:
        # Open the stage with USD
        # stage_utils.open_stage(urdf_converter.usd_path)
        # Reinitialize the simulation
        app = omni.kit.app.get_app_interface()
        # Run simulation
        with contextlib.suppress(KeyboardInterrupt):
            while app.is_running():
                # perform step
                app.update()
                
if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
