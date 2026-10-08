from pathlib import Path
from omnigibson.robots.robot import Robot
from omnigibson.macros import gm


REGISTERED_ROBOTS = []
for yaml_file in sorted(Path(gm.DATA_PATH).glob("*/models/*/*.yaml")):
    if yaml_file.stem == yaml_file.parent.name:
        REGISTERED_ROBOTS.append(yaml_file.stem)

# ADEPT keeps extra robot definitions beside the code. Only a yaml whose stem matches its
# parent directory is a robot; import_config.yaml and poses.yaml are not registered.
_ADEPT_CUSTOM_ASSETS = Path(__file__).resolve().parents[1] / "adept" / "custom_assets"
if _ADEPT_CUSTOM_ASSETS.is_dir():
    for yaml_file in sorted(_ADEPT_CUSTOM_ASSETS.glob("*/*.yaml")):
        if yaml_file.stem == yaml_file.parent.name and yaml_file.stem not in REGISTERED_ROBOTS:
            REGISTERED_ROBOTS.append(yaml_file.stem)

__all__ = [
    "Robot",
    "REGISTERED_ROBOTS",
]
