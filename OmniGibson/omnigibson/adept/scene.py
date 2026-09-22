from omnigibson.scenes.scene_base import Scene
from omnigibson.utils.asset_utils import get_bddl_version, get_omnigibson_version
from packaging.version import Version


class ADEPTScene(Scene):
    """保存 ADEPT 场景及软件版本信息."""

    @staticmethod
    def _get_current_versions():
        """读取已安装的软件版本, 避免调用版本控制命令."""
        return {
            "omnigibson": {"version": get_omnigibson_version()},
            "bddl": {"version": get_bddl_version()},
        }

    @staticmethod
    def _check_versions_compatible(saved_versions):
        """核对场景的软件版本兼容性."""
        for component, current in ADEPTScene._get_current_versions().items():
            saved = saved_versions.get(component, {}).get("version")
            if saved is not None and Version(saved) > Version(current["version"]):
                raise ValueError(f"Scene requires newer {component}: {saved}")
