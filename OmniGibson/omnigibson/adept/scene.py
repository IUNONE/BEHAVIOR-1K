from omnigibson.adept.wooden_table import ADEPTWoodenTable  # noqa: F401
from omnigibson.adept.custom_asset import ADEPTAssetObject  # noqa: F401
from omnigibson.scenes.scene_base import Scene
from omnigibson.utils.asset_utils import get_bddl_version, get_omnigibson_version
from packaging.version import Version


class ADEPTScene(Scene):
    """保存 ADEPT 场景及软件版本信息."""

    def _should_load_object(self, obj_info, task_metadata):
        """排除 ADEPT 模板中的通用 Robot, 由当前环境配置重新创建机器人."""
        if not self._include_robots and obj_info["class_name"] == "Robot":
            return False
        return super()._should_load_object(obj_info, task_metadata)

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
