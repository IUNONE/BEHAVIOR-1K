from pathlib import Path

from omnigibson.objects.usd_object import USDObject


class ADEPTAssetObject(USDObject):
    """通过相对 ADEPT 目录的路径加载自定义 USD 资产."""

    def __init__(self, name, asset_path, **kwargs):
        """保存可迁移的相对路径并解析当前安装位置."""
        super().__init__(name=name, usd_path=str(Path(__file__).parent / asset_path), **kwargs)

    def _prepare_to_load(self):
        """为未加密的项目资产准备原生物理结构."""
        return self._preapply_articulation_root(self.usd_path)
