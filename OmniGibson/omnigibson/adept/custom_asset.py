from pathlib import Path

import omnigibson.lazy as lazy

from omnigibson.objects.usd_object import USDObject


class ADEPTAssetObject(USDObject):
    """通过相对 ADEPT 目录的路径加载自定义 USD 资产."""

    def __init__(self, name, asset_path, **kwargs):
        """保存可迁移的相对路径并解析当前安装位置."""
        super().__init__(name=name, usd_path=str(Path(__file__).parent / asset_path), **kwargs)

    def _prepare_to_load(self):
        """为未加密的项目资产准备原生物理结构."""
        stage = lazy.pxr.Usd.Stage.Open(self.usd_path)
        for prim in stage.Traverse():
            for name in ("inputs:diffuse_texture", "inputs:normalmap_texture"):
                attribute = prim.GetAttribute(name)
                if not attribute.IsValid():
                    continue
                asset = attribute.Get()
                if not asset.resolvedPath or not Path(asset.resolvedPath).is_file():
                    raise FileNotFoundError(f"Unresolved custom asset texture: {prim.GetPath()} {name}={asset.path}")
                print(f"ADEPT texture: {asset.resolvedPath}", flush=True)
        return self._preapply_articulation_root(self.usd_path)
