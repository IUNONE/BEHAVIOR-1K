from pathlib import Path

import omnigibson as og
import omnigibson.lazy as lazy
from omnigibson.objects.dataset_object import DatasetObject


class ADEPTWoodenTable(DatasetObject):
    """保留原生桌子几何并为可见表面绑定木纹材质."""

    def _post_load(self):
        """创建使用原始网格 UV 和项目木纹贴图的粗糙木质表面."""
        super()._post_load()
        texture = Path(__file__).parent / "custom_assets/cube_bookcase/textures/wood_color.png"
        with og.sim.editing_usd():
            stage = og.sim.stage
            path = f"{self.prim_path}/WoodMaterial"
            material = lazy.pxr.UsdShade.Material.Define(stage, path)
            shader = lazy.pxr.UsdShade.Shader.Define(stage, f"{path}/Surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("roughness", lazy.pxr.Sdf.ValueTypeNames.Float).Set(0.65)
            shader.CreateInput("metallic", lazy.pxr.Sdf.ValueTypeNames.Float).Set(0.0)
            uv = lazy.pxr.UsdShade.Shader.Define(stage, f"{path}/UV")
            uv.CreateIdAttr("UsdPrimvarReader_float2")
            uv.CreateInput("varname", lazy.pxr.Sdf.ValueTypeNames.Token).Set("st")
            image = lazy.pxr.UsdShade.Shader.Define(stage, f"{path}/Texture")
            image.CreateIdAttr("UsdUVTexture")
            image.CreateInput("file", lazy.pxr.Sdf.ValueTypeNames.Asset).Set(lazy.pxr.Sdf.AssetPath(str(texture)))
            image.CreateInput("sourceColorSpace", lazy.pxr.Sdf.ValueTypeNames.Token).Set("sRGB")
            image.CreateInput("wrapS", lazy.pxr.Sdf.ValueTypeNames.Token).Set("repeat")
            image.CreateInput("wrapT", lazy.pxr.Sdf.ValueTypeNames.Token).Set("repeat")
            image.CreateInput("st", lazy.pxr.Sdf.ValueTypeNames.Float2).ConnectToSource(
                uv.CreateOutput("result", lazy.pxr.Sdf.ValueTypeNames.Float2)
            )
            shader.CreateInput("diffuseColor", lazy.pxr.Sdf.ValueTypeNames.Color3f).ConnectToSource(
                image.CreateOutput("rgb", lazy.pxr.Sdf.ValueTypeNames.Float3)
            )
            material.CreateSurfaceOutput().ConnectToSource(shader.CreateOutput("surface", lazy.pxr.Sdf.ValueTypeNames.Token))
            for link in self.links.values():
                for mesh in link.visual_meshes.values():
                    lazy.pxr.UsdShade.MaterialBindingAPI.Apply(mesh.prim).Bind(
                        material, bindingStrength=lazy.pxr.UsdShade.Tokens.strongerThanDescendants,
                    )
