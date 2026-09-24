import argparse
import sys
import traceback
from pathlib import Path

import cv2
import torch as th

import omnigibson as og
import omnigibson.utils.transform_utils as T
from omnigibson.adept.scene import ADEPTScene
from omnigibson.macros import gm


def main():
    """输出书本局部六个轴向视图, 用于确认书脊外向轴和书底到书顶轴."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="outputs/adept/book_axes")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    gm.HEADLESS = args.headless
    gm.ENABLE_TRANSITION_RULES = False
    gm.USE_GPU_DYNAMICS = False
    output = Path(args.output_dir).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    config = {
        "scene": {"type": ADEPTScene.__name__, "use_floor_plane": False},
        "objects": [{"type": "DatasetObject", "name": "book", "category": "hardback",
                     "model": "aceozs", "fixed_base": True,
                     "position": [0.0, 0.0, 1.0], "orientation": [0.0, 0.0, 0.0, 1.0]}],
        "robots": [],
        "task": {"type": "DummyTask", "include_obs": False},
        "env": {"external_sensors": [{
            "sensor_type": "VisionSensor", "name": "inspection", "relative_prim_path": "/inspection",
            "modalities": ["rgb"], "position": [0.4, 0.0, 1.0],
            "sensor_kwargs": {"image_width": 1024, "image_height": 1024, "horizontal_aperture": 24.0},
        }]},
    }
    try:
        env = og.Environment(configs=config)
        book = env.scene.object_registry("name", "book")
        camera = env.external_sensors["inspection"]
        rotation = T.quat2mat(book.get_position_orientation()[1])
        center = (book.aabb[0] + book.aabb[1]) / 2
        images = []
        for axis, letter in enumerate("XYZ"):
            for sign, word in ((1, "plus"), (-1, "minus")):
                backward = rotation[:, axis] * sign
                up = rotation[:, 1 if axis == 2 else 2]
                right = th.nn.functional.normalize(th.linalg.cross(up, backward), dim=0)
                upward = th.linalg.cross(backward, right)
                orientation = T.mat2quat(th.stack([right, upward, backward], dim=1))
                camera.set_position_orientation(center + backward * 0.4, orientation)
                camera.reset_render_product()
                for _ in range(32):
                    og.sim.render()
                rgb = camera.get_obs()[0]["rgb"][..., :3].detach().cpu().numpy()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                label = f"{'+' if sign > 0 else '-'}{letter} face (book local frame)"
                cv2.putText(bgr, label, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 5)
                cv2.putText(bgr, label, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
                cv2.imwrite(str(output / f"{word}_{letter.lower()}.png"), bgr)
                images.append(bgr)
        overview = cv2.vconcat([cv2.hconcat(images[i:i + 2]) for i in range(0, 6, 2)])
        cv2.imwrite(str(output / "overview.png"), overview)
        print(f"Book axis views saved to {output.resolve()}", flush=True)
    except Exception:
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if og.app is not None:
            og.shutdown()


if __name__ == "__main__":
    main()
