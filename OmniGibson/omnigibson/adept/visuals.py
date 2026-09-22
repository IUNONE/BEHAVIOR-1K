from pathlib import Path
from fractions import Fraction

import av
import cv2
import numpy as np


CAMERA_LINKS = {"head": "zed_link", "left_wrist": "left_realsense_link", "right_wrist": "right_realsense_link"}


def cameras(env):
    """返回机器人三路相机与固定桌侧相机."""
    robot = env.robots[0]
    sensors = {role: robot.sensors[f"{robot.name}:{link}:Camera:0"] for role, link in CAMERA_LINKS.items()}
    sensors["table_side"] = env.external_sensors["table_side"]
    return sensors


def frames(env):
    """读取四路相机当前 RGB 帧."""
    return {role: sensor.get_obs()[0]["rgb"][..., :3].detach().cpu().numpy() for role, sensor in cameras(env).items()}


def mosaic(images):
    """将四路视角排列成统一尺寸的预览画面."""
    images = {key: cv2.resize(value, (640, 480)) for key, value in images.items()}
    return np.concatenate([np.concatenate([images["head"], images["table_side"]], axis=1),
                           np.concatenate([images["left_wrist"], images["right_wrist"]], axis=1)], axis=0)


class VideoWriter:
    """逐帧写入 RGB 视频并完成编码收尾."""

    def __init__(self, path, fps):
        """创建视频文件和编码流."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.container = av.open(str(path), "w")
        self.stream = self.container.add_stream("libx264", rate=Fraction(str(fps)))
        self.stream.pix_fmt = "yuv420p"
        self.initialized = False

    def write(self, rgb):
        """编码一帧 RGB 图像."""
        if not self.initialized:
            self.stream.height, self.stream.width = rgb.shape[:2]
            self.initialized = True
        for packet in self.stream.encode(av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")):
            self.container.mux(packet)

    def close(self):
        """写入编码缓冲区并关闭文件."""
        if self.initialized:
            for packet in self.stream.encode():
                self.container.mux(packet)
        self.container.close()
