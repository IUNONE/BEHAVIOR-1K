from contextlib import ExitStack
from itertools import zip_longest
from pathlib import Path

import av
import cv2
import numpy as np

from omnigibson.adept.visuals import VideoWriter


def parse_layout(value):
    """解析行数 x 列数形式的视频布局."""
    parts = value.lower().split("x")
    if len(parts) != 2 or not all(part.isdecimal() and int(part) > 0 for part in parts):
        raise ValueError("layout must be positive ROWSxCOLS, e.g. 3x4.")
    return tuple(map(int, parts))


def list_instance_ids(task_dir):
    """按数字顺序列出任务目录中保存的全部实例, 检查所需状态文件."""
    directory = Path(task_dir) / "instances"
    ids = sorted(int(path.name) for path in directory.iterdir() if path.is_dir() and path.name.isdecimal())
    if not ids:
        raise ValueError(f"No instances found in {directory}.")
    for instance_id in ids:
        for name in ("tro_state.json", "task_parameters.json"):
            path = directory / str(instance_id) / name
            if not path.is_file():
                raise FileNotFoundError(path)
    return ids


def _draw_cell(canvas, rgb, row, column, width, height, header, instance_id):
    """按原始纵横比放置画面并在顶部居中绘制实例标签."""
    top, left = row * (height + header), column * width
    scale = min(width / rgb.shape[1], height / rgb.shape[0])
    image_width = max(1, round(rgb.shape[1] * scale))
    image_height = max(1, round(rgb.shape[0] * scale))
    resized = cv2.resize(rgb, (image_width, image_height), interpolation=cv2.INTER_AREA)
    y = top + header + (height - image_height) // 2
    x = left + (width - image_width) // 2
    canvas[y:y + image_height, x:x + image_width] = resized
    canvas[top:top + header, left:left + width] = (24, 31, 44)
    label = f"instance_id: {instance_id:03d}"
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = min(header / 50, width / 420)
    (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, 1)
    origin = (left + (width - text_width) // 2, top + (header + text_height - baseline) // 2)
    cv2.putText(canvas, label, origin, font, font_scale, (239, 204, 132), 1, cv2.LINE_AA)


def compose_instance_previews(videos, output_path, layout="1x1"):
    """将本次实例视频按数字编号分组并逐页串接为汇总视频.

    多格布局的单格宽度最多为 640 像素, 高度按原视频比例计算.
    默认单格布局保留原分辨率, 额外添加标题栏.
    每页从各视频首帧同步播放, 较短视频停留在末帧, 末页空位保留背景.
    """
    rows, columns = parse_layout(layout)
    ordered = sorted(videos.items())
    if not ordered:
        raise ValueError("No preview videos to compose.")
    with av.open(str(ordered[0][1])) as source:
        stream = source.streams.video[0]
        fps = stream.average_rate
        source_width, source_height = stream.width, stream.height
    per_page = rows * columns
    width = source_width if per_page == 1 else min(640, source_width)
    width += width % 2
    height = round(source_height * width / source_width)
    height += height % 2
    header = max(48, 2 * round(width / 40))
    writer = VideoWriter(output_path, fps)
    try:
        for start in range(0, len(ordered), per_page):
            page = ordered[start:start + per_page]
            with ExitStack() as stack:
                readers = [stack.enter_context(av.open(str(path))) for _, path in page]
                if any(reader.streams.video[0].average_rate != fps for reader in readers):
                    raise ValueError("Preview videos must have the same frame rate.")
                last_frames = [None] * len(page)
                for decoded in zip_longest(*(reader.decode(video=0) for reader in readers)):
                    canvas = np.full((rows * (height + header), columns * width, 3), (14, 19, 28), dtype=np.uint8)
                    for index, frame in enumerate(decoded):
                        if frame is not None:
                            last_frames[index] = frame.to_ndarray(format="rgb24")
                        if last_frames[index] is None:
                            raise ValueError(f"Empty preview video: {page[index][1]}")
                        _draw_cell(canvas, last_frames[index], index // columns, index % columns,
                                   width, height, header, page[index][0])
                    writer.write(canvas)
                if any(frame is None for frame in last_frames):
                    raise ValueError("A preview page contains an empty video.")
    finally:
        writer.close()
    return Path(output_path)
