from __future__ import annotations

import base64
import inspect
import json

# 消息类型与 openwam/deploy/server.py 保持同一套冻结字符串, 仿真环境不导入 openwam.
OBS = "obs"
RESET = "reset"
PING = "ping"
PREPARE_EVAL = "prepare_eval"
ACTION = "action"
RESET_ACK = "reset_ack"
PONG = "pong"
EVAL_READY = "eval_ready"
ERROR = "error"
ERR_INTERNAL = "internal_error"

try:
    from websockets.exceptions import ConnectionClosed as _ConnectionClosed

    _DROP_ERRORS: tuple = (OSError, _ConnectionClosed)
except ImportError:
    _DROP_ERRORS = (OSError,)


class ServerError(RuntimeError):
    """表示推理服务返回的结构化错误."""

    def __init__(self, status: int, code: str = "", message: str = "", raw_body: str = ""):
        """保存状态码, 错误码和消息.

        参数
        ----
        status : int
            HTTP 风格状态码. 校验错误为 400, 服务内部错误为 500.
        code : str
            服务端错误码.
        message : str
            服务端错误消息.
        raw_body : str
            原始响应文本.
        """
        descriptor = f"[{status}] {code or 'http_error'}: {message or raw_body or '<empty body>'}"
        super().__init__(descriptor)
        self.status = status
        self.code = code
        self.message = message
        self.raw_body = raw_body


def server_error_from_body(status: int, body: dict, raw_body: str = "") -> ServerError:
    """用服务端错误体构造 ServerError.

    参数
    ----
    status : int
        HTTP 风格状态码.
    body : dict
        含 code 与 message 的错误体.
    raw_body : str
        原始响应文本.

    返回
    ----
    ServerError
        结构化错误.
    """
    info = body if isinstance(body, dict) else {}
    return ServerError(
        status=status,
        code=info.get("code", ""),
        message=info.get("message", ""),
        raw_body=raw_body,
    )


def encode_numpy_b64(image) -> str:
    """把 HxWx3 uint8 RGB 编码为无损 PNG 的 base64 字符串.

    参数
    ----
    image : numpy.ndarray
        不做缩放的原始相机图像.

    返回
    ----
    str
        base64 编码的 PNG.
    """
    import io

    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="PNG", compress_level=1)
    return base64.b64encode(buffer.getvalue()).decode("utf-8")


def decode_numpy_b64(encoded):
    """将服务端无损 PNG 解码为 RGB uint8 数组."""
    import io

    import numpy as np
    from PIL import Image

    with Image.open(io.BytesIO(base64.b64decode(encoded))) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8).copy()


def build_payload(
    head: str,
    left_wrist: str | None = None,
    right_wrist: str | None = None,
    prompt: str = "",
    state: list | None = None,
) -> dict:
    """组装一条观测消息.

    参数
    ----
    head : str
        头部相机 PNG 的 base64.
    left_wrist : str or None
        左腕相机 PNG 的 base64.
    right_wrist : str or None
        右腕相机 PNG 的 base64.
    prompt : str
        原样发给模型的指令.
    state : list or None
        未归一化的本体状态.

    返回
    ----
    dict
        不含 type 字段的观测体. 传输层再写入 obs.
    """
    payload = {
        "images": {
            "head_camera": head,
            "left_wrist_camera": left_wrist,
            "right_wrist_camera": right_wrist,
        },
        "prompt": prompt,
    }
    if state is not None:
        payload["state"] = list(state)
    return payload


class WSPolicyClient:
    """用一条持久 websocket 连接收发观测, 复位和探活.

    注意事项
    --------
    timeout 限制单次往返. open_timeout 只限制建连, 避免探活被推理超时拖住.
    predict_once 在连接断开时不重发观测, 避免一次掉线把服务端缓存的下一步动作消耗掉.
    """

    def __init__(
        self,
        ws_url: str,
        timeout: float = 300.0,
        open_timeout: float = 10.0,
        compression: str | None = None,
    ):
        """保存连接参数.

        参数
        ----
        ws_url : str
            形如 ws://127.0.0.1:8848 的服务地址.
        timeout : float
            单次接收超时, 单位秒.
        open_timeout : float
            建连超时, 单位秒.
        compression : str or None
            websocket 压缩扩展. None 表示关闭 permessage-deflate.
        """
        self.ws_url = ws_url
        self.timeout = timeout
        self.open_timeout = open_timeout
        self.compression = compression
        self._ws = None

    def _connect(self) -> None:
        """在尚未连接时建立 websocket."""
        if self._ws is not None:
            return
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise ImportError(
                "WebSocket transport needs websockets>=12 (websockets.sync). Install with: pip install -U websockets"
            ) from exc
        # 多相机 base64 会超过默认 1 MB. 关闭 keepalive, 否则首次推理超过 20 秒会被对端掐断.
        # proxy=None 避免 websockets>=15 把本机连接送进 HTTP 代理.
        # 旧版本的 connect 不接受 ping_interval 或 proxy, 只传入当前签名里存在的参数.
        kwargs = {
            "max_size": None,
            "compression": self.compression,
            "open_timeout": self.open_timeout,
            "ping_interval": None,
            "proxy": None,
        }
        supported = set(inspect.signature(connect).parameters)
        self._ws = connect(self.ws_url, **{key: value for key, value in kwargs.items() if key in supported})

    def _roundtrip(self, message: dict, *, reconnect: bool = True) -> dict:
        """发送一条 JSON 并返回解析后的响应.

        参数
        ----
        message : dict
            已包含 type 的消息.
        reconnect : bool
            为 True 时, 发送前连接断开则重连并重发一次.

        返回
        ----
        dict
            服务端 JSON 对象.
        """
        body = json.dumps(message)
        attempts = (0, 1) if reconnect else (0,)
        for attempt in attempts:
            self._connect()
            try:
                self._ws.send(body)
                try:
                    raw = self._ws.recv(timeout=self.timeout)
                except TypeError:
                    raw = self._ws.recv()
                break
            except _DROP_ERRORS:
                self.close()
                if attempt == attempts[-1]:
                    raise
        data = json.loads(raw)
        if isinstance(data, dict) and data.get("type") == ERROR:
            status = 500 if data.get("code") == ERR_INTERNAL else 400
            raise server_error_from_body(status, data, raw)
        return data

    def predict(self, payload: dict) -> dict:
        """发送观测, 连接断开时重连并重发一次.

        参数
        ----
        payload : dict
            build_payload 的返回值.

        返回
        ----
        dict
            服务端响应.
        """
        return self._roundtrip({**payload, "type": OBS})

    def predict_once(self, payload: dict) -> dict:
        """发送一条观测, 连接断开时不重发.

        参数
        ----
        payload : dict
            build_payload 的返回值.

        返回
        ----
        dict
            服务端响应.
        """
        return self._roundtrip({**payload, "type": OBS}, reconnect=False)

    def reset(self) -> dict:
        """请求服务端清空本回合动作缓存.

        返回
        ----
        dict
            服务端响应.
        """
        return self._roundtrip({"type": RESET})

    def prepare_evaluation(self, payload: dict) -> dict:
        """发送实际初态, 完成模型预热并读取记录元数据."""
        return self._roundtrip({**payload, "type": PREPARE_EVAL}, reconnect=False)

    def ping(self) -> dict:
        """探活. 失败时不重试.

        返回
        ----
        dict
            服务端响应.
        """
        return self._roundtrip({"type": PING}, reconnect=False)

    def close(self) -> None:
        """关闭当前连接."""
        if self._ws is not None:
            try:
                self._ws.close()
            finally:
                self._ws = None
