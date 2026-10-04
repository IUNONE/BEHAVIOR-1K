"""Read-only position streaming after disabling torque on the selected arm."""
import threading
import time
from pathlib import Path
import numpy as np
import yaml


class ArmStream:
    def __init__(self, port, baudrate, config, arm):
        if arm not in ('left', 'right'):
            raise ValueError('Expected left or right arm')
        self.arm = arm
        data = yaml.safe_load(Path(config).read_text())
        joints = data['joints']
        self.ids = list(range(0, 9) if arm == 'left' else range(9, 18))
        if data.get('robot') != 'R1Pro' or data.get('arm') != arm or joints.get('ids') != self.ids:
            raise ValueError(f'Expected an R1Pro {arm}-arm calibration with IDs {self.ids[0]}–{self.ids[-1]}')
        self.offsets = np.asarray(joints['offsets'], dtype=float)
        self.signs = np.asarray(joints['signs'], dtype=float)
        if (self.offsets.shape != (9,) or self.signs.shape != (9,)
                or not np.isfinite(self.offsets).all() or not np.isin(self.signs, [-1, 1]).all()):
            raise ValueError('Invalid calibration offsets/signs')
        self.port_name, self.baudrate = port, baudrate
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sample = None
        self.status = 'Connecting'
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.status, self.sample

    def _run(self):
        from dynamixel_sdk import PortHandler, PacketHandler
        port = PortHandler(self.port_name)
        packet = PacketHandler(2.0)
        def checked(result, error):
            if result != 0 or error:
                raise RuntimeError(packet.getTxRxResult(result) if result else packet.getRxPacketError(error))
        try:
            if not port.openPort() or not port.setBaudRate(self.baudrate):
                raise RuntimeError('Cannot open serial port/set baudrate')
            for motor in self.ids:
                result, error = packet.write1ByteTxRx(port, motor, 64, 0)
                checked(result, error)
            while not self.stop.is_set():
                raw = []
                for motor in self.ids:
                    value, result, error = packet.read4ByteTxRx(port, motor, 132)
                    checked(result, error)
                    if value >= 2**31:
                        value -= 2**32
                    raw.append(value * 360 / 4096)
                raw = np.asarray(raw)
                angles = (raw - self.offsets) * self.signs
                with self.lock:
                    self.sample = (time.monotonic(), raw, angles)
                    self.status = 'Connected / torque OFF'
                self.stop.wait(1 / 30)
        except Exception as error:
            with self.lock:
                self.status = f'ERROR: {error}. Restart viewer after checking connection.'
        finally:
            port.closePort()

    def close(self):
        self.stop.set()
        self.thread.join()
