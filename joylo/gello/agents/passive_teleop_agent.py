import numpy as np
import torch as th

from gello.agents.agent import Agent


class PassiveTeleopAgent(Agent):
    """Compose simulation actions without issuing any motor motion commands."""

    def __init__(self, config, joycon_agent, read_joints=None, close_reader=None):
        super().__init__()
        self.config = config
        self.joycon = joycon_agent
        self.read_joints = read_joints
        self.close_reader = close_reader
        self._held_joints = None

    def act(self, obs):
        if self.read_joints is None:
            # Refresh the held pose during simulation resets/pauses, not on every
            # running step (which would let contact forces gradually move it).
            if self._held_joints is None or obs["waiting_to_resume"] or obs.get("reset_joints", False):
                self._held_joints = th.cat([
                    obs[f"arm_{side}_joint_positions"].detach().cpu()
                    for side in ("left", "right")
                ]).to(dtype=th.float32).clone()
            action = self._held_joints.clone()
        else:
            joints = np.asarray(self.read_joints(), dtype=float)
            if joints.shape != (2 * self.config.motors_per_arm,) or not np.isfinite(joints).all():
                raise ValueError("Invalid JoyLo joint readings")
            action = th.from_numpy(joints[self.config.gello_to_obs_indices].astype(np.float32))
        if self.joycon is not None:
            action = th.cat([action, self.joycon.act(obs)])
        return action

    def close(self):
        if self.close_reader is not None:
            self.close_reader()
