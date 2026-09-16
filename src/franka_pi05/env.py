from __future__ import annotations

from importlib.resources import files
from typing import Any

import mujoco
import numpy as np


class FrankaMuJoCoEnv:
    """Small Franka manipulation environment with DROID-style I/O.

    Actions are normalized ``[dq_1..dq_7, gripper]``. Joint values map to
    velocity limits and gripper maps from 0 (closed) to 1 (open).
    """

    control_hz = 15
    action_dim = 8

    def __init__(self, image_size: int = 224, render_images: bool = True):
        xml = files("franka_pi05").joinpath("assets/franka_table.xml")
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.image_size = image_size
        self.renderer = (
            mujoco.Renderer(self.model, height=image_size, width=image_size)
            if render_images
            else None
        )
        self._steps_per_action = max(1, round((1 / self.control_hz) / self.model.opt.timestep))
        self.reset()

    def reset(self) -> dict[str, Any]:
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        return self.observation()

    def render(self, camera: str) -> np.ndarray:
        if self.renderer is None:
            return np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.renderer.update_scene(self.data, camera=camera)
        return np.asarray(self.renderer.render()).copy()

    def observation(self) -> dict[str, Any]:
        q = self.data.qpos[:7].astype(np.float32).copy()
        grip = np.array([self.data.qpos[7:9].mean() / 0.04], dtype=np.float32)
        return {
            "observation/exterior_image_1_left": self.render("external"),
            "observation/wrist_image_left": self.render("wrist"),
            "observation/joint_position": q,
            "observation/gripper_position": grip,
        }

    def step(self, action: np.ndarray) -> dict[str, Any]:
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (self.action_dim,):
            raise ValueError(f"action must have shape (8,), got {action.shape}")
        action = np.clip(action, -1.0, 1.0)
        velocity_limits = np.array([2, 2, 2, 2, 2.5, 2.5, 2.5])
        self.data.ctrl[:7] = action[:7] * velocity_limits
        # OpenPI/DROID uses a binary gripper command: <=0.5 closed, >0.5 open.
        width = 0.04 if action[7] > 0.5 else 0.0
        self.data.ctrl[7:9] = width
        for _ in range(self._steps_per_action):
            # Approximate the gravity/Coriolis compensation supplied by the
            # real Franka low-level controller.
            self.data.qfrc_applied[:7] = self.data.qfrc_bias[:7]
            mujoco.mj_step(self.model, self.data)
        return self.observation()

    def close(self) -> None:
        if self.renderer is not None:
            renderer = self.renderer
            self.renderer = None
            if hasattr(renderer, "close"):
                renderer.close()
            else:
                # MuJoCo 2.3.x predates Renderer.close(). Free the render
                # resources before releasing the native OpenGL context.
                renderer._gl_context.make_current()
                renderer._mjr_context.free()
                renderer._gl_context.free()

    def __enter__(self) -> "FrankaMuJoCoEnv":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
