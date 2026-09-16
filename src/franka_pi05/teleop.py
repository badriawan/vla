"""Keyboard joint teleoperation in a native MuJoCo/GLFW window."""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np

from .env import FrankaMuJoCoEnv


class KeyboardControl:
    def __init__(self):
        self.joint = 0
        self.speed = 0.10  # normalized velocity, matching env.step
        self.gripper = 1.0
        self.held: set[int] = set()
        self.paused = False
        self.reset_requested = False

    def key(self, key: int, event: int) -> None:
        if event == glfw.RELEASE:
            self.held.discard(key)
            return
        if event != glfw.PRESS:
            return
        self.held.add(key)
        if glfw.KEY_1 <= key <= glfw.KEY_7:
            self.joint = key - glfw.KEY_1
        elif key == glfw.KEY_G:
            self.gripper = 1.0
        elif key == glfw.KEY_H:
            self.gripper = 0.0
        elif key == glfw.KEY_EQUAL:
            self.speed = min(0.30, self.speed + 0.025)
        elif key == glfw.KEY_MINUS:
            self.speed = max(0.025, self.speed - 0.025)
        elif key == glfw.KEY_SPACE:
            self.paused = not self.paused
            self.held.clear()
        elif key == glfw.KEY_R:
            self.reset_requested = True
            self.held.clear()

    def action(self, env: FrankaMuJoCoEnv) -> np.ndarray:
        action = np.zeros(8)
        action[7] = self.gripper
        if self.paused:
            return action
        direction = int(glfw.KEY_UP in self.held) - int(glfw.KEY_DOWN in self.held)
        joint_id = env.model.joint(f"joint{self.joint + 1}").id
        lo, hi = env.model.jnt_range[joint_id]
        q = env.data.qpos[self.joint]
        if (direction > 0 and q >= hi - 0.05) or (direction < 0 and q <= lo + 0.05):
            direction = 0
        action[self.joint] = direction * self.speed
        return action


def run(smoke_test: Path | None = None) -> None:
    if not glfw.init():
        raise RuntimeError("GLFW cannot start. Run from a macOS desktop terminal.")
    window = None
    context = None
    try:
        # Keep framebuffer and logical window dimensions aligned on Retina.
        glfw.window_hint(glfw.COCOA_RETINA_FRAMEBUFFER, glfw.FALSE)
        window = glfw.create_window(1100, 800, "Franka Panda | Keyboard Teleop", None, None)
        if not window:
            raise RuntimeError("Cannot create the teleop window")
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        with FrankaMuJoCoEnv(render_images=False) as env:
            control = KeyboardControl()
            scene = mujoco.MjvScene(env.model, maxgeom=2000)
            context = mujoco.MjrContext(env.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(camera)
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = [0.35, 0, 0.85]
            camera.distance = 2.2
            camera.azimuth = 135
            camera.elevation = -25
            options = mujoco.MjvOption()

            def key_callback(win, key, scancode, event, mods):
                if key == glfw.KEY_ESCAPE and event == glfw.PRESS:
                    glfw.set_window_should_close(win, True)
                control.key(key, event)

            def focus_callback(win, focused):
                if not focused:
                    control.held.clear()
                    control.paused = True

            def scroll_callback(win, x, y):
                camera.distance = float(np.clip(camera.distance * (0.9 ** y), 0.6, 5))

            glfw.set_key_callback(window, key_callback)
            glfw.set_window_focus_callback(window, focus_callback)
            glfw.set_scroll_callback(window, scroll_callback)
            print("Teleop ready: click window. 1-7 select joint; hold Up/Down to move; "
                  "G/H open/close; +/- speed; Space pause; R reset; Esc quit.", flush=True)
            next_step = time.monotonic()
            ticks = 0
            start_q = env.data.qpos[:7].copy()
            while not glfw.window_should_close(window):
                glfw.poll_events()
                if control.reset_requested:
                    env.reset()
                    control.gripper = 1.0
                    control.reset_requested = False
                now = time.monotonic()
                if now >= next_step:
                    if smoke_test is not None:
                        control.paused = False
                        key_callback(window, glfw.KEY_UP, 0,
                                     glfw.PRESS if ticks == 0 else glfw.RELEASE if ticks == 8 else glfw.REPEAT, 0)
                    if not control.paused:
                        env.step(control.action(env))
                    next_step = now + 1 / env.control_hz
                    ticks += 1
                width, height = glfw.get_framebuffer_size(window)
                if width == 0 or height == 0:
                    time.sleep(0.02)
                    continue
                viewport = mujoco.MjrRect(0, 0, width, height)
                mujoco.mjv_updateScene(env.model, env.data, options, None, camera,
                                      mujoco.mjtCatBit.mjCAT_ALL, scene)
                mujoco.mjr_render(viewport, scene, context)
                state = "PAUSED - Space to resume" if control.paused else "READY - hold Up/Down"
                text = (f"FRANKA PANDA | {state}\nJoint {control.joint + 1} | Speed {control.speed:.3f} | "
                        f"Gripper {'OPEN' if control.gripper else 'CLOSED'}\n"
                        "1-7: select joint    Up/Down: move selected joint\n"
                        "G: open gripper    H: close gripper    +/-: speed\n"
                        "Space: pause    R: reset    Scroll: zoom    Esc: quit")
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                   viewport, text, "", context)
                if smoke_test is not None and ticks >= 12:
                    from PIL import Image
                    pixels = np.empty((height, width, 3), dtype=np.uint8)
                    mujoco.mjr_readPixels(pixels, None, viewport, context)
                    smoke_test.parent.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(pixels[::-1]).save(smoke_test)
                    assert abs(env.data.qpos[0] - start_q[0]) > 0.01, "Keyboard input did not move joint"
                    assert control.action(env)[0] == 0, "Release did not stop command"
                    print("GUI smoke test passed: joint moved and key release stopped command.", flush=True)
                    break
                glfw.swap_buffers(window)
                time.sleep(0.005)
    finally:
        if context is not None:
            context.free()
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke-test", type=Path, help="Exercise controls, save screenshot, then exit")
    args = parser.parse_args()
    run(args.smoke_test)


if __name__ == "__main__":
    main()
