import glfw
import numpy as np

from franka_pi05.env import FrankaMuJoCoEnv
from franka_pi05.teleop import KeyboardControl


def test_press_release_selection_and_pause():
    with FrankaMuJoCoEnv(render_images=False) as env:
        c = KeyboardControl()
        c.key(glfw.KEY_3, glfw.PRESS)
        c.key(glfw.KEY_UP, glfw.PRESS)
        assert c.action(env)[2] > 0
        assert np.count_nonzero(c.action(env)[:7]) == 1
        c.key(glfw.KEY_UP, glfw.RELEASE)
        assert not c.action(env)[:7].any()
        c.key(glfw.KEY_DOWN, glfw.PRESS)
        assert c.action(env)[2] < 0
        c.key(glfw.KEY_SPACE, glfw.PRESS)
        assert not c.action(env)[:7].any()
        c.key(glfw.KEY_SPACE, glfw.PRESS)
        assert not c.action(env)[:7].any()


def test_gripper_limits_and_reset():
    with FrankaMuJoCoEnv(render_images=False) as env:
        c = KeyboardControl()
        c.key(glfw.KEY_H, glfw.PRESS)
        assert c.action(env)[7] == 0
        c.key(glfw.KEY_G, glfw.PRESS)
        assert c.action(env)[7] == 1
        env.data.qpos[0] = env.model.jnt_range[0, 1] - 0.01
        c.key(glfw.KEY_UP, glfw.PRESS)
        assert c.action(env)[0] == 0
        c.key(glfw.KEY_UP, glfw.RELEASE)
        c.key(glfw.KEY_DOWN, glfw.PRESS)
        assert c.action(env)[0] < 0
        c.key(glfw.KEY_R, glfw.PRESS)
        assert c.reset_requested and not c.held
