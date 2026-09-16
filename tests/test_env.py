import numpy as np

from franka_pi05.env import FrankaMuJoCoEnv
from franka_pi05.policy import MockPolicy


def test_observation_and_action_contract():
    with FrankaMuJoCoEnv(image_size=64, render_images=False) as env:
        obs = env.reset()
        assert obs["observation/exterior_image_1_left"].shape == (64, 64, 3)
        assert obs["observation/wrist_image_left"].shape == (64, 64, 3)
        assert obs["observation/joint_position"].shape == (7,)
        assert obs["observation/gripper_position"].shape == (1,)
        result = MockPolicy().infer({**obs, "prompt": "test"})
        next_obs = env.step(result["actions"][0])
        assert np.isfinite(next_obs["observation/joint_position"]).all()


def test_rejects_wrong_action_shape():
    with FrankaMuJoCoEnv(image_size=32, render_images=False) as env:
        try:
            env.step(np.zeros(7))
        except ValueError as exc:
            assert "shape (8,)" in str(exc)
        else:
            raise AssertionError("invalid action was accepted")


def test_zero_velocity_holds_arm_pose():
    with FrankaMuJoCoEnv(image_size=32, render_images=False) as env:
        before = env.data.qpos[:7].copy()
        for _ in range(10):
            env.step(np.zeros(8))
        np.testing.assert_allclose(env.data.qpos[:7], before, atol=2e-2)


def test_panda_model_and_camera_contract():
    with FrankaMuJoCoEnv(render_images=False) as env:
        assert env.model.nmesh > 0
        assert env.model.nu == 9
        assert env.model.nq == 16
        assert env.model.camera("wrist").bodyid == env.model.body("hand").id
        # Panda's hand hangs above the tabletop in the initial pose.
        tcp = env.data.site("tcp").xpos
        assert 0.2 < tcp[0] < 0.8
        assert tcp[2] > 0.5


def test_close_legacy_renderer_is_idempotent():
    from types import SimpleNamespace
    from unittest.mock import Mock

    with FrankaMuJoCoEnv(render_images=False) as env:
        gl = Mock()
        mjr = Mock()
        env.renderer = SimpleNamespace(_gl_context=gl, _mjr_context=mjr)
        env.close()
        env.close()
        gl.make_current.assert_called_once()
        mjr.free.assert_called_once()
        gl.free.assert_called_once()
