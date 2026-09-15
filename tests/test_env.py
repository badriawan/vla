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
