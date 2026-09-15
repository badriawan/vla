from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .env import FrankaMuJoCoEnv
from .policy import MockPolicy, make_remote_policy


def run_rollout(
    mode: str,
    prompt: str,
    steps: int,
    host: str,
    port: int,
    open_loop_horizon: int,
    save_frames: Path | None = None,
    no_render: bool = False,
) -> None:
    policy = MockPolicy() if mode == "mock" else make_remote_policy(host, port)
    save_frames and save_frames.mkdir(parents=True, exist_ok=True)

    with FrankaMuJoCoEnv(render_images=not no_render) as env:
        obs = env.reset()
        chunk = np.empty((0, 8), dtype=np.float32)
        chunk_index = 0
        for step in range(steps):
            if chunk_index >= min(len(chunk), open_loop_horizon):
                request = {**obs, "prompt": prompt}
                chunk = np.asarray(policy.infer(request)["actions"], dtype=np.float32)
                if chunk.ndim != 2 or chunk.shape[1] != env.action_dim:
                    raise ValueError(f"policy actions must be [horizon, 8], got {chunk.shape}")
                chunk_index = 0
            obs = env.step(chunk[chunk_index])
            chunk_index += 1
            if save_frames is not None:
                from PIL import Image

                Image.fromarray(obs["observation/exterior_image_1_left"]).save(
                    save_frames / f"frame_{step:05d}.png"
                )
        print(f"Completed {steps} simulation steps using {mode} policy.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run pi0.5 on a MuJoCo Franka scene")
    parser.add_argument("--mode", choices=("mock", "remote"), default="mock")
    parser.add_argument("--prompt", default="pick up the red cube")
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--open-loop-horizon", type=int, default=8)
    parser.add_argument("--save-frames", type=Path)
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Use blank camera frames for headless physics/client smoke tests",
    )
    args = parser.parse_args()
    run_rollout(**vars(args))


if __name__ == "__main__":
    main()
