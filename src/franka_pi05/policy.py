from __future__ import annotations

from typing import Any, Protocol

import numpy as np


class Policy(Protocol):
    def infer(self, observation: dict[str, Any]) -> dict[str, np.ndarray]: ...


class MockPolicy:
    """Deterministic policy used to validate simulation plumbing."""

    def __init__(self, horizon: int = 10):
        self.horizon = horizon

    def infer(self, observation: dict[str, Any]) -> dict[str, np.ndarray]:
        del observation
        actions = np.zeros((self.horizon, 8), dtype=np.float32)
        actions[:, 0] = 0.08
        actions[:, 7] = 1.0
        return {"actions": actions}


def make_remote_policy(host: str, port: int) -> Policy:
    try:
        from openpi_client import websocket_client_policy
    except ImportError as exc:
        raise RuntimeError(
            "openpi-client is missing. Install it from the OpenPI repository's "
            "packages/openpi-client directory."
        ) from exc
    return websocket_client_policy.WebsocketClientPolicy(host=host, port=port)

