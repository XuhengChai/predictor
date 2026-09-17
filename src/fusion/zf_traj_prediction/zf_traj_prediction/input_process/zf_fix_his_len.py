from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class HistoryFixConfig:
    mode: str = "none"
    keep_seconds: float | None = None
    drop_count: int | None = None
    fill_method: str = "linear"
    random_seed: int | None = None
    psi_end: float | None = None


class HistoryLengthFixer:
    def __init__(self, expected_length: int, frequency: float):
        self.expected_length = int(expected_length)
        self.frequency = float(frequency)
        self.dt = 1.0 / self.frequency

    @staticmethod
    def none() -> HistoryFixConfig:
        return HistoryFixConfig(mode="none")

    @staticmethod
    def keep_last_seconds(seconds: float) -> HistoryFixConfig:
        return HistoryFixConfig(mode="keep_last", keep_seconds=seconds, fill_method="cv")

    @staticmethod
    def drop_middle_frames(drop_count: int, fill_method: str = "linear", psi_end: float | None = None) -> HistoryFixConfig:
        return HistoryFixConfig(mode="drop_middle", drop_count=drop_count, fill_method=fill_method, psi_end=psi_end)

    @staticmethod
    def drop_random_frames(
        drop_count: int,
        fill_method: str = "linear",
        random_seed: int | None = None,
        psi_end: float | None = None,
    ) -> HistoryFixConfig:
        return HistoryFixConfig(
            mode="drop_random",
            drop_count=drop_count,
            fill_method=fill_method,
            random_seed=random_seed,
            psi_end=psi_end,
        )

    @staticmethod
    def compute_history_confidence(observed_frames: int, expected_frames: int) -> float:
        if expected_frames <= 0:
            return 0.0
        observed = max(0, min(int(observed_frames), int(expected_frames)))
        return observed / float(expected_frames)

    def apply(self, agent: np.ndarray, config: HistoryFixConfig | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        config = config or self.none()
        fixed_agent = np.array(agent, copy=True)
        history_len = len(fixed_agent)
        if history_len != self.expected_length:
            raise ValueError(f"Agent history length {history_len} != expected length {self.expected_length}")

        normalized_mode = (config.mode or "none").lower()
        observed_mask = np.ones(self.expected_length, dtype=bool)
        meta = {
            "mode": normalized_mode,
            "fill_method": config.fill_method,
            "expected_frames": self.expected_length,
            "observed_frames": self.expected_length,
            "confidence": 1.0,
            "total_history_error": 0.0,
        }

        if normalized_mode == "none":
            return fixed_agent, meta

        fill_method = config.fill_method
        if normalized_mode == "keep_last":
            if config.keep_seconds is None:
                raise ValueError("keep_seconds is required when mode='keep_last'")
            keep_frames = int(round(float(config.keep_seconds) * self.frequency))
            keep_frames = max(2, min(self.expected_length, keep_frames))
            observed_mask[:] = False
            observed_mask[-keep_frames:] = True
            meta["keep_seconds"] = config.keep_seconds
            meta["observed_frames"] = keep_frames
            fill_method = "cv"
        elif normalized_mode == "drop_middle":
            missing = _normalize_drop_count(config.drop_count, self.expected_length)
            start = (self.expected_length - missing) // 2
            observed_mask[start:start + missing] = False
            meta["drop_count"] = missing
        elif normalized_mode == "drop_random":
            missing = _normalize_drop_count(config.drop_count, self.expected_length)
            if missing >= self.expected_length - 2:
                raise ValueError("drop_random requires at least two boundary frames to remain observed")
            rng = np.random.default_rng(config.random_seed)
            drop_indices = np.sort(rng.choice(np.arange(1, self.expected_length - 1), size=missing, replace=False))
            observed_mask[drop_indices] = False
            meta["drop_count"] = missing
            meta["drop_indices"] = drop_indices.tolist()
        else:
            raise ValueError(f"Unsupported history fix mode: {config.mode}")

        original_xy = fixed_agent[:, :2].astype(float).copy()
        xy = original_xy.copy()
        xy[~observed_mask] = np.nan
        xy = _fill_missing_xy(
            xy=xy,
            observed_mask=observed_mask,
            agent=fixed_agent,
            dt=self.dt,
            fill_method=fill_method,
            psi_end=config.psi_end,
        )
        fixed_agent[:, :2] = xy
        meta["fill_method"] = fill_method
        meta["observed_frames"] = int(np.count_nonzero(observed_mask))
        meta["confidence"] = self.compute_history_confidence(meta["observed_frames"], self.expected_length)
        meta["total_history_error"] = _compute_total_history_error(original_xy, xy)
        return fixed_agent, meta


def _normalize_drop_count(drop_count: int | None, expected_length: int) -> int:
    if drop_count is None:
        raise ValueError("drop_count is required for the selected history fix mode")
    missing = int(drop_count)
    if missing <= 0:
        raise ValueError("drop_count must be positive")
    if missing >= expected_length:
        raise ValueError("drop_count must be smaller than expected history length")
    return missing


def _fill_missing_xy(
    xy: np.ndarray,
    observed_mask: np.ndarray,
    agent: np.ndarray,
    dt: float,
    fill_method: str,
    psi_end: float | None,
) -> np.ndarray:
    result = np.array(xy, copy=True)
    missing_runs = _find_missing_runs(observed_mask)
    for start, end in missing_runs:
        left = start - 1 if start > 0 and observed_mask[start - 1] else None
        right = end + 1 if end + 1 < len(observed_mask) and observed_mask[end + 1] else None
        if left is not None and right is not None:
            result[start:end + 1] = _fill_internal_gap(
                start_xy=result[left],
                end_xy=result[right],
                gap_size=end - start + 1,
                fill_method=fill_method,
                psi0=_heading_from_agent(agent, left),
                psi1=psi_end if psi_end is not None else _heading_from_agent(agent, right),
            )
        elif right is not None:
            vx, vy = _velocity_from_agent(agent, right, result)
            for idx in range(right - 1, -1, -1):
                result[idx] = result[idx + 1] - np.array([vx, vy]) * dt
        elif left is not None:
            vx, vy = _velocity_from_agent(agent, left, result)
            for idx in range(left + 1, len(result)):
                result[idx] = result[idx - 1] + np.array([vx, vy]) * dt
        else:
            raise ValueError("Unable to reconstruct history with no observed boundary points")
    return result


def _compute_total_history_error(
    original_xy: np.ndarray,
    filled_xy: np.ndarray,
) -> float:
    per_frame_error = np.linalg.norm(filled_xy - original_xy, axis=1)
    return float(np.sum(per_frame_error))


def _find_missing_runs(observed_mask: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start = None
    for idx, is_observed in enumerate(observed_mask):
        if not is_observed and start is None:
            start = idx
        elif is_observed and start is not None:
            runs.append((start, idx - 1))
            start = None
    if start is not None:
        runs.append((start, len(observed_mask) - 1))
    return runs


def _fill_internal_gap(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    gap_size: int,
    fill_method: str,
    psi0: float,
    psi1: float,
) -> np.ndarray:
    if gap_size <= 0:
        return np.empty((0, 2), dtype=float)
    normalized_method = (fill_method or "linear").lower()
    if normalized_method == "linear":
        return _linear_fill(start_xy, end_xy, gap_size)
    if normalized_method == "bezier":
        return _bezier_fill(start_xy, end_xy, gap_size, psi0, psi1)
    raise ValueError(f"Unsupported fill_method for internal gap: {fill_method}")


def _linear_fill(start_xy: np.ndarray, end_xy: np.ndarray, gap_size: int) -> np.ndarray:
    t = np.linspace(0.0, 1.0, gap_size + 2, dtype=float)[1:-1][:, None]
    return start_xy[None, :] * (1.0 - t) + end_xy[None, :] * t


def _bezier_fill(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    gap_size: int,
    psi0: float,
    psi1: float,
) -> np.ndarray:
    dist = np.linalg.norm(end_xy - start_xy)
    alpha = 0.35
    p0 = np.asarray(start_xy, dtype=float)
    p3 = np.asarray(end_xy, dtype=float)
    p1 = p0 + np.array([math.cos(psi0), math.sin(psi0)], dtype=float) * dist * alpha
    p2 = p3 - np.array([math.cos(psi1), math.sin(psi1)], dtype=float) * dist * alpha
    t = np.linspace(0.0, 1.0, gap_size + 2, dtype=float)[1:-1][:, None]
    return (
        ((1.0 - t) ** 3) * p0[None, :]
        + 3.0 * ((1.0 - t) ** 2) * t * p1[None, :]
        + 3.0 * (1.0 - t) * (t ** 2) * p2[None, :]
        + (t ** 3) * p3[None, :]
    )


def _velocity_from_agent(agent: np.ndarray, idx: int, xy: np.ndarray) -> tuple[float, float]:
    if agent.shape[1] > 3:
        vx = float(agent[idx, 2])
        vy = float(agent[idx, 3])
        if np.isfinite(vx) and np.isfinite(vy) and (abs(vx) > 1e-6 or abs(vy) > 1e-6):
            return vx, vy
    if idx + 1 < len(xy) and np.all(np.isfinite(xy[idx + 1])):
        delta = xy[idx + 1] - xy[idx]
        return float(delta[0]), float(delta[1])
    if idx > 0 and np.all(np.isfinite(xy[idx - 1])):
        delta = xy[idx] - xy[idx - 1]
        return float(delta[0]), float(delta[1])
    return 0.0, 0.0


def _heading_from_agent(agent: np.ndarray, idx: int) -> float:
    if agent.shape[1] > 5:
        vx = float(agent[idx, 2])
        vy = float(agent[idx, 3])
        if np.isfinite(vx) and np.isfinite(vy) and (abs(vx) > 1e-6 or abs(vy) > 1e-6):
            return math.atan2(vy, vx)
        yaw = float(agent[idx, 5])
        if np.isfinite(yaw):
            return yaw
    return 0.0