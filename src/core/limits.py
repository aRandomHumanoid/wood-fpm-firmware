"""Workspace bounds and motion limits."""

from __future__ import annotations

from dataclasses import dataclass


class BoundsError(ValueError):
    pass


class OvercurrentError(RuntimeError):
    """Raised when an axis's measured current exceeds the configured
    overcurrent limit during a move. Motion is halted before the raise."""


@dataclass(frozen=True)
class MachineBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def check(self, x: float, y: float) -> None:
        if not self.contains(x, y):
            raise BoundsError(
                f"({x:.3f}, {y:.3f}) outside workspace "
                f"[{self.x_min}, {self.x_max}] x [{self.y_min}, {self.y_max}]"
            )

    def clamp(self, x: float, y: float) -> tuple[float, float]:
        return (
            min(max(x, self.x_min), self.x_max),
            min(max(y, self.y_min), self.y_max),
        )


@dataclass(frozen=True)
class MotionLimits:
    v_max_mm_s: float
    a_max_mm_s2: float
    overcurrent_mA: int = 0  # 0 = disabled

    def clamp_velocity(self, v: float) -> float:
        return min(max(v, 0.0), self.v_max_mm_s)

    def clamp_acceleration(self, a: float) -> float:
        return min(max(a, 0.0), self.a_max_mm_s2)
