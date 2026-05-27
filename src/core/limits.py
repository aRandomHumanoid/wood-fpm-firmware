"""Workspace bounds and motion limits."""

from __future__ import annotations

from dataclasses import dataclass


def _coerce_float(value: float, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be numeric, got {value!r}") from exc


class BoundsError(ValueError):
    pass


@dataclass(frozen=True)
class MachineBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        object.__setattr__(self, "x_min", _coerce_float(self.x_min, "MachineBounds.x_min"))
        object.__setattr__(self, "x_max", _coerce_float(self.x_max, "MachineBounds.x_max"))
        object.__setattr__(self, "y_min", _coerce_float(self.y_min, "MachineBounds.y_min"))
        object.__setattr__(self, "y_max", _coerce_float(self.y_max, "MachineBounds.y_max"))

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

    def __post_init__(self):
        object.__setattr__(self, "v_max_mm_s", _coerce_float(self.v_max_mm_s, "MotionLimits.v_max_mm_s"))
        object.__setattr__(self, "a_max_mm_s2", _coerce_float(self.a_max_mm_s2, "MotionLimits.a_max_mm_s2"))

    def clamp_velocity(self, v: float) -> float:
        return min(max(v, 0.0), self.v_max_mm_s)

    def clamp_acceleration(self, a: float) -> float:
        return min(max(a, 0.0), self.a_max_mm_s2)
