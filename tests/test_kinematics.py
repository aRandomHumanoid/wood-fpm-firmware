"""Kinematics sanity checks. The PPM math is intricate, so these tests focus on:
  - Inverse at the origin → identity-ish (theta ≈ 0)
  - Inverse outside the reachable surface → ok=False
  - Forward kinematics returns finite (x, y, z) for joint angles within bounds
  - Inverse rejects targets beyond theta_max
"""

import math
import numpy as np
import pytest

from src.core.kinematics import PPMKinematics


@pytest.fixture
def kin():
    # Reasonable defaults so the kinematics is solvable.
    return PPMKinematics(Lc=100.0, H=20.0, D=15.0, G_deg=60.0, theta_max_deg=25.0)


def test_inverse_at_origin(kin):
    r = kin.inverse_kinematics(0.0, 0.0)
    assert r.ok, r.reason
    # At (0, 0), workspace is centered above the base; theta_cal should be ~0.
    assert abs(r.theta_deg) < 1e-3
    # phi is arctan2(0, 0) — implementation-defined; we just require finite.
    assert math.isfinite(r.phi_deg)


def test_inverse_outside_theta_max(kin):
    # Push x large enough that theta exceeds theta_max.
    big = kin.L_char * math.tan(math.radians(kin.theta_max_deg_value() + 5))
    r = kin.inverse_kinematics(big, 0.0)
    assert not r.ok
    assert "theta" in r.reason.lower()


def test_forward_finite(kin):
    x, y, z = kin.forward_kinematics(theta=0.05, phi=0.1)
    assert all(math.isfinite(v) for v in (x, y, z))


def test_inverse_unreachable_returns_ok_false(kin):
    # A target on the workspace plane that's beyond the reachable surface
    # should be rejected, not crash.
    r = kin.inverse_kinematics(1e6, 1e6)
    assert not r.ok


# Convenience helper so the test file doesn't need to know the private attr name.
def _theta_max_deg(self):
    return math.degrees(self.theta_max)


PPMKinematics.theta_max_deg_value = _theta_max_deg
