"""PPM (Parallel Platform Mechanism) kinematics.

Adapted from `inverse-example.txt`. Visualization, animation, noise simulation,
and surface-fitting helpers from the original have been removed; only the
forward and inverse kinematics needed for motion control remain.

Conventions:
  - `inverse_kinematics(x_mm, y_mm)` returns joint angles in DEGREES.
  - `forward_kinematics(theta_rad, phi_rad)` takes joint angles in RADIANS
    and returns end-effector (x, y, z) in the same units as the link lengths.
  - Callers (MotionController) handle deg<->rad conversion at the boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class KinematicsResult:
    ok: bool
    theta_deg: float
    phi_deg: float
    reason: str = ""


class PPMKinematics:
    def __init__(
        self,
        Lc: float,
        H: float,
        D: float,
        G_deg: float,
        theta_max_deg: float = 25.0,
    ):
        self.L_char = float(Lc)
        self.Ht = float(H)
        self.Dt = float(D)
        self.Gt = float(G_deg)
        self.theta_max = np.deg2rad(theta_max_deg)

        G = np.deg2rad(G_deg)
        self.A0 = (Lc - 2 * H) / 2
        self.B0 = np.sqrt(H ** 2 + D ** 2)
        self.C0 = np.sqrt((2 * self.A0 + H) ** 2 + D ** 2)
        self.D0 = np.sqrt((D + D * np.cos(G / 2)) ** 2 + (D * np.sin(G / 2)) ** 2)

        self.A = np.array([self.A0] * 2)
        self.B = np.array([self.B0] * 6)
        self.C = np.array([self.C0] * 3)
        self.D = np.array([self.D0] * 2)

        self._T = np.eye(4)
        self._Rx = np.eye(4)
        self._Ry = np.eye(4)
        self._Rz = np.eye(4)

        # Cache the end-effector position at the home configuration so
        # forward_kinematics() can return user-facing offsets where (0,0,*)
        # is the home pose. (The raw N6 in the example is not zero at home.)
        self._home_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._home_xyz = self._raw_forward(0.0, 0.0)

    # -------------------- math helpers --------------------

    @staticmethod
    def _law_of_cos(L1: float, L2: float, L3: float) -> float:
        ratio = (L2 ** 2 + L3 ** 2 - L1 ** 2) / (2 * L2 * L3)
        ratio = float(np.clip(ratio, -1.0, 1.0))
        return float(np.arccos(ratio))

    @staticmethod
    def _law_of_cos_theta(L1: float, L2: float, theta: float) -> float:
        return float(np.sqrt(L1 ** 2 + L2 ** 2 - 2 * L1 * L2 * np.cos(theta)))

    @staticmethod
    def _law_of_sin(L1: float, theta1: float, L2: float) -> float:
        v = L2 * (np.sin(theta1) / L1)
        return float(np.arcsin(np.clip(v, -1.0, 1.0)))

    def _t_xyz(self, dx, dy, dz):
        M = np.eye(4)
        M[0, 3] = dx
        M[1, 3] = dy
        M[2, 3] = dz
        return M

    def _r_x(self, t):
        M = np.eye(4)
        c, s = np.cos(t), np.sin(t)
        M[1, 1] = c; M[1, 2] = -s
        M[2, 1] = s; M[2, 2] = c
        return M

    def _r_y(self, t):
        M = np.eye(4)
        c, s = np.cos(t), np.sin(t)
        M[0, 0] = c;  M[0, 2] = s
        M[2, 0] = -s; M[2, 2] = c
        return M

    @staticmethod
    def _trilaterate(P1, P2, P3, r1, r2, r3):
        x3, y3, z3 = P1
        x4, y4, z4 = P2
        x5, y5, z5 = P3
        d = 2 * (y3 * (z4 - z5) + y4 * (z5 - z3) + y5 * (z3 - z4))
        w3 = x3 ** 2 + y3 ** 2 + z3 ** 2 - r1 ** 2
        w4 = x4 ** 2 + y4 ** 2 + z4 ** 2 - r2 ** 2
        w5 = x5 ** 2 + y5 ** 2 + z5 ** 2 - r3 ** 2
        a1 = (-2 / d) * (x3 * (z4 - z5) + x4 * (z5 - z3) + x5 * (z3 - z4))
        b1 = (1 / d) * (z3 * (w5 - w4) + z4 * (w3 - w5) + z5 * (w4 - w3))
        a2 = (2 / d) * (x3 * (y4 - y5) + x4 * (y5 - y3) + x5 * (y3 - y4))
        b2 = (-1 / d) * (y3 * (w5 - w4) + y4 * (w3 - w5) + y5 * (w4 - w3))
        roots = np.roots(
            [
                a1 ** 2 + a2 ** 2 + 1,
                2 * a1 * (b1 - y3) - 2 * x3 + 2 * a2 * (b2 - z3),
                x3 ** 2 + (b1 - y3) ** 2 + (b2 - z3) ** 2 - r1 ** 2,
            ]
        )
        x = roots[0]
        return np.array([x, a1 * x + b1, a2 * x + b2], dtype=float).real

    # -------------------- forward kinematics --------------------

    def forward_kinematics(self, theta: float, phi: float) -> Tuple[float, float, float]:
        """Return user-facing (x, y, z) for joint angles (rad).

        The X and Y components are reported relative to the home pose so that
        ``forward_kinematics(0, 0) == (0, 0, z_home)``. Z is the absolute
        height returned by the raw mechanism math.
        """
        x, y, z = self._raw_forward(theta, phi)
        hx, hy, _hz = self._home_xyz
        return x - hx, y - hy, z

    def _raw_forward(self, theta: float, phi: float) -> Tuple[float, float, float]:
        """Raw 3D end-effector position, unshifted. Used internally."""
        theta = -theta
        phi = -phi

        N1 = np.array([self.A[0], 0.0, 0.0])

        L2 = self._law_of_cos_theta(self.A[0], self.A[1], np.pi - theta)
        psi_L = self._law_of_sin(L2, np.pi - theta, self.A[1])

        T_N2 = self._r_x(phi) @ self._r_y(psi_L) @ self._t_xyz(L2, 0.0, 0.0) @ self._r_x(-phi)
        N2 = T_N2[0:3, 3]

        theta_B0 = self._law_of_cos(self.C[0], L2, self.B[0]) - np.pi
        theta_B1 = self._law_of_cos(self.C[1], L2, self.B[1]) - np.pi
        theta_B2 = self._law_of_cos(self.C[2], L2, self.B[2]) - np.pi

        # Enforce plane(self.p, self.n) for N4
        d = N2 - N1
        k = np.array([0.0, 0.0, 1.0])
        if np.linalg.norm(d) > 1e-9:
            u = d / np.linalg.norm(d)
            n_plane = np.cross(u, k)
        else:
            n_plane = np.array([0.0, 1.0, 0.0])
        p_plane = N2

        R0 = T_N2[0:3, 0:3]
        p2 = N2.copy()
        n_local = R0.T @ n_plane
        q_local = R0.T @ (p_plane - p2)

        Ry_B1 = self._r_y(theta_B1)[0:3, 0:3]
        v_local = Ry_B1 @ np.array([1.0, 0.0, 0.0])
        vx, vy, vz = v_local
        nx, ny, nz = n_local
        B1_len = float(self.B[1])

        C0v = nx * vx
        A_ = ny * vy + nz * vz
        B_ = nz * vy - ny * vz
        rhs = float(np.dot(n_local, q_local)) / max(B1_len, 1e-12)
        C_ = rhs - C0v

        r = np.hypot(A_, B_)
        if r < 1e-12:
            alpha = 0.0
        else:
            x = np.clip(C_ / r, -1.0, 1.0)
            phi0 = np.arctan2(B_, A_)
            a1 = phi0 + np.arccos(x)
            a2 = phi0 - np.arccos(x)
            alpha = a1 if abs(a1) < abs(a2) else a2

        c, s = np.cos(alpha), np.sin(alpha)
        Rx_local = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0,  c,  -s],
                [0.0,  s,   c],
            ]
        )
        T_N2_corr = T_N2.copy()
        T_N2_corr[0:3, 0:3] = R0 @ Rx_local

        C0_L = self._law_of_cos(self.B[0], L2, self.C[0])
        C1_L = self._law_of_cos(self.B[1], L2, self.C[1])
        C2_L = self._law_of_cos(self.B[2], L2, self.C[2])

        skew_D0 = self.C[0] * np.cos(C0_L) - self.C[1] * np.cos(C1_L)
        delta_D0 = np.tan(skew_D0 / self.D[0])
        skew_D1 = self.C[2] * np.cos(C2_L) - self.C[1] * np.cos(C1_L)
        delta_D1 = np.tan(skew_D1 / self.D[1])

        phi_B1 = 0.0
        phi_B0 = self._law_of_cos(
            self.D[0] * np.cos(delta_D0),
            self.B[0] * np.sin(theta_B0),
            self.B[1] * np.sin(theta_B1),
        )
        phi_B2 = -self._law_of_cos(
            self.D[1] * np.cos(delta_D1),
            self.B[1] * np.sin(theta_B1),
            self.B[2] * np.sin(theta_B2),
        )

        T_N3 = T_N2_corr @ self._r_x(phi_B0) @ self._r_y(theta_B0) @ self._t_xyz(self.B[0], 0, 0)
        T_N4 = T_N2_corr @ self._r_x(phi_B1) @ self._r_y(theta_B1) @ self._t_xyz(self.B[1], 0, 0)
        T_N5 = T_N2_corr @ self._r_x(phi_B2) @ self._r_y(theta_B2) @ self._t_xyz(self.B[2], 0, 0)
        N3, N4, N5 = T_N3[0:3, 3], T_N4[0:3, 3], T_N5[0:3, 3]

        N6 = self._trilaterate(N3, N5, N4, self.B[3], self.B[5], self.B[4])
        return float(N6[0]), float(N6[1]), float(N6[2])

    # -------------------- inverse kinematics --------------------

    def inverse_kinematics(self, x: float, y: float) -> KinematicsResult:
        """Map workspace (x, y) to joint angles (theta_deg, phi_deg)."""
        A, Lc = float(self.A0), float(self.L_char)

        R = np.hypot(x, Lc)
        Az = np.arctan2(x, Lc)
        El = np.arctan2(y, R)
        theta = np.arctan2(np.hypot(x, y), Lc)

        if theta > self.theta_max:
            return KinematicsResult(
                ok=False,
                theta_deg=float("nan"),
                phi_deg=float("nan"),
                reason=f"theta ({np.rad2deg(theta):.2f} deg) exceeds theta_max ({np.rad2deg(self.theta_max):.2f} deg)",
            )

        OB = 2.0 * A * abs(np.cos(theta))
        cosEl, sinEl = np.cos(El), np.sin(El)
        cosAz, sinAz = np.cos(Az), np.sin(Az)
        xb = OB * cosEl * sinAz
        yb = OB * sinEl
        zb = OB * cosEl * cosAz

        arg = (abs(zb) - self.A0) / self.A0
        if not -1.0 <= arg <= 1.0:
            return KinematicsResult(
                ok=False,
                theta_deg=float("nan"),
                phi_deg=float("nan"),
                reason=f"target outside reachable surface (arccos arg = {arg:.3f})",
            )

        theta_cal = float(np.arccos(arg))
        phi_cal = float(np.arctan2(xb, yb))
        return KinematicsResult(
            ok=True,
            theta_deg=float(np.rad2deg(theta_cal)),
            phi_deg=float(np.rad2deg(phi_cal)),
        )
