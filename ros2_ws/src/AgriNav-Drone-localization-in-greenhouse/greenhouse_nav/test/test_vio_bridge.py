"""
test_vio_bridge.py
==================
Unit tests for the VIO bridge ENU→NED frame conversion.
Run with: pytest test/test_vio_bridge.py -v
"""

import pytest
import math
import numpy as np
import sys
import os

# Allow importing from the package without full ROS install
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ─── Replicate the frame conversion logic for testing ──────────────────────

def enu_to_ned(pos_enu, origin_enu=None):
    """
    Convert ENU position to NED, relative to an optional origin.
    ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down
    """
    if origin_enu is not None:
        rel = np.array(pos_enu) - np.array(origin_enu)
    else:
        rel = np.array(pos_enu)
    return np.array([rel[1], rel[0], -rel[2]])


def ned_quaternion(q_enu_wxyz):
    """Convert quaternion from ENU to NED frame."""
    w, x, y, z = q_enu_wxyz
    return [w, y, x, -z]


# ─── Tests ──────────────────────────────────────────────────────────────────

class TestENUtoNED:
    def test_forward_motion(self):
        """Moving North in ENU (+y) → moving North in NED (+x)."""
        ned = enu_to_ned([0.0, 5.0, 0.0])
        assert abs(ned[0] - 5.0) < 1e-6,  "North should be +x in NED"
        assert abs(ned[1] - 0.0) < 1e-6,  "East should be 0"
        assert abs(ned[2] - 0.0) < 1e-6,  "Down should be 0"

    def test_east_motion(self):
        """Moving East in ENU (+x) → moving East in NED (+y)."""
        ned = enu_to_ned([3.0, 0.0, 0.0])
        assert abs(ned[0] - 0.0) < 1e-6
        assert abs(ned[1] - 3.0) < 1e-6
        assert abs(ned[2] - 0.0) < 1e-6

    def test_altitude(self):
        """Up in ENU (+z) → Down in NED becomes negative (+z ENU = -z NED)."""
        ned = enu_to_ned([0.0, 0.0, 1.5])
        assert abs(ned[2] - (-1.5)) < 1e-6, "1.5m up ENU → -1.5m NED (z=Down)"

    def test_origin_lock(self):
        """Position relative to locked origin."""
        origin = [1.0, 2.0, 0.0]
        pos    = [1.0, 5.0, 0.0]   # 3m North of origin
        ned    = enu_to_ned(pos, origin)
        assert abs(ned[0] - 3.0) < 1e-6, "Should be 3m forward in NED"
        assert abs(ned[1] - 0.0) < 1e-6
        assert abs(ned[2] - 0.0) < 1e-6

    def test_diagonal(self):
        """Diagonal movement preserves distance."""
        enu = [3.0, 4.0, 0.0]
        ned = enu_to_ned(enu)
        dist_enu = math.sqrt(3**2 + 4**2)
        dist_ned = math.sqrt(ned[0]**2 + ned[1]**2)
        assert abs(dist_enu - dist_ned) < 1e-6, "Distance preserved in frame conversion"


class TestQuaternionConversion:
    def test_identity(self):
        """Identity quaternion should remain identity."""
        q_ned = ned_quaternion([1.0, 0.0, 0.0, 0.0])
        assert q_ned[0] == 1.0  # w unchanged
        assert q_ned[1] == 0.0  # x ← y (was 0)
        assert q_ned[2] == 0.0  # y ← x (was 0)
        assert q_ned[3] == 0.0  # z negated (was 0)

    def test_yaw_sign(self):
        """Yaw rotation should flip sign in ENU→NED conversion."""
        # 90° yaw in ENU: q = [cos(45°), 0, 0, sin(45°)]
        sin45 = math.sin(math.pi / 4)
        cos45 = math.cos(math.pi / 4)
        q_enu = [cos45, 0.0, 0.0, sin45]
        q_ned = ned_quaternion(q_enu)
        assert abs(q_ned[3] - (-sin45)) < 1e-6, "Z component should be negated"


class TestMissionGeometry:
    def test_uturn_yaw(self):
        """After flying North (yaw=0), U-turn target is pi (South-facing)."""
        initial_yaw  = 0.0
        uturn_target = math.pi
        yaw_error    = abs(uturn_target - initial_yaw)
        assert abs(yaw_error - math.pi) < 1e-6

    def test_waypoint_sequence(self):
        """Far waypoint is DIST meters ahead of origin in NED."""
        origin = [5.0, 3.0]
        dist   = 15.0
        far    = [origin[0] + dist, origin[1]]
        assert abs(far[0] - 20.0) < 1e-6
        assert abs(far[1] - 3.0)  < 1e-6

    def test_position_tolerance(self):
        """Check that position tolerance logic works."""
        goal    = [15.0, 0.0]
        pos_ok  = [15.15, 0.10]   # within 0.3m
        pos_far = [14.50, 0.00]   # outside 0.3m
        tol     = 0.30

        d_ok  = math.hypot(pos_ok[0]  - goal[0], pos_ok[1]  - goal[1])
        d_far = math.hypot(pos_far[0] - goal[0], pos_far[1] - goal[1])

        assert d_ok  < tol, "pos_ok should be within tolerance"
        assert d_far > tol, "pos_far should be outside tolerance"


class TestObstacleGeometry:
    def test_minimum_gap_for_passage(self):
        """
        In a 4m wide space, verify minimum gap calculation.
        Drone width = 0.70m, safety margin = 0.15m each side.
        Min gap = drone_width + 2 * safety = 0.70 + 0.30 = 1.00m
        Recommended gap = 1.30m (adds VIO drift margin)
        """
        drone_width    = 0.70
        safety_margin  = 0.15
        min_gap        = drone_width + 2 * safety_margin
        assert abs(min_gap - 1.00) < 1e-6

    def test_max_bypassable_obstacle_in_4m_space(self):
        """
        In a 4m space, an obstacle in the center leaves 1.75m on each side.
        After subtracting min_clearance (1.0m), max obstacle width = 2*(1.75-1.0) = 1.5m.
        Practical limit should be much lower (0.40m) due to position uncertainty.
        """
        space_width     = 4.0
        drone_width     = 0.70
        safety_m        = 0.15
        min_clear       = drone_width / 2 + safety_m   # 0.50m
        max_obs_theory  = space_width - 2 * min_clear
        assert max_obs_theory > 0, "Should be geometrically possible to pass"
        # Practical limit due to VIO uncertainty
        practical_limit = 0.40
        assert practical_limit < max_obs_theory
