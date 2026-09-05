"""Pure geometry helpers the bridge node needs, kept separate from it so they can be
tested without ``rclpy`` -- nothing here imports it or anything ROS-specific."""

from __future__ import annotations

import math

__all__ = ["quaternion_to_yaw"]


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """The yaw (rotation about +Z) a ``geometry_msgs/Quaternion`` encodes, in radians.

    Standard formula for a Z-only rotation extracted from a general quaternion; exact
    for the planar case this project's robots are always in (no roll or pitch), which is
    also all :class:`~parkscan_core.types.Pose2D` can represent in the first place.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
