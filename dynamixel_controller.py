"""Backward-compatible wrapper around the packaged hardware interface."""
from __future__ import annotations

from typing import Optional, Tuple

from ostrich_neck.hardware import OstrichNeckHardwareInterface


_INTERFACE: Optional[OstrichNeckHardwareInterface] = None


def _ensure_interface() -> OstrichNeckHardwareInterface:
    global _INTERFACE
    if _INTERFACE is None:
        _INTERFACE = OstrichNeckHardwareInterface()
        _INTERFACE.connect(move_home=True)
    return _INTERFACE


def move(yaw: int, pitch: int) -> None:
    """Move motors to given yaw and pitch encoder positions (0-4095)."""
    neck = _ensure_interface()
    neck.move_to_encoder_steps(yaw, pitch)


def move_from_angles(yaw_pitch_radians: Tuple[float, float]) -> Tuple[int, int]:
    """Convert ``(yaw, pitch)`` radians to encoder steps and send the command."""
    neck = _ensure_interface()
    return neck.move_from_angles(yaw_pitch_radians)


def cleanup() -> None:
    """Disable torque and close the port safely."""
    global _INTERFACE
    if _INTERFACE is None:
        return
    _INTERFACE.shutdown()
    _INTERFACE = None
