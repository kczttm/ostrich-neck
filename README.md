# Ostrich Neck Hardware Interface

Local Python package that exposes a small API for controlling the 2-DOF "ostrich" neck powered by Dynamixel actuators. Install in editable mode while developing:

```bash
pip install -e /path/to/ostrich-neck
```

The package provides:

- `ostrich_neck.conversion.convert_neck_input_to_encoder_steps`: convert `(yaw, pitch)` radians to encoder counts.
- `ostrich_neck.hardware.OstrichNeckHardwareInterface`: high-level hardware controller with safe connect / shutdown helpers.

The legacy `keyboard_control.py` script now uses the packaged interface but remains available for manual tests.
