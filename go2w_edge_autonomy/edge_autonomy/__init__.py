from __future__ import annotations

"""GO2W edge autonomy package.

This package is the Python orchestration layer around the C++ SLAM gateway:
it owns map metadata, runtime state parsing, planner context construction,
and the local executor that sends validated JSON commands to the gateway.
"""
