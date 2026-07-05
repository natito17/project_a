#!/sim_ws/.venv/bin/python3
"""Compatibility wrapper for local tests and direct script execution.

The actual node implementation lives in gap_follow/reactive_node.py so it is
importable by ROS2 console scripts (`ros2 run gap_follow reactive_node`).
This wrapper keeps existing `python3 scripts/reactive_node.py` and test imports
working.
"""

from gap_follow.reactive_node import *  # noqa: F401,F403


if __name__ == '__main__':
    main()
