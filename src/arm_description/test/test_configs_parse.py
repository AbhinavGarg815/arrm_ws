#!/usr/bin/env python3
"""Every YAML config in this package must actually parse.

This exists because an RViz config with `{X: 0; Y: 0; Z: 0}` -- semicolons
instead of commas -- is invalid YAML, and RViz's response is to load NOTHING
and report nothing. The window opens, it is empty, and there is no error
anywhere to tell you why. Half an hour of "is the robot not being published?"
later, the answer is a punctuation mistake.

Config files are data. Data gets validated.
"""

from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

SHARE = Path(get_package_share_directory("arm_description"))
CONFIGS = sorted(
    [p for p in (SHARE / "config").glob("*.yaml")]
    + [p for p in (SHARE / "rviz").glob("*.rviz")]
)


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_yaml_parses(path):
    with path.open() as fh:
        assert yaml.safe_load(fh) is not None, f"{path.name} parsed to nothing"


def test_rviz_fixed_frame_is_world():
    """A Fixed Frame that does not exist renders an empty window too."""
    for path in (SHARE / "rviz").glob("*.rviz"):
        cfg = yaml.safe_load(path.read_text())
        frame = cfg["Visualization Manager"]["Global Options"]["Fixed Frame"]
        assert frame == "world", (
            f"{path.name} has Fixed Frame '{frame}'; the URDF root is 'world'"
        )
