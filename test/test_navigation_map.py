import os
from pathlib import Path
import runpy
import tempfile
import unittest

import yaml


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "drive_manager.launch.py"
configured_map_yaml = runpy.run_path(str(LAUNCH_FILE))["configured_map_yaml"]


class NavigationMapTests(unittest.TestCase):
    def test_launch_default_follows_the_coordinate_map_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "mission.yaml"
            for map_path in ("~/map_first.yaml", "~/map_second.yaml"):
                config.write_text(yaml.safe_dump({
                    "mission_driver": {"ros__parameters": {
                        "navigation_map_yaml": map_path
                    }}
                }), encoding="utf-8")
                self.assertEqual(
                    configured_map_yaml(config),
                    os.path.abspath(os.path.expanduser(map_path)),
                )

    def test_missing_coordinate_map_cannot_silently_use_an_old_map(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "mission.yaml"
            config.write_text(yaml.safe_dump({
                "mission_driver": {"ros__parameters": {"capture_map_id": "new_map"}}
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "navigation_map_yaml"):
                configured_map_yaml(config)
