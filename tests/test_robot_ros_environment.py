import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


V2_DIR = Path(__file__).resolve().parents[1] / "v2"
sys.path.insert(0, str(V2_DIR))

from fragments.robot_fragment import _remove_ros2_python_paths


class RobotRosEnvironmentTest(unittest.TestCase):
    def test_remove_ros2_python_paths_filters_humble_from_pythonpath_and_syspath(self):
        original_sys_path = [
            "/opt/ros/humble/lib/python3.10/site-packages",
            "/home/cisl/anaconda3/envs/ros_env/lib/python3.9/site-packages",
            "/opt/ros/humble/local/lib/python3.10/dist-packages",
        ]
        env = {
            "PYTHONPATH": os.pathsep.join(original_sys_path),
        }

        with patch.object(sys, "path", list(original_sys_path)), patch.dict(os.environ, env, clear=True):
            _remove_ros2_python_paths()

            self.assertEqual(
                sys.path,
                ["/home/cisl/anaconda3/envs/ros_env/lib/python3.9/site-packages"],
            )
            self.assertEqual(
                os.environ["PYTHONPATH"],
                "/home/cisl/anaconda3/envs/ros_env/lib/python3.9/site-packages",
            )


if __name__ == "__main__":
    unittest.main()
