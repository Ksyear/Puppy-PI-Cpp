"""Import path setup shared by the tests (robot/ and tools/ are not packages)."""

import os
import sys

PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ROBOT_DIR = os.path.join(PACKAGE_DIR, "robot")
TOOLS_DIR = os.path.join(PACKAGE_DIR, "tools")
CONFIG_PATH = os.path.join(PACKAGE_DIR, "config", "robot_config.yaml")

for path in (ROBOT_DIR, TOOLS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

CLIENT_ID = "d8b27f10-09a8-4c21-b649-1e7058d46b72"
OTHER_ID = "5f0c1f4e-2b7a-4f7e-9d0c-3b1a2c4d5e6f"
