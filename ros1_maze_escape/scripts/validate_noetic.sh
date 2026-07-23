#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${ROS1_MAZE_VALIDATION_IMAGE:-ros1-maze-escape:noetic-validation}"

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

command -v docker >/dev/null 2>&1 ||
  fail "Docker command is not available."
docker info >/dev/null 2>&1 ||
  fail "Docker daemon is not available. Start Docker Desktop and retry."

test -f "${PROJECT_ROOT}/package.xml" ||
  fail "package.xml not found at ${PROJECT_ROOT}"
test -f "${PROJECT_ROOT}/docker/Dockerfile.noetic" ||
  fail "Dockerfile.noetic is missing."

echo "[1/2] Building ROS Noetic validation image: ${IMAGE_NAME}"
docker build \
  --file "${PROJECT_ROOT}/docker/Dockerfile.noetic" \
  --tag "${IMAGE_NAME}" \
  "${PROJECT_ROOT}/docker"

echo "[2/2] Building and testing in an isolated catkin workspace"
docker run --rm \
  --network none \
  --mount "type=bind,src=${PROJECT_ROOT},dst=/package_src,readonly" \
  "${IMAGE_NAME}" \
  bash -lc '
set -Eeuo pipefail

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

WORKSPACE=/tmp/noetic_validation_ws
mkdir -p "${WORKSPACE}/src"
cp -a /package_src "${WORKSPACE}/src/ros1_maze_escape"
# catkin_pkg treats COLCON_IGNORE as an ignore marker too. Remove it only from
# this disposable copy so the real package keeps its required colcon marker.
rm -f "${WORKSPACE}/src/ros1_maze_escape/COLCON_IGNORE"

# Validation-only message package. It exists only in this disposable container
# workspace and is not a replacement for the PuppyPi vendor package.
mkdir -p "${WORKSPACE}/src/puppy_control/msg"
cat > "${WORKSPACE}/src/puppy_control/msg/Velocity.msg" <<'"'"'EOF_VELOCITY'"'"'
float32 x
float32 y
float32 yaw_rate
EOF_VELOCITY

cat > "${WORKSPACE}/src/puppy_control/CMakeLists.txt" <<'"'"'EOF_CMAKE'"'"'
cmake_minimum_required(VERSION 3.0.2)
project(puppy_control)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES Velocity.msg)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
EOF_CMAKE

cat > "${WORKSPACE}/src/puppy_control/package.xml" <<'"'"'EOF_PACKAGE'"'"'
<?xml version="1.0"?>
<package format="2">
  <name>puppy_control</name>
  <version>0.0.0</version>
  <description>Validation-only Velocity message stub.</description>
  <maintainer email="validation@example.com">Validation Only</maintainer>
  <license>UNLICENSED</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
EOF_PACKAGE

cd "${WORKSPACE}"
catkin_make \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ros1_maze_escape

for TEST_TARGET in \
  velocity_adapter_test \
  frontier_scorer_test \
  frontier_detector_test
do
  TEST_RESULT="${WORKSPACE}/build/test_results/ros1_maze_escape/gtest-${TEST_TARGET}.xml"
  test -s "${TEST_RESULT}" || {
    echo "ERROR: expected test result was not generated: ${TEST_RESULT}" >&2
    exit 1
  }

  TEST_COUNT="$(
    python3 -c \
      "import sys, xml.etree.ElementTree as ET; print(ET.parse(sys.argv[1]).getroot().attrib.get(\"tests\", \"0\"))" \
      "${TEST_RESULT}"
  )"
  test "${TEST_COUNT}" -gt 0 || {
    echo "ERROR: test target ${TEST_TARGET} executed zero test cases" >&2
    exit 1
  }
done

catkin_test_results --verbose build/test_results

source "${WORKSPACE}/devel/setup.bash"

for launch_file in "${WORKSPACE}"/src/ros1_maze_escape/launch/*.launch; do
  xmllint --noout "${launch_file}"
done
xmllint --noout "${WORKSPACE}/src/ros1_maze_escape/package.xml"

python3 - <<'"'"'EOF_YAML'"'"'
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

config_dir = Path("/tmp/noetic_validation_ws/src/ros1_maze_escape/config")
for config_file in sorted(config_dir.glob("*.yaml")):
    with config_file.open("r", encoding="utf-8") as stream:
        yaml.safe_load(stream)

common = yaml.safe_load((config_dir / "costmap_common.yaml").read_text())
global_map = yaml.safe_load((config_dir / "global_costmap.yaml").read_text())
local_map = yaml.safe_load((config_dir / "local_costmap.yaml").read_text())
dwa = yaml.safe_load((config_dir / "dwa_local_planner.yaml").read_text())

assert "global_costmap" not in common
assert "local_costmap" not in common
assert global_map["global_costmap"]["global_frame"] == "map"
assert local_map["local_costmap"]["global_frame"] == "odom"
assert local_map["local_costmap"]["rolling_window"] is True
assert "DWAPlannerROS" in dwa
assert dwa["DWAPlannerROS"]["max_vel_y"] == 0.0
assert dwa["DWAPlannerROS"]["min_vel_y"] == 0.0

navigation_launch = ET.parse(
    "/tmp/noetic_validation_ws/src/ros1_maze_escape/launch/navigation.launch"
)
common_load_namespaces = {
    element.attrib.get("ns")
    for element in navigation_launch.findall(".//rosparam")
    if element.attrib.get("file", "").endswith("config/costmap_common.yaml")
}
assert common_load_namespaces == {"global_costmap", "local_costmap"}
EOF_YAML

# --files parses substitutions and includes but never starts ROS nodes.
roslaunch --files ros1_maze_escape odometry.launch use_rf2o:=false
roslaunch --files ros1_maze_escape mapping.launch \
  use_rf2o:=false publish_lidar_tf:=false
roslaunch --files ros1_maze_escape navigation.launch
roslaunch --files ros1_maze_escape explore_baseline.launch
roslaunch --files ros1_maze_escape maze_escape.launch \
  mode:=custom use_rf2o:=false publish_lidar_tf:=false
roslaunch --files ros1_maze_escape maze_escape.launch \
  mode:=baseline use_rf2o:=false publish_lidar_tf:=false

if roslaunch --files ros1_maze_escape maze_escape.launch \
    mode:=invalid use_rf2o:=false publish_lidar_tf:=false \
    >/tmp/invalid_mode.log 2>&1; then
  cat /tmp/invalid_mode.log >&2
  echo "ERROR: invalid mode was accepted" >&2
  exit 1
fi

echo "NOETIC_VALIDATION_OK"
'
