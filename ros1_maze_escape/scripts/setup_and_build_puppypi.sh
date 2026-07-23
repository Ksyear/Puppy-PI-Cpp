#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
CATKIN_WORKSPACE="/home/pi/puppypi_maze_ws"
VENDOR_SETUP="/home/pi/puppy_pi/devel/setup.bash"
NOETIC_SETUP="/opt/ros/noetic/setup.bash"
PACKAGE_LINK="${CATKIN_WORKSPACE}/src/ros1_maze_escape"

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

test -f "${PACKAGE_DIR}/package.xml" ||
  fail "package.xml not found in ${PACKAGE_DIR}"
test -f "${PACKAGE_DIR}/CMakeLists.txt" ||
  fail "CMakeLists.txt not found in ${PACKAGE_DIR}"
test -f "${NOETIC_SETUP}" ||
  fail "ROS Noetic setup not found: ${NOETIC_SETUP}"
test -f "${VENDOR_SETUP}" ||
  fail "PuppyPi ROS1 workspace setup not found: ${VENDOR_SETUP}"

source "${NOETIC_SETUP}"
source "${VENDOR_SETUP}"

test "${ROS_DISTRO:-}" = "noetic" ||
  fail "Expected ROS_DISTRO=noetic, got ${ROS_DISTRO:-unset}"
command -v catkin_make >/dev/null 2>&1 ||
  fail "catkin_make is not installed or not on PATH"

PUPPY_CONTROL_PATH="$(rospack find puppy_control 2>/dev/null)" ||
  fail "ROS1 package puppy_control is not visible after sourcing ${VENDOR_SETUP}"

EXPECTED_VELOCITY_MESSAGE=$'float32 x\nfloat32 y\nfloat32 yaw_rate'
ACTUAL_VELOCITY_MESSAGE="$(rosmsg show puppy_control/Velocity 2>/dev/null)" ||
  fail "Cannot load puppy_control/Velocity"
test "${ACTUAL_VELOCITY_MESSAGE}" = "${EXPECTED_VELOCITY_MESSAGE}" ||
  fail "puppy_control/Velocity does not contain exactly float32 x, y, yaw_rate"

mkdir -p "${CATKIN_WORKSPACE}/src"

if test -L "${PACKAGE_LINK}"; then
  LINK_TARGET="$(readlink -f "${PACKAGE_LINK}")"
  test "${LINK_TARGET}" = "${PACKAGE_DIR}" ||
    fail "${PACKAGE_LINK} already points to ${LINK_TARGET}; refusing to replace it"
elif test -e "${PACKAGE_LINK}"; then
  fail "${PACKAGE_LINK} already exists and is not a symbolic link"
else
  ln -s "${PACKAGE_DIR}" "${PACKAGE_LINK}"
fi

echo "Package source: ${PACKAGE_DIR}"
echo "Catkin link:    ${PACKAGE_LINK}"
echo "Vendor package: ${PUPPY_CONTROL_PATH}"

cd "${CATKIN_WORKSPACE}"
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests
catkin_test_results --verbose build/test_results

source "${CATKIN_WORKSPACE}/devel/setup.bash"
rospack find ros1_maze_escape >/dev/null

echo "PUPPYPI_BUILD_AND_TEST_OK"
echo "Apply this workspace in the current terminal with:"
echo "source ${CATKIN_WORKSPACE}/devel/setup.bash"
