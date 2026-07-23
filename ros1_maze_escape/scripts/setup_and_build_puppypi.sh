#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
CATKIN_WORKSPACE="/home/pi/puppypi_maze_ws"
VENDOR_SETUP="/home/pi/puppy_pi/devel/setup.bash"
NOETIC_SETUP="/opt/ros/noetic/setup.bash"
PACKAGE_LINK="${CATKIN_WORKSPACE}/src/ros1_maze_escape"
LINK_FARM_MARKER="${PACKAGE_LINK}/.ros1_maze_escape_source"

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
  unlink "${PACKAGE_LINK}"
  mkdir "${PACKAGE_LINK}"
elif test -e "${PACKAGE_LINK}"; then
  test -d "${PACKAGE_LINK}" ||
    fail "${PACKAGE_LINK} exists and is not a directory"
  test -f "${LINK_FARM_MARKER}" ||
    fail "${PACKAGE_LINK} exists but was not created by this script"
  test "$(cat "${LINK_FARM_MARKER}")" = "${PACKAGE_DIR}" ||
    fail "${PACKAGE_LINK} belongs to a different package source"
else
  mkdir "${PACKAGE_LINK}"
fi

if ! test -f "${LINK_FARM_MARKER}"; then
  printf '%s\n' "${PACKAGE_DIR}" >"${LINK_FARM_MARKER}"
fi

for PACKAGE_ENTRY in \
  CMakeLists.txt \
  package.xml \
  README.md \
  include \
  src \
  launch \
  config \
  test
do
  SOURCE_ENTRY="${PACKAGE_DIR}/${PACKAGE_ENTRY}"
  LINK_ENTRY="${PACKAGE_LINK}/${PACKAGE_ENTRY}"

  test -e "${SOURCE_ENTRY}" ||
    fail "Required package entry is missing: ${SOURCE_ENTRY}"

  if test -L "${LINK_ENTRY}"; then
    test "$(readlink -f "${LINK_ENTRY}")" = "$(readlink -f "${SOURCE_ENTRY}")" ||
      fail "${LINK_ENTRY} points to an unexpected target"
  elif test -e "${LINK_ENTRY}"; then
    fail "${LINK_ENTRY} exists and is not a symbolic link"
  else
    ln -s "${SOURCE_ENTRY}" "${LINK_ENTRY}"
  fi
done

echo "Package source: ${PACKAGE_DIR}"
echo "Catkin package: ${PACKAGE_LINK}"
echo "Vendor package: ${PUPPY_CONTROL_PATH}"

cd "${CATKIN_WORKSPACE}"
catkin_make \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ros1_maze_escape

for TEST_TARGET in \
  velocity_adapter_test \
  frontier_scorer_test \
  frontier_detector_test
do
  TEST_RESULT="${CATKIN_WORKSPACE}/build/test_results/ros1_maze_escape/gtest-${TEST_TARGET}.xml"
  test -s "${TEST_RESULT}" ||
    fail "Expected test result was not generated: ${TEST_RESULT}"

  TEST_COUNT="$(
    python3 -c \
      'import sys, xml.etree.ElementTree as ET; print(ET.parse(sys.argv[1]).getroot().attrib.get("tests", "0"))' \
      "${TEST_RESULT}"
  )" || fail "Cannot parse test result: ${TEST_RESULT}"
  test "${TEST_COUNT}" -gt 0 ||
    fail "Test target ${TEST_TARGET} executed zero test cases"
done

catkin_test_results --verbose build/test_results

source "${CATKIN_WORKSPACE}/devel/setup.bash"
rospack find ros1_maze_escape >/dev/null

echo "PUPPYPI_BUILD_AND_TEST_OK"
echo "Apply this workspace in the current terminal with:"
echo "source ${CATKIN_WORKSPACE}/devel/setup.bash"
