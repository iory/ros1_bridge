#!/bin/bash

# Get number of parallel jobs from argument, default to 6
JOBS=${1:-6}

echo "Building with ${JOBS} parallel jobs"

rm -rf .obj-x86_64-linux-gnu/ debian

source /opt/ros/one/setup.bash
source /opt/ros/jazzy/setup.bash
bloom-generate rosdebian --ros-distro jazzy
echo "3.0 (native)" > debian/source/format

# Patch debian/rules to enable parallel build
sed -i "s/dh \$@ /dh \$@ --parallel --max-parallel=${JOBS} /" debian/rules
sed -i "s/dh_auto_build$/CMAKE_BUILD_PARALLEL_LEVEL=${JOBS} dh_auto_build -- -j${JOBS}/" debian/rules

# Set parallel build options for debuild
# nocheck: Skip running tests during package build
export DEB_BUILD_OPTIONS="parallel=${JOBS} nocheck"
export CMAKE_BUILD_PARALLEL_LEVEL=${JOBS}
export MAKEFLAGS="-j${JOBS}"

debuild -j${JOBS} -us -uc


