#!/bin/bash


rm -rf .obj-x86_64-linux-gnu/ debian

source /opt/ros/one/setup.bash
source /opt/ros/jazzy/setup.bash
bloom-generate rosdebian --ros-distro jazzy
echo "3.0 (native)" > debian/source/format

# Patch debian/rules to enable parallel build
sed -i 's/dh $@ /dh $@ --parallel --max-parallel=6 /' debian/rules
sed -i 's/dh_auto_build$/CMAKE_BUILD_PARALLEL_LEVEL=6 dh_auto_build -- -j6/' debian/rules

# Set parallel build options for debuild
export DEB_BUILD_OPTIONS="parallel=6"
export CMAKE_BUILD_PARALLEL_LEVEL=6
export MAKEFLAGS="-j6"

debuild -j6 -us -uc


