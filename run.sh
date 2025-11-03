#!/bin/bash


rm -rf .obj-x86_64-linux-gnu/ debian

source /opt/ros/one/setup.bash
source /opt/ros/jazzy/setup.bash
bloom-generate rosdebian --ros-distro jazzy
echo "3.0 (native)" > debian/source/format
debuild -us -uc


