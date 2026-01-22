#!/usr/bin/env bash
#
# This script builds the jetbot_ros docker container from source.
# It should be run from the root dir of the jetbot_ros project:
#
#     $ cd /path/to/your/jetbot_ros
#     $ docker/build.sh
#
ROS_DISTRO=${1:-"jazzy"}
BASE_IMAGE="osrf/ros:jazzy-desktop"
TAG="ubuntu24.04"

# break on errors
set -e


build_container()
{
	local container_image=$1
	local dockerfile=$2
	
	echo "building $container_image"
	echo "BASE_IMAGE=$BASE_IMAGE"

	sudo docker buildx build --platform linux/arm64 --load -t $container_image -f $dockerfile \
			--build-arg BASE_IMAGE=$BASE_IMAGE \
			.
}

build_container "jetbot_ros:$ROS_DISTRO-$TAG" "Dockerfile"
