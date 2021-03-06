#!/bin/bash

# Script based on Makefile of 'https://github.com/duckietown/rpi-ros-kinetic-base' repo

# Variables for script
imageArch=amd64
imageTag=ccsu_duckietown/base:master19-$imageArch

echo "Starting image build -- Info: $imageTag : $imageArch."
docker build -t $imageTag --build-arg ARCH=$imageArch .
echo "Finished image build."