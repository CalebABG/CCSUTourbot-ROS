#!/bin/bash

# Variables for script
imageArch=amd64
imageTag=ccsu_duckietown/dt18/rpi-ros-kinetic-base:master19-$imageArch

echo "Starting image build -- Info: $imageTag : $imageArch."
docker build -t $imageTag --build-arg ARCH=$imageArch .
echo "Finished image build."