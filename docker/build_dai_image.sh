#!/bin/bash

# Step 1: Clone the depthai-ros repository
git clone https://github.com/luxonis/depthai-ros.git

# Step 2: Change directory to the cloned repository
cd depthai-ros

# Step 3: Build the Docker image with the name 'dai'
docker build -t dai .
