#!/bin/bash

HOME_DIR=$(pwd)

if [ $# -eq 2 ]
then
    BUCKET_NAME="$1"
    DEPLOYMENT_ROLE_ARN="$2"
    echo "Bucket name is " $BUCKET_NAME
    echo "Deployment role ARN is " $DEPLOYMENT_ROLE_ARN
else
    echo "Please run this script with two arguments"
    echo "./IDE_setup.sh BUCKET_NAME DEPLOYMENT_ROLE_ARN"
    exit 1
fi

FIRST_RUN=0
if [ ! -d robot_ws/log ]
then
    # Script first run
    FIRST_RUN=1
    while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        echo "wait for other apt instances to finish"
        sleep 1
    done
    
    # Need to make configurations
    sudo apt update
    sudo apt upgrade -y
    sudo -H pip install -U boto3
    sudo -H pip3 install -U colcon-bundle
    aws greengrass associate-service-role-to-account --role-arn $DEPLOYMENT_ROLE_ARN
fi

# prepare docker for armhf compilation
if [[ $(docker ps -q) ]]
then
    EXISTING_CONTAINER_ID=$(docker ps -q)
    echo "Use currently running container: $EXISTING_CONTAINER_ID"
elif [[ $(docker ps -q -a) ]]
then
    EXISTING_CONTAINER_ID=$(docker ps -q -a -l)
    echo "Restart existing container: $EXISTING_CONTAINER_ID"
    docker start $EXISTING_CONTAINER_ID
else
    echo "Create new container"
    cd /opt/robomaker/cross-compilation-dockerfile/
    sudo bin/build_image.bash
    cd $HOME_DIR
    EXISTING_CONTAINER_ID=$(sudo docker run --name cloudy-watcher -v $HOME_DIR:/ws -dt ros-cross-compile:armhf)
fi

if [ $FIRST_RUN -eq 1 ]
then
    echo "Configure container for build process"
    docker exec $EXISTING_CONTAINER_ID apt update
    docker exec $EXISTING_CONTAINER_ID apt upgrade -y
    docker exec $EXISTING_CONTAINER_ID pip3 install -U colcon-bundle
    docker exec $EXISTING_CONTAINER_ID bash -c 'echo "deb [arch=armhf] http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'
    docker exec $EXISTING_CONTAINER_ID apt update
    docker exec $EXISTING_CONTAINER_ID apt install ros-dashing-ros-base
fi

# start build docker
cd $HOME_DIR
echo "Cross compile started with ID: " $EXISTING_CONTAINER_ID
docker exec $EXISTING_CONTAINER_ID ws/rosbot_deploy/armhf.bash

aws s3 cp $HOME_DIR/../robot_ws/armhf_bundle/output.tar s3://$BUCKET_NAME/cloudwatchsample/robot_ws/bundle/output.armhf.tar
python deploy.py  $BUCKET_NAME
