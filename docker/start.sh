#!/bin/bash
if [ ! -d f1tenth_gym ] ; then
    git clone --depth=1 https://github.com/f1tenth/f1tenth_gym ## use depth 1 due to low speed
else
    echo f1tenth_gym exists, not cloning, pulling in latest updates.
    cd f1tenth_gym
    git pull
    cd ..
fi

if [ ! -d f1tenth_gym_ros ] ; then
    git clone https://github.com/f1tenth/f1tenth_gym_ros
    cd f1tenth_gym_ros
    git checkout docker_agent
    cd ..
else
    echo f1tenth_gym_ros exists, not cloning, pulling in latest updates.
    cd f1tenth_gym_ros
    git checkout docker_agent
    git pull
    cd ..
fi

docker-compose up --build