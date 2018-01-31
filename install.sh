#!/bin/bash

DIR=$PWD

pip install -r requirements.txt

cd $HBP/GazeboRosPackages/src
git clone git@github.com:HBPNeurorobotics/embodied_attention.git
git clone git@github.com:HBPNeurorobotics/holographic.git
cd holographic/ros_holographic
python setup.py install
cd ../../..
catkin_make

cd /tmp
git clone https://github.com/tensorflow/models
cd models/research/slim
python setup.py install

cd $DIR
