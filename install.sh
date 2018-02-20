#!/bin/bash

DIR=$PWD

source $VIRTUAL_ENV/bin/activate

cd `dirname $0`
pip install -r requirements.txt

cd $HBP/GazeboRosPackages/src

if cd embodied_attention; then
  git pull
  cd ..
else
  git clone git@github.com:HBPNeurorobotics/embodied_attention.git
fi

if cd holographic; then
  git pull
  cd ..
else
  git clone git@github.com:HBPNeurorobotics/holographic.git
fi

cd holographic/vsa
python setup.py install --user
cd ../..

cd ..
catkin_make

cd `mktemp -d`
git clone https://github.com/tensorflow/models
cd models/research/slim
python setup.py install --user

cd $DIR
