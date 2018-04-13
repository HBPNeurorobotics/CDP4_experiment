#!/bin/bash

# system dependencies:
# apt-get install python-pip python-dev python-virtualenv protobuf-compiler python-pil python-lxml python-dev git

DIR=$PWD

source $VIRTUAL_ENV/bin/activate

cd `dirname $0`
pip install -r requirements.txt

cd $HBP/GazeboRosPackages/src

# install embodied attention model
if cd embodied_attention; then
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/embodied_attention.git
fi

cd embodied_attention/attention
python setup.py install --user
cd ../..

# install memory model
if cd holographic; then
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/holographic.git
fi

cd holographic/vsa
python setup.py install --user

cd $HBP/GazeboRosPackages/
catkin_make

# install tensorflow
cd $HOME/.opt
virtualenv --system-site-packages tensorflow_venv
source tensorflow_venv/bin/activate
pip install --upgrade tensorflow # or pip install --upgrade tensorflow-gpu

# install models
if cd models; then
  git pull origin master
  cd ..
else
  git clone https://github.com/tensorflow/models
fi

cd models/research
protoc object_detection/protos/*.proto --python_out=.
cd ../..

# install pretrained models
mkdir -p $HOME/.opt/graph_def
cd /tmp
curl -OL http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz
tar -xzf faster_rcnn_resnet101_coco_11_06_2017.tar.gz faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb
cp -a faster_rcnn_resnet101_coco_11_06_2017 $HOME/.opt/graph_def/
ln -sf $HOME/.opt/graph_def/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb $HOME/.opt/graph_def/frozen_inference_graph.pb

cd $DIR
