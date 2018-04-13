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
cd ../..

cd ..
catkin_make

# install tensorflow
cd $HOME/.opt
virtualenv --system-site-packages tensorflow_venv
source tensorflow_venv/bin/activate
pip install --upgrade tensorflow # or pip install --upgrade tensorflow-gpu

# install models
git clone https://github.com/tensorflow/models
cd models/research
protoc object_detection/protos/*.proto --python_out=.
cd ../..

# install pretrained models
mkdir -p $HOME/.opt/graph_def
cd /tmp
for model in \
  ssd_mobilenet_v1_coco_11_06_2017 \
  ssd_inception_v2_coco_11_06_2017 \
  rfcn_resnet101_coco_11_06_2017 \
  faster_rcnn_resnet101_coco_11_06_2017 \
  faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017
do \
  curl -OL http://download.tensorflow.org/models/object_detection/$model.tar.gz
  tar -xzf $model.tar.gz $model/frozen_inference_graph.pb
  cp -a $model $HOME/.opt/graph_def/
done
ln -sf $HOME/.opt/graph_def/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb $HOME/.opt/graph_def/frozen_inference_graph.pb

cd $DIR
