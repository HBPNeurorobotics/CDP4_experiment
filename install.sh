#!/bin/bash

# system dependencies:
# apt-get install python-pip python-dev python-virtualenv protobuf-compiler python-pil python-lxml python-dev git

if [[ $# -ne 0 && $1 == "gpu" ]]; then
  echo "going to download gpu versions of models"
  gpu=1
else
  echo "going to download cpu versions of models"
  gpu=0
fi

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

# download saliency model
cd ../model/
if [ $gpu ]; then
  curl -k -o model.ckpt.meta "https://neurorobotics-files.net/owncloud/index.php/s/hdjl7TjzSUqF1Ww/download"
  curl -k -o model.ckpt.index "https://neurorobotics-files.net/owncloud/index.php/s/DCPB80foqkteuC4/download"
  curl -k -o model.ckpt.data-00000-of-00001 "https://neurorobotics-files.net/owncloud/index.php/s/bkpmmvrVkeELapr/download"
else
  curl -k -o model.ckpt.meta "https://neurorobotics-files.net/owncloud/index.php/s/TNpWFSX8xLvfbYD/download"
  curl -k -o model.ckpt.index "https://neurorobotics-files.net/owncloud/index.php/s/sDCFUGTrzJyhDA5/download"
  curl -k -o model.ckpt.data-00000-of-00001 "https://neurorobotics-files.net/owncloud/index.php/s/Scti429S7D11tMv/download"
fi

cd $HBP/GazeboRosPackages/src

# install memory model
if cd holographic; then
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/holographic.git
fi

cd holographic/vsa
python setup.py install --user

cd $HBP/Models/

# install world and poster models
if cd CDP4_models; then
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/CDP4_models.git
fi

cd CDP4_models
./install.sh

cd $HBP/GazeboRosPackages/
catkin_make

# install tensorflow
cd $HOME/.opt
virtualenv --system-site-packages tensorflow_venv
source tensorflow_venv/bin/activate
if [ $gpu ]; then
  pip install --upgrade tensorflow-gpu
else
  pip install --upgrade tensorflow
fi
deactivate

# install object detection models
if cd models; then
  git pull origin master
  cd ..
else
  git clone https://github.com/tensorflow/models
fi

cd models/research
protoc object_detection/protos/*.proto --python_out=.

# install pretrained models
mkdir -p $HOME/.opt/graph_def
cd /tmp
curl -OL http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz
tar -xzf faster_rcnn_resnet101_coco_11_06_2017.tar.gz faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb
cp -a faster_rcnn_resnet101_coco_11_06_2017 $HOME/.opt/graph_def/
ln -sf $HOME/.opt/graph_def/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb $HOME/.opt/graph_def/frozen_inference_graph.pb

cd $DIR
