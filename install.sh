#!/bin/bash

# system dependencies:
# apt-get install python-pip python-dev python-virtualenv protobuf-compiler python-pil python-lxml python-dev git

printf "\033[1;33mWould you like to install the GPU version of tensorflow? (Y/n)\033[0m\n"
read tf_gpu
if [ "$tf_gpu" == "Y" -o "$tf_gpu" == "y" ]
then
  echo "going to download gpu versions of models"
  gpu=1
else
  echo "going to download cpu versions of models"
  unset gpu
fi

DIR=$PWD

source $VIRTUAL_ENV/bin/activate

cd `dirname $0`
pip install -r requirements.txt

cd $HBP/GazeboRosPackages/src

# install embodied attention model
if [ -d embodied_attention ]; then
  cd embodied_attention
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/embodied_attention.git
fi

cd embodied_attention/attention

python setup.py install --user

# download saliency model
cd ..
mkdir -p model
if [ `cat model/config` == "gpu" ] && [ $gpu ]; then
  echo "GPU weights already present"
elif [ `cat model/config` == "cpu" ] && [ -z $gpu ]; then
  echo "CPU weights already present"
elif [ $gpu ]; then
  curl -k -o model.ckpt.meta "https://neurorobotics-files.net/owncloud/index.php/s/hdjl7TjzSUqF1Ww/download"
  curl -k -o model.ckpt.index "https://neurorobotics-files.net/owncloud/index.php/s/DCPB80foqkteuC4/download"
  curl -k -o model.ckpt.data-00000-of-00001 "https://neurorobotics-files.net/owncloud/index.php/s/bkpmmvrVkeELapr/download"
  echo "gpu" > config
else
  curl -k -o model.ckpt.meta "https://neurorobotics-files.net/owncloud/index.php/s/TNpWFSX8xLvfbYD/download"
  curl -k -o model.ckpt.index "https://neurorobotics-files.net/owncloud/index.php/s/sDCFUGTrzJyhDA5/download"
  curl -k -o model.ckpt.data-00000-of-00001 "https://neurorobotics-files.net/owncloud/index.php/s/Scti429S7D11tMv/download"
  echo "cpu" > config
fi


cd $HBP/GazeboRosPackages/src

# install memory model
if [ -d holographic ]; then
  cd holographic
  git pull origin master
  cd ..
else
  git clone https://github.com/HBPNeurorobotics/holographic.git
fi

cd holographic/vsa
python setup.py install --user

cd $HBP/Models/

# install world and poster models
if [ -d CDP4_models ]; then
  cd CDP4_models
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
if [ ! -d tensorflow_venv ]
then
  virtualenv --system-site-packages tensorflow_venv
  source tensorflow_venv/bin/activate
  if [ $gpu ]; then
    pip install --upgrade tensorflow-gpu==1.6.0
  else
    pip install --upgrade tensorflow==1.6.0
  fi
  deactivate
fi

# install object detection models
if [ -d models ]; then
  cd models
  git pull origin master
else
  git clone https://github.com/tensorflow/models
  cd models
fi

# for some reason, protoc can not compile the most recent version of the files
git checkout c31b3c2
cd research
protoc object_detection/protos/*.proto --python_out=.

if [ ! -d $HOME/.opt/graph_def ]
then
  # install pretrained models
  mkdir -p $HOME/.opt/graph_def
  cd /tmp
  curl -OL http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz
  tar -xzf faster_rcnn_resnet101_coco_11_06_2017.tar.gz faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb
  cp -a faster_rcnn_resnet101_coco_11_06_2017 $HOME/.opt/graph_def/
  ln -sf $HOME/.opt/graph_def/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb $HOME/.opt/graph_def/frozen_inference_graph.pb
fi

cd $DIR
