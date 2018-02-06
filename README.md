CDP4 experiment on the NRP
====================

This repository contains the highlight CDP4 experiment on the NRP.
Here is a block diagram of the components that will be integrated in this experiment for March 2018:

![CDP4 experiment](img/experiment.png "Components of CDP4 experiment")


Installation
-----------

* Clone this folder into the `NRP/Experiments/` (`$NRP_EXPERIMENTS_DIRECTORY`) folder

Run `./install.sh` or follow these instructions:

* The following repos are needed in the `GazeboRosPackages/src/` (`$HBP/GazeboRosPackages/src`) folder:
  * [embodied_attention](https://github.com/HBPNeurorobotics/embodied_attention)
    * `git clone git@github.com:HBPNeurorobotics/embodied_attention.git`
  * **Optional** memory - [holographic](https://github.com/HBPNeurorobotics/holographic)
    * `git clone git@github.com:HBPNeurorobotics/holographic.git`
* Run `catkin_make` in `GazeboRosPackages/`
* **Optional** object identification - Install [slim](https://github.com/tensorflow/models/tree/master/research/slim)
  * `git clone git@github.com:tensorflow/models.git`
  * `cd models/research/slim`
  * `python setup.py install`

The following libraries should be installed into your platform virtual environment (``~/.opt/platform_venv``):
* tensorflow>=1.4.1
* numpy>=1.13.3
* scikit-image
* wget (used to download the network model files on first run)

Usage
-----

See nrp.launch for the different nodes provided:
* embodied_attention forward.py - converts camera input to saliency map
* embodied_attention saccade.py - converts saliency map to saccade targets
* embodied_attention attention.py - moves eyes of robot, initiates object identification, stores information in memory
* embodied_attention recognize.py - provides object recognition
* ros_holographic visual_memory_module.py - provides memory

Additionally, you can `rosrun embodied_attention visualizer.py` for a brief visualization of the saccade mechanism
