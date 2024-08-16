# Learning Forward Kinematics Model For Tendon-Driven Robots (ROS2)
This repo is about learning forward kinematics model for tendon-driven robots, accounting for hysteresis.

![alt text](figures/intro.jpg?raw=true "Title")
#### Abstract
Tendon-driven continuum robots have been gaining popularity in medical applications due to their ability to curve around complex anatomical structures, potentially reducing the invasiveness of surgery. However, accurate modeling is required to plan and control the movements of these flexible robots. Physics-based models have limitations due to unmodeled effects, leading to mismatches between model prediction and actual robot shape. Recently proposed learning-based methods have been shown to overcome some of these limitations but do not account for hysteresis, a significant source of error for these robots. To overcome these challenges, we propose a novel deep decoder neural network that predicts the complete shape of tendon-driven robots using point clouds as the shape representation, conditioned on prior configurations to account for hysteresis. We evaluate our method on a physical tendon-driven robot and show that our network model accurately predicts the robot's shape, significantly outperforming a state-of-the-art physics-based model and a learning-based model that does not account for hysteresis.


## Hardware Details
![alt text](figures/setup.jpg?raw=true "Title")
All computation was performed on a computer with an AMD Ryzen 7 3700X 8-core precessor, 64 GB of 3200 Hz DDR4 DRAM, and an NVIDIA GeForce RTX 3060 Ti.

#### Cameras
Model: Intel RealSense D405 (see: https://github.com/IntelRealSense/realsense-ros)

#### Tendon-driven robot
The robot we used in the experiment (shown red in the figure above) consists of a 3D printed, flexible thermoplastic polyurethane (TPU) material body with a thin nitinol tube embedded in the 3D printed structure with a length of 0.2 m, consisting of 9 circular disks that connect 3 straight-routed tendons at 120 degrees apart and 1 helically-routed tendon, with linear actuators pulling on the tendons to control the robot's shape at the robot's base frome.


## Build

```
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
cd ~/git/Tendon_Robot_Learning/build && ninja
```

## Configure the environment
```
cd ~/git/Tendon_Robot_Learning/build && roseloquent && export PYTHONPATH=$PYTHONPATH:$PWD
```
## Training
```
cd ~/git/Tendon_Robot_Learning/build && python3 learn_actual_pc.py path_to_data_file.pickle path_to_model_weight.pth path_to_test_indices.pickle --hys 1   
```
The positional arguments are a data file name, a weight file name, and a test dataset (consisting of indices as Numpy array). 
To train a simulation model, see `learn_tendon_shape.py`.

To get help:
```
python3 learn_actual_pc.py -h
```

## Data collection conversion

The tendon-driven robot data collection is in the form of Python dictionary where each key and value includes tendon configuration, length configuration, servo commands, and the corresponding robot shape (point clouds). To use the data collection in the model training, you need to convert it to Python list or Numpy array. You can easily do so using Python codes described below:
`traj_to_datalist.py` : convert the trajectory dataset into a Python list, and save it as a Pickle file.
`pickle_to_list.py` : convert the nominal dataset that visits the home configuration to a Python list, and save it as a Pickle file

For data collection for simulation, see `simulated_data_collection.py` or `pbf_data_collection.py`.

## Inference

Hysteresis quantification: `test_hysteresis_config.py`_
Physics-based model comparison (Simulation): `test_sim_config.py`_
Learned model: `test_config.py`


