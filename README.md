# Learning Forward Kinematics Model For Tendon-Driven Robots 
This repo is about learning forward kinematics model for tendon-driven robots, accounting for hysteresis.

![alt text](figures/intro.jpg?raw=true)


## Build

```
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
cd ~/git/Tendon_Robot_Learning/build && ninja
```

## Configure the environment
```
cd ~/git/Tendon_Robot_Learning/build && roseloquent && export PYTHONPATH=$PYTHONPATH:$PWD
```
## Train a model
```
cd ~/git/Tendon_Robot_Learning/build && python3 learn_actual_pc.py path_to_data_file.pickle path_to_model_weight.pth path_to_test_indices.pickle --hys 1   
```
The positional arguments are a data file name, a weight file name, and a test dataset (consisting of indices as Numpy array). 
To train a simulation model, see `learn_tendon_shape.py`.

## Data collection conversion

The tendon-driven robot data collection is in the form of Python dictionary where each key and value includes tendon configuration, length configuration, servo commands, and the corresponding robot shape (point clouds). To use the data collection in the model training, you need to convert it to Python list or Numpy array. You can easily do so using Python codes described below:
`traj_to_datalist.py` : convert the trajectory dataset into a Python list, and save it as a Pickle file.
`pickle_to_list.py` : convert the nominal dataset that visits the home configuration to a Python list, and save it as a Pickle file

For data collection for simulation, see `simulated_data_collection.py` or `pbf_data_collection.py`.

## Testing the learned model

Hysteresis quantification: `test_hysteresis_config.py`
Physics-based model comparison (Simulation): `test_sim_config.py`
Learned model: `test_config.py`

The learned model trained with 3 nominal sets (i.e., home configuration and 2 random configs in between) : data_fixed_third.pth (weight), data_fixed_third.pickle (dataset). 
The original dataset for 3 nominal sets (Jarvis) : ~/ros2_ws/src/realsense-ros/realsense2_camera/scripts/data/data_storage_fixed_*

## Testing the learned model on trajectory data
```
cd ~/git/Tendon_Robot_Learning/build && python3 test_config.py path_to_model_weight.pth path_to_data_file.pickle path_to_test_indices.pickle -H 1
```
