# Basic ros2 bridge for stretch_mujoco (Kavraki Lab)

Credit to:
[Robocasa](https://github.com/robocasa/robocasa),
[Stretch_Mujoco](https://github.com/hello-robot/stretch_mujoco)


```
mkdir KAVRAKI_LAB
cd KAVRAKI_LAB
mkdir stretch_ws
cd stretch_ws
mkdir src
cd src
git clone <this repo>
```

# Installing simulator
klab is the conda env name. If you have a fork of stretch_mujoco that you plan to edit, "git clone https://github.com/<your_fork_name>/stretch_mujoco instead" of "git clone https://github.com/hello-robot/stretch_mujoco"

```
conda create -c conda-forge -n klab python=3.10
conda activate klab
cd <KAVRAKI_LAB>
git clone https://github.com/hello-robot/stretch_mujoco
cd stretch_mujoco
pip install -e .
conda install mujoco
cd ..
git clone https://github.com/ARISE-Initiative/robosuite
cd robosuite
pip install -e .
cd ..
git clone https://github.com/robocasa/robocasa
cd robocasa
pip install -e .
conda install -c numba numba=0.56
python robocasa/scripts/download_kitchen_assets.py  
python robocasa/scripts/setup_macros.py   
```

# Getting ros2 to work with simulator (bridge_node)
```
conda activate klab

source /opt/ros/humble/setup.bash
cd stretch_ws
colcon build
source install/setup.bash
ros2 run stretch_ros2_sim bridge_node
```

Go to another shell
```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
