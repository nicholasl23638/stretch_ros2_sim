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
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

# Installing simulator
If you have a fork of stretch_mujoco that you plan to edit, "git clone https://github.com/<your_fork_name>/stretch_mujoco instead" of "git clone https://github.com/hello-robot/stretch_mujoco"

```
cd <KAVRAKI LAB>/stretch_ws
virtualenv -p python3 ./venv
source ./venv/bin/activate
cd ..
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
pip install -c numba==0.56
python robocasa/scripts/download_kitchen_assets.py  
python robocasa/scripts/setup_macros.py   
```

# Getting ros2 to work with simulator (bridge_node)
```
cd stretch_ws
source ./venv/bin/activate
source /opt/ros/humble/setup.bash
cd stretch_ws
colcon build
source install/setup.bash
ros2 run stretch_ros2_sim bridge_node
```

Go to another shell
```
source /opt/ros/humble/setup.bash
ros2 launch stretch_ros2_sim stretch_gamepad.launch.yaml
```
