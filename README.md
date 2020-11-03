# robobee
jetson nano quadrotor


## dependencies
- drone simulator
https://github.com/tahsinkose/sjtu-drone
roslaunch sjtu_drone simple.launch

- ros_pat: https://github.com/poine/pat
- common_simulations https://github.com/poine/common_simulations
- common_control https://github.com/poine/common_control
- common_vision https://github.com/poine/common_control


## Quickstart

### Installing
We assume a ubuntu 20.04 and ros noetic install as per http://wiki.ros.org/Installation/Ubuntu

`$ sudo apt-get install ros-noetic-desktop-full`

Create an overlay_workspace

```
mkdir -p ~/my_workspace/src
cd  ~/my_workspace/src
catkin_init_workspace
```

Download packages:

```
git clone 
```

Build packages in workspace
```
cd ..
catkin_make
```

Add the workspace (do that permanently in bashrc)
```
source ~/my_workspace/devel/setup.bash
```


### Running

roslaunch robobee simulation.launch
