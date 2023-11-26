# sentry_sim
The simulation of  season in RoboMaster 2023 season based on gazebo

<div align="center"><img src="img/slope_img.png" width=90% /></div>
<div align="center">uphill</div>
<br>

<div align="center"><img src="img/sim_img.png" width=90% /></div>
<div align="center">navigation in bumopy road</div>
<br>

## Environment Setting
- CPU:4800h
- GPU 2060
- RAM: 16G
- OS: Ubuntu 20.04
- ROS: noetic

Some files in `autonomous_exploration_development_environment` package are modified, therefore, I recommend that use `noetic` as your ROS version and clone from my repo directly. Because I added collison attribute for the robot and used different controller, which is different from the original version of the [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment).

```SHELL
sudo apt-get install xterm
sudo apt install libusb-dev

cd ~
mkdir ws_sentry_sim
cd ws_sentry_sim
mkdir src
cd src
git clone git@github.com:66Lau/sentry_sim.git
cd ..
catkin_make
```

## Launch Simulation
```SHELL
cd ws_sentry_sim
source devel/setup.bash
roslaunch sentry_gazebo startup_rmuc.launch
```

https://github.com/66Lau/sentry_sim/assets/95697190/a8513286-9576-4109-98dd-e6898c791bb9
