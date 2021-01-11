# ROS2 dashing 설치 및 turtlebot3 setup

프로젝트를 시작하기에 앞서 ROS2 설치와 간단한 setup을 수행합니다.

해당 과정은 

[turtlebot3 e-manual]: https:/emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup

에서 확인할 수 있으며, Ubuntu 18.04 LTS 버전에서 실행되었습니다.



**GPG Key 설치**

```
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo sh -C 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu 'lsb-release -cs' main" > /etc/apt/sources.list.d/ros2-latest.list'
```

**ROS2 패키지 설치**

```
$ sudo apt update
$ sudo apt install ros-dashing desktop
$ sudo apt install ros-dashing-ros-base
$ source /opt/ros/dashing/setup.bash
```

ROS2 패키지 설치가 완료되면 ROS2에서 사용하게 될 Tool의 의존성 패키지를 Install 합니다.

**Install Colcon**

```
$ sudo apt install python3-colcon-commo-extensions
```

**Install Gazebo9**

```
$ curl -sSL http://get.gazebosim.org | sh
### Gazebo11이 미리 설치되어 있다면 삭제합니다.
$ sudo apt remove gazebo11 libgazebo11-dev
$ sudo apt install gazebo9 libgazebo9-dev
$ sudo apt install ros-dashing-gazebo-ros-pkgs
```

**Install Cartographer**

```
$ sudo apt install ros-dashing-cartographer
$ sudo apt install ros-dashing-cartographer-ros
```

**Install Navigation**

```
$ sudo apt install ros-dashing-navigation2
$ sudo apt install ros-dashing-nav2-bringup
```

**Install vcstool**

````
$ sudo apt install python3.vcstool
````



Turtlebot3 패키지를 설치합니다.

만약 Git이 설치되어 있지 않다면 먼저 Install 해줍니다.

**Install Git**

```
$ sudo apt install git
```

**Install Turtlebot3 Packages**

```
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
$ vcs import src < turtlebot3.repos
$ colcon build --symlink-install
```

**환경변수 설정**

```
$ echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc
```

