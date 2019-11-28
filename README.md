# GPS to ENU

It's a program mainly designed for gps transform to enu.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.


### Prerequisites
Find the installation instruction on [ros](https://www.ros.org/).

#### Eigen3
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 3.3.1 or later required, 3.3.4 recommended

#### Gflags
[Gflags](https://github.com/gflags/gflags) apt install recommended


### Installing
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/jianglingxin/gps-to-enu.git
cd ../..
catkin_make
```

### Running

u should change the topic name of gps topic in gps_to_odom.cpp
