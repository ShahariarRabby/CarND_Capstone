This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

The main goal for this project was to program a car to drive through a controlled course which included traffic lights. The car was to be able to drive through the track, while stopping at the appropriate lights. We were able to test the code through a simulator. 

## Team Members
The members of team:

| Name                          | GitHub account                                     | Udacity Email                 |
|:------------------------------|:---------------------------------------------------|-------------------------------|
| Mohamed Elhayany (Team Lead)  | [Melhaya](https://github.com/Melhaya)              | mohamed.osama211@gmail.com    |
| Dixon Liang                   | [dixonliang](https://github.com/dixonliang)        | liang.dixon@gmail.com         |
| Shahariar Rabby               | [ShahariarRabby](https://github.com/ShahariarRabby)| shahariarrabby@gmail.com     |


[image1]: ./imgs/final-project-ros-graph-v2.png "Carla's System Architecture"

## System Architecture
Carla's system can be broken down into three main parts:
- The perception subsystem detects traffic lights and obstacles.
- The planning subsystem (node waypoint updater) updates the waypoints and the
associated target velocities.
- The control subsystem actuates the throttle, steering, and brake to navigate
the waypoints with the target velocity.

![alt text][image1]

### Code Implementation

There were three sections to the code that we implemented. 

### Waypoint Update Node

The primarily goal for the waypoint was to










### DBW (Drive-By-Wire) Node

This node is responsible for generating electronic control by publishing the appropriate steering, throttle, and brake commands for the car follwoing the car's waypoints. Primarily for the controller, we have implemented a generic PID. 

The below code is part of /control in twist_contorller.py which is responsible for our throttle changes which controls how the car is at a stop or not by appliyng the brake. 


```shell
        if linear_vel == 0. and current_vel < 0.1:
        	throttle = 0
        	brake    = 700 #N*m to hold the car in place if we are stopped at a light. acceleration ~ 1m/s^2

        elif throttle <.1 and vel_error < 0:
        	throttle = 0
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel)*(self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)*self.wheel_radius # Torque N*m

        return throttle, brake, steering
```


### Traffic Light Detection Node

This node consisted of a detector and a classifier. The main purpose of this node is to detect any traffic light and then classify the light to give direction for the car. 

We implemented the detector by updating two functions /process_traffic_lights and /get_closest_waypoint. Effectively,  we use the vehcile's location to feed in the coordinates for the closest traffic light coordinates using these two functions. 

For the classifier, we decided to use an OpenCV approach. Below is the main piece of code as we decided on a range of colors for red on the traffic light, if red pixels are detected after filtering, we classify the light as red which then directs the car to stop. On this implementation, anything that is not red, ends up being Unknown which allows the car to keep moving forward. 


```shell
        # Threshold the HSV image, keep only the red pixels
        lower_red_hue_range = cv2.inRange(hsv_image, (0, 100, 100), (10, 255, 255))
        upper_red_hue_range = cv2.inRange(hsv_image, (160, 100, 100), (179, 255, 255))
        mask = cv2.bitwise_or(lower_red_hue_range, upper_red_hue_range)

        num_of_red_pixels = cv2.countNonZero(mask)

        if num_of_red_pixels >= 200:
            return TrafficLight.RED
            
        return TrafficLight.UNKNOWN
```

At end of this node, we publish the index for the waypoint for the nearest upcoming red light stop to /traffic_waypoint. 

## Instructions for Build

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
### Common Error

if you get following error message
```
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:76 (find_package):
  Could not find a package configuration file provided by "dbw_mkz_msgs" with
  any of the following names:

    dbw_mkz_msgsConfig.cmake
    dbw_mkz_msgs-config.cmake

  Add the installation prefix of "dbw_mkz_msgs" to CMAKE_PREFIX_PATH or set
  "dbw_mkz_msgs_DIR" to a directory containing one of the above files.  If
  "dbw_mkz_msgs" provides a separate development package or SDK, be sure it
  has been installed.
Call Stack (most recent call first):
  styx/CMakeLists.txt:10 (find_package)


-- Could not find the required component 'dbw_mkz_msgs'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "dbw_mkz_msgs" with
  any of the following names:

    dbw_mkz_msgsConfig.cmake
    dbw_mkz_msgs-config.cmake

  Add the installation prefix of "dbw_mkz_msgs" to CMAKE_PREFIX_PATH or set
  "dbw_mkz_msgs_DIR" to a directory containing one of the above files.  If
  "dbw_mkz_msgs" provides a separate development package or SDK, be sure it
  has been installed.
Call Stack (most recent call first):
  styx/CMakeLists.txt:10 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/workspace/CarND-Capstone/ros/build/CMakeFiles/CMakeOutput.log".
See also "/home/workspace/CarND-Capstone/ros/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
```

run the following commands
```
sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
cd /home/workspace/CarND-Capstone/ros
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
