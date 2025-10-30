
# HW1: Bring up your robot

The goal of this homework is to build ROS packages to simulate a 4-degrees-of-freedom robotic manipulator arm (Armando) into the Gazebo environment.

## Running docker container and connecting other terminals

Since bash scripts have been shared beforehand, the image building process will be skipped.
The first step, before executing the ROS2 scripts, is to run the docker container via the command:
````
 $ ~/ros2_docker_scripts/ros2_run_container.sh <ros2_image_name> <ros2_container_name> <workspace_directory_name>  
````

After that it is possible to connect another terminal to the docker container by running:
````
$ ~/ros2_docker_scripts/docker_connect.sh  
````
and then inputting the number that refers to ```` <ros2_container_name> ````.
It is to be noted that the urdf_lauch and joint_state_publisher_gui packages dependencies are missing from the Docker image provided. These packages can be installed by running:
````
$ sudo apt install ros-${ROS_DISTRO}-urdf-launch ros-${ROS_DISTRO}-joint-state-publisher-gui  
````

## ROS2
### Building and sourcing the environment
Following the creation and running of the docker container, to setup the RO2 workspace it is requested to access the src directory of the workspace(1.), clone the packages of the repository (2.), return to the workspace directory (3.), build the workspace (4.) and source the overlay (5.), as shown in the lines beneath:  
````
1. $ cd <workspace_directory_name>/src 

2. $ git clone https://github.com/DhaTeremin/HW1-repository.git temp && mv temp/* temp/.* . 2>/dev/null && rmdir temp

3. $ cd ..

4. $ colcon build

5. $ source install/setup.bash
````
The second row command can be executed just by one of the terminal connected to the docker container, conversly the first and third row are mandatory for every terminal that connects to correctly use the workspace.

Then the workflow splits between RViz2 and Gazebo. 

### RViz2 startup
For launching the robot and visualize it in RViz2 via launch file, the command is the one that follows:
````
$ ros2 launch armando_description armando_display.launch.py
````
### Gazebo environment startup with controller (position or joint_trajectory)
For launching the robot and starting the Gazebo simulation environment via launch file, the commands are the following, depending on which controller to start: 
#### Start the position controller

````
$ ros2 launch armando_gazebo armando_world.launch.py controller_type:=0
````
#### Start the joint_trajectory controller
````
$ ros2 launch armando_gazebo armando_world.launch.py controller_type:=1
````
### Running the controller node
After starting up the controller, it is required to run the related publisher via the arm_controller_node. The preliminary steps are opening another terminal, connecting it to the docker container and source the overlay, as aforementioned in the ***Building and sourcing environment*** section. The related command are the following: 

````
$ cd <workspace_directory_name> 

$ source install/setup.bash
````
The controller node, then, can be run depending on the publisher selected
#### Running the position controller publisher
````
$ ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=0
````
#### Running the joint_trajectory controller publisher
````
$ ros2 run armando_controller arm_controller_node --ros-args -p controller_type:=1
````
### Running the camera bridge and checking the images
To access the images captured by the camera sensor in the ros2 environment, a bridge between it and Gazebo environment is needed. To run the bridge via the launch file, in which it is defined, the command is:
````
$ ros2 launch armando_gazebo armando_gazebo.launch.py
````
To check the images captured by the camera sensor, the ````rqt_image_view```` tool can be used. To run it the command is:

````
$ ros2 run rqt_image_view rqt_image_view
````
