# Homework 1 ROS2 Notes


### TO INSTALL PACKAGE FOR ASSIGNMENT 

1. Set up environment variables for ROS.
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Fork your own repository of f23_robotics (using web interface)

3. Clone your fork
<pre>
git clone git@github.com:JWSaye/f24_robotics.git
</pre>

4. Make the package
<pre>
cd f24_robotics
colcon build
</pre>

5. Set up variables to use the package you just created
<pre>
source install/setup.bash
</pre>

### FOR SIMULATION USE
Start webots simulation with connect back to ROS in the virtual machine
<pre>
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py
</pre>

### FOR TURTLEBOT3 USE
Start the turtlebot bringup script
<pre>
ros2 launch turtlebot3_bringup robot.launch.py
</pre>

### RUN CONTROLLER
In a new terminal, move into the f24_robotics directory and source the install script again.

Next, run the controller code:
<pre>
ros2 run webots_ros2_homework1_python webots_ros2_homework1_python
</pre>

At the end of the run, the log of AprilTag detections will be in the file:
<pre>
  ~/f24_robotics/apriltag_detections.log
</pre>
