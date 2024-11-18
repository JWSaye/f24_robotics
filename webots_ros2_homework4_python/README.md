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

### Algorithm Explanation
While deciding what the best methodology would be for finding as many AprilTags as possible in a short amount of time, we considered if using a map would be useful. We ended
up deciding that the map would not add significant value to being able to find as many tags as possible. With the belief that the majority of the tags will be on the outer walls,
we decided a wall following algorithm would be sufficient to get a good score for the search and rescue. We tested the AprilTag detection in the lab to get a feel for how far away
and at what angles in the camera's field of view we could have a tag be present and get detected. We came to the results of about 5 feet and 30 degrees of view to be safe. This led
us to develop a wall following algorithm to meet these specifications.

The algorithm begins in a state where it is searching for a wall to follow. It uses a range of lidar scans within its front and right sides to detect if there is an object within
about a meter of it. If the entire range of the lidar that is sampled is not consistant with the minimum reading, it sees it as an obstacle in the way and attempts to move around it. If
the whole range of lidars is simalarly small, we call that a wall. When it first detects a wall, the robot spins about 360 degrees to be able to capture any AprilTags in the area. It then
continues to spin about 90 degrees to begin following the wall. The robot will stop every 5-7 meters to spin around another 360 degrees to detect more AprilTags. We chose to do this because
we wanted to robot to be able to detect as many AprilTags in as few spins as possible. This will aslo allow it to see any AprilTags that may be in the middle of the area facing the walls.

The algorithm has a few corrective states as well. First, when the controller is first started, the robot enters a 'wall finder' state. This state makes the robot travel straight until something
is detected in front of it. If the robot gets stuck in a state where it keeps detecting something in front of it that isn't a wall, it will make moves to turn 90 degrees, go straight, and turn
back the way it was going originally.

To start the AprilTag detection, we attempted to create a launch file, but we couldn't get that working in time. In place of that, I incorperated two system calls into the run file to run the first 
two given commands used to start the AprilTag detections. I then added a new subscription to the '/detections' topic with a callback to an 'apriltags_detections' function used to log the unique detections.

