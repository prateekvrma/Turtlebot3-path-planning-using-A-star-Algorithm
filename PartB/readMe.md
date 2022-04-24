# ENPM661: Project 3 (Phase 2)
# Implementation of A-star algorithm on Turtlebot3 in ROS. 

## Dependencies
Python 3.7
ROS Melodic or above

## Steps to run:
1. Unzip the PartB folder in the ~/catkin_ws/src folder.

2. Go to the folder location in the terminal.

3. Perform "catkin_make" or "catkin build" operation.

4. Source the setup in the catkin workspace: "source devel/setup.bash"

5. In terminal run: "roslaunch verma_astar turtlebot2_map.launch model:=burger"

6. The above command should launch the Gazebo environment.

7. Next, in another terminal go to the ~/catkin_ws/src/verma_astar/src folder and run:

"python publisher.py"

8. Enter the required parameters:


Enter start x: 1

Enter start y: 1

Enter start theta: 0

Enter start x: 9

Enter start y: 9

Enter the RPM 1: 10

Enter the RPM 1: 15

Enter the radius of the robot: 0.105

Enter the clearance: 0.3

9. Visualize the output.

