# YZV406E, Robotics, Fall 2024
# Assignment #1 - TurtleBot the Explorer

**Due Date:** check Ninova

> This is an individual assignment, group work is not allowed!  
> Any type of cheating is strictly prohibited. Please, submit your own work!  
> You can ask your questions via e-mail (bicer21@itu.edu.tr) or message board in Ninova

## Summary

### Assignment
Turtlebot is instructed to explore the map you have created. As a fellow engineer of the robot, you need to make this possible by publishing movements that make the robot navigate around the world you have created. You will create the world using Gazebo. The world will include rooms where robot needs to explore. The robot can explore rooms and exit the rooms using the laser information. You need to write a custom ROS2 node that explores the area. You will be utilizing laser sensor values of the TurtleBot (/scan topic) to avoid obstacles while the robot wanders around. The robot should continuously navigate around the world. Your solution may be tested on another world, so avoid implementing a navigation node that is developed specifically for your world. Your solution will be tested for 2 minutes at maximum.

### Submission Type
1. A Python file containing the source code of your controller. The file to be submitted is called `turtle_explorer.py`. A skeleton for the file can be found in an archive supplied with the assignment description. It should be possible to compile the package by placing the file into a ros2 workspace as described in further and running the colcon build. Do not forget to add your id's to your code as a comment in the specified section.

2. A single page report that addresses followings:
   - 1-2 sentences that explain the world you have created and the robot's role in the world.
   - Explain how you have managed to explore the map without hitting an obstacle.
   - Report the explored area with percentage (include the extracted map).
   - Explain challenges and how you have managed to overcome them.

3. Also include `map.pgm`, and `map.yaml` in the submission.

## Skeleton Code, Referee and Preparing Solution

### Tracker Node checks followings:
- Percentage of the explored area of the map
- Elapsed time in seconds

Tracker would log the information above in the terminal. The robot should explore the world and avoid obstacles. Tracker would save the extracted map after 120 seconds as .pgm file under the directory where the launch file is executed.

## Creating the map with Gazebo

In this assignment, the map where robot wanders will be designed by you! The designed world should be a location within ITU Campus. Design your world according to that location (e.g. Faculty Entrance Floor). You need to follow below steps to create the map with gazebo:

1. Open gazebo with the command: `gazebo`
2. Select "Building Editor" from the "Edit" menu.
3. Create your world with the following specifications:
   - There should be at least 4 room-like wall separations
   - Map should be at least 10x12 gazebo grids (below pane)
   - Do not make huge worlds as it would take too much time to explore the world (14x14 at maximum)!
4. Save the world model as "A1_World"
5. Add at least 3-4 simple objects to create a more challenging environment

## Run the launch file

1. `ros2 launch a1 a1_launch.py`
2. Run the solution code: `ros2 run a1 turtle_explorer.py`

## Developing the exploring node

Source code for answer node will contain your implementation to move the robot to explore the world as much as possible around 2 minutes. The robot will make use of the laser sensor values coming from /scan topic to avoid obstacles while exploring the world.

### Launch File Components
The launch file ("nav_launch.py") executes following:
- Gazebo
- MapServer for publishing map
- AMCL for localizing robot in the given map
- Nav2 Lifecycle to configure and launch MapServer and AMCL in coordination
- Launch file of Robot State Publisher
- RViz
- Referee Node

### Workspace Preparation
To finalize the preparation of workspace:
1. Move the downloaded "a1" folder to "~/ros2_ws/src" folder in home
2. Check dependencies: within the "~/ros2_ws" execute `rosdep install -i --from-path src --rosdistro humble -y`
3. Move the world and model files into corresponding folders as aforementioned above
4. Within the "~/ros2_ws", execute `colcon build` command

### Running the Project
Open up a new terminal and execute launch file by:
```bash
ros2 launch a1 a1.launch.py x_pose:=-5 y_pose:=-3 world_name:=A1_world.world
```

Then, skeleton code can be executed by:
```bash
ros2 run a1 turtle_explorer
```

## Technical Information

### Scan Data Usage
Laser scan data will be available within the callback function that your program registers when it subscribes to the /scan topic. A LaserScan message provides information about how far objects are from the robot via 360 laser sensors on its body.

### Robot Navigation
You can send Twist messages to control the robot's velocity:
- Set x component of linear velocity
- Set z component of angular velocity

### Map Information
The map is provided in an array with occupancy_grid variable with the following values:
- 0 = fully free space
- 100 = fully occupied
- -1 = unknown (not yet explored)

### Additional Tools
- Use `ros2 run tf2_tools view_frames` and `evince frames.pdf` to view TF frames
- Use `ros2 run tf2_ros tf2_echo /base_footprint /odom` to see transformations

## Marking Criteria

- Creation of a unique world
- Publishing movements that navigates the robot
- Exploration of the world through robot navigation
- Obstacle/Wall collision avoidance
- Modular code: divide operations into functions to make code seem more elegant
- Code with comments (good practice)
- Sufficient explanation for specified questions in Report document

## Hints

- Use teleoperation to get the sense of robot navigation
- Consider implementing a controller like Proportional (P) control or another advanced controller
- You may make use of /map topic for intelligent exploration