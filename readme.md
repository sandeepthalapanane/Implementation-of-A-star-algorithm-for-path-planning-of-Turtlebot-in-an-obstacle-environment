# ENPM661 - Planning for Autonomous Robots

## Project 3 - Phase 2 
<br />

## Implementation and Visualizattion of A-star Algorithm using Turtlebot 3
<br />

## Team Members
* Sandeep Thalapanane - - sandeept - 119402535
* Sandip Sharan Senthil Kumar - Sandip26 - 119340196

## Environment used - VS code

<br />

## Installation of Dependecies


### `Pygame`

<br />To install pygame, type the following command in the terminal

```
pip install pygame
```

### `OrderedSet`

<br />To install ordered set, type the following command in the terminal

```
pip install orderedset
```

### `Heapdict`

<br />To install heapdict, type the following command in the terminal

```
pip install heapdict
```

### `Argparse`

<br />To install argparse, type the following command in the terminal

```
pip install argparse
```

### `ROS2 - Galactic`

<br />ROS2 Installation link

````
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
````
### `Turtlebot3` 

</br> Turtlebot3 Installation Link
````
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html
````
- Follow the steps from "__2.1 - B.Install from source code__" given in the link above to download the required turtlebot 3 repos

## Libraries Used 

- Numpy
- Time
- Math

## Git Repo Link 

https://github.com/sandipsharan/A-star-algorithm-for-turtlebot.git

## To Run the program


### <ins>Part - 1 (A -star)</ins>

- Unzip the given package and open the file name given below in VSC
````
proj3p2_a_star_sandeep_sandipsharan_part1.py
````
- Give the required inputs such as
* goal_node => goal_x:=4(meters) goal_y:=0.25(meters)
* start_node => start_x:=0.5(meters) start_y:=1(meters)
* RPM1, RPM2, Clearance => RPM1:=50 RPM2:=100 clearance:=50(mm)

- Click `Ctrl+Alt+N` or the run button on the top right corner of the application


### <ins>Part - 2 (ROS)</ins>

- Unzip the given package to your ros2_workspace src folder
- Open the terminal and run the command given below
```
source /opt/ros/galactic/setup.bash
source ~/ros2_workspace_name/install/setup.bash
```
```
cd ros2_workspace_name
```
- To run the launch file run the command given below
```
colcon build
```
```
source install/local_setup.bash
```
```
ros2 launch a_star_tb3 empty_world.launch.py goal_x:=5 goal_y:=0 start_x:=0 start_y:=0.25 RPM1:=40 RPM2:=20 clearance:=100 
```
- Input sample for goal_node => goal_x:=4(meters) goal_y:=0.25(meters)
- Input sample for start_node => start_x:=0.5(meters) start_y:=1(meters)
- Input sample for RPM1, RPM2, Clearance => RPM1:=50 RPM2:=100 clearance:=50(mm)

- To change the input values of start node and goal node, you can edit in the ros2 launch command given above with your own values
- After giving the inputs, you can see the path being printed out in the terminal and also in the pygame window and the velocity commands started to publish on the turtlebot which can be seen in the gazebo window
</br>

## Output

Visualization Video link 

https://drive.google.com/file/d/1zll9keBU-kjOqwKYhA4M8OywR19TG_tA/view?usp=share_link

Gazebo Video Link 

https://drive.google.com/file/d/1zMZkRd9BUZkixb4Scdb6FKqUckAuu_gh/view?usp=share_link








