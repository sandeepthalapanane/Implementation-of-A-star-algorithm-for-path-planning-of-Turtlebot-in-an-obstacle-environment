
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
import argparse


def generate_launch_description():

    parser = argparse.ArgumentParser()
    parser.add_argument('--goal_x', type=float)
    parser.add_argument('--goal_y', type=float)
    parser.add_argument('--start_x', type=float)
    parser.add_argument('--start_y', type=float)
    parser.add_argument('--RPM1', type=float)
    parser.add_argument('--RPM2', type=float)
    parser.add_argument('--clearance', type=float)
    args, unknown = parser.parse_known_args()

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('--start_x', default='0.0')
    y_pose = LaunchConfiguration('--start_y', default='0.0')

    world = os.path.join(
        get_package_share_directory('a_star_tb3'),
        'worlds',
        'map.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    for arg in unknown:
        if arg.startswith('goal_x:='):
            args.goal_x = float(arg.split(':=')[1])
        elif arg.startswith('goal_y:='):
            args.goal_y = float(arg.split(':=')[1])
        elif arg.startswith('start_x:='):
            args.start_x = float(arg.split(':=')[1])
        elif arg.startswith('start_y:='):
            args.start_y = float(arg.split(':=')[1])
        elif arg.startswith('RPM1:='):
            args.RPM1 = float(arg.split(':=')[1])
        elif arg.startswith('RPM2:='):
            args.RPM2 = float(arg.split(':=')[1])
        elif arg.startswith('clearance:='):
            args.clearance = float(arg.split(':=')[1])
    
    print('args', args)
    
    declared_goalx = DeclareLaunchArgument('--goal_x', default_value=str(args.goal_x))
    declared_goaly = DeclareLaunchArgument('--goal_y', default_value=str(args.goal_y))
    declared_startx = DeclareLaunchArgument('--start_x', default_value=str(args.start_x))
    declared_starty = DeclareLaunchArgument('--start_y', default_value=str(args.start_y))
    declared_RPM1 = DeclareLaunchArgument('--RPM1', default_value=str(args.RPM1))
    declared_RPM2 = DeclareLaunchArgument('--RPM2', default_value=str(args.RPM2))
    declared_clearance = DeclareLaunchArgument('--clearance', default_value=str(args.clearance))


    my_node = TimerAction(
        period=5.0,
        actions=[
            Node(
        package='a_star_tb3',
        executable='a_star_tb3_script.py',
        output='screen',
        emulate_tty=True,
        arguments = [LaunchConfiguration('--goal_x'), LaunchConfiguration('--goal_y'), LaunchConfiguration('--start_x'), LaunchConfiguration('--start_y'),
                    LaunchConfiguration('--RPM1'), LaunchConfiguration('--RPM2'), LaunchConfiguration('--clearance')  ],
    ), ])

   

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declared_startx)
    ld.add_action(declared_starty)
    ld.add_action(declared_RPM1)
    ld.add_action(declared_RPM2)
    ld.add_action(declared_clearance)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(start_rviz)
    ld.add_action(declared_goalx)
    ld.add_action(declared_goaly)
    ld.add_action(my_node)

    return ld
