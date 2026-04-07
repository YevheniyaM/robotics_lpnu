from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    lab2_pkg = FindPackageShare('lab2_marynchak')
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')

    world = PathJoinSubstitution([lab2_pkg, 'worlds', 'robot.sdf'])
    rviz = PathJoinSubstitution([lab2_pkg, 'config', 'robot.rviz'])
    gz_launch = PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [world]}.items(),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan', 
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' 
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'lidar_link']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz],
            output='screen'
        )
    ])