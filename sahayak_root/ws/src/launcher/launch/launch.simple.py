from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_spawn_prefix = get_package_share_directory('robot_spawn_simple')
    return LaunchDescription([
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments=["0", "0.75", "0.05", "0", "0", "0", "map", "marker_0"],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_spawn_prefix, 'launch', 'spawn.launch.py'))),
        Node(
            package='assurer',
            executable='Assurer',
            remappings=[
                ("unreliable", "/camera/image_raw"),
                ("reliable", "/camera/image_raw_reliable"),
            ],
            output='screen'
        ),
        Node(
            package='assurer',
            executable='Assurer',
            remappings=[
                ("unreliable", "/range_finder/image_depth"),
                ("reliable", "/range_finder/image_depth_reliable"),
            ],
            output='screen'
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            remappings=[
                ('camera_info', '/camera/camera_info'),
                ('image', '/camera/image_raw_reliable'),
                ('image_rect', '/camera/image_raw_rect')
            ],
            output='screen'
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            remappings=[
                ('camera_info', '/camera/camera_info'),
                ('image', '/range_finder/image_depth_reliable'),
                ('image_rect', '/range_finder/image_depth_rect')
            ],
            output='screen'
        ),
        Node(
            package='imshower',
            executable='Imshower',
            remappings=[
                ('image', '/camera/image_raw_rect')
            ],
            output='screen'
        ),
        # Node(package="visual_odometry",
        #      executable="visual_odometry",
        #      remappings=[
        #          ('image', '/kinect_color/image_raw'),
        #          ('depth', '/kinect_range/image_depth'),
        #          ('camera_info', '/kinect_color/camera_info')
        #      ]),
    ])
