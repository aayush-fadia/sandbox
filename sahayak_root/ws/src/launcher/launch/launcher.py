from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    robot_spawn_prefix = get_package_share_directory('robot_spawn')
    my_image_proc_prefix = get_package_share_directory('my_depth_image_proc')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_spawn_prefix, 'launch', 'spawn.launch.py'))),
        Node(
            package='assurer',
            executable='Assurer',
            remappings=[
                ("unreliable", "/color_cam/image_raw"),
                ("reliable", "/color_cam/image_raw_reliable"),
            ],
            output='screen'
        ),
        Node(
            package='assurer',
            executable='Assurer',
            remappings=[
                ("unreliable", "/depth_cam/image_depth"),
                ("reliable", "/depth_cam/image_depth_reliable"),
            ],
            output='screen'
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            remappings=[
                ('camera_info', '/color_cam/camera_info'),
                ('image', '/color_cam/image_raw_reliable'),
                ('image_rect', '/color_cam/image_raw_rect')
            ],
            output='screen'
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            remappings=[
                ('camera_info', '/color_cam/camera_info'),
                ('image', '/depth_cam/image_depth_reliable'),
                ('image_rect', '/depth_cam/image_depth_rect')
            ],
            output='screen'
        ),
        Node(
            package='imshower',
            executable='Imshower',
            remappings=[
                ('image', '/color_cam/image_raw_rect')
            ],
            output='screen'
        ),
        Node(package="tf2_ros",
             executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        # Node(package="visual_odometry",
        #      executable="visual_odometry",
        #      remappings=[
        #          ('image', '/kinect_color/image_raw'),
        #          ('depth', '/kinect_range/image_depth'),
        #          ('camera_info', '/kinect_color/camera_info')
        #      ]),
    ])
