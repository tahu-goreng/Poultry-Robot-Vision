from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_params = LaunchConfiguration('lidar_params')

    my_robot_share = get_package_share_directory('my_robot')
    slam_params = os.path.join(my_robot_share, 'config', 'slam_params_online_async.yaml')
    rviz_cfg    = os.path.join(my_robot_share, 'config', 'ydlidar_slam_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        DeclareLaunchArgument(
            'lidar_params',
            default_value='/home/waesco704/ydlidar_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml',
            description='Path to YDLIDAR params yaml'
        ),

        # ==========================
        # YDLIDAR Driver (FORCE params-file)
        # ==========================
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            arguments=['--ros-args', '--params-file', lidar_params],
        ),

        # ==========================
        # ESP32 Serial Bridge (ODOM + TF odom->base_link)
        # ==========================
        Node(
            package='my_robot',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'cmd_timeout': 0.5
            }]
        ),

        # ==========================
        # SLAM Toolbox (MAPPING)
        # ==========================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                slam_params
            ]
        ),

        # ==========================
        # Static TF: base_link → laser_frame
        # ==========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            output='screen',
            arguments=['0.25', '0.0', '0.65', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        # ==========================
        # cmd_vel_bridge
        # ==========================
        Node(
            package='my_robot',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[{
                'min_distance_stop': 0.25,
                'slowdown_start': 0.8,
                'scan_topic': '/scan',
                'input_cmd': '/cmd_vel',
                'output_cmd': '/cmd_vel_safe'
            }]
        ),

        # ==========================
        # RViz
        # ==========================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])

