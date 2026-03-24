import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def get_path(package, dir, file):
    return os.path.join(get_package_share_directory(package), dir, file)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_odometry",
                executable="lidar_odometry",
                name="lidar_odometry",
                output="screen",
                parameters=[
                    {
                        "icp_method": "trimmed",
                        "odom_frame": "odom",
                        "use_odom_guess": False,
                        "show_debug_scans": False,
                        "publish_tf": False,
                        "rebase_translation_min_m": 0.05,
                        "rebase_angle_min_deg": 2.0,
                        "rebase_time_min_ms": 1000,
                    }
                ],
            )
        ]
    )
