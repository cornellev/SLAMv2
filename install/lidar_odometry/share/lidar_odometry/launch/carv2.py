from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Custom Motor Controller (GPIO 18)
            Node(
                package="cev_lidar_odometry_ros2",
                executable="motor_controller",
                name="motor_controller_node",
                output="screen",
            ),
            
            # 2. Official RPLIDAR Driver (UART TX/RX)
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                output="screen",
                parameters=[{
                    "serial_port": "/dev/ttyAMA0",  # Pi's hardware UART
                    "serial_baudrate": 115200,      # A1M8 baudrate
                    "frame_id": "laser",
                    'inverted': False,
                    "angle_compensate": True,
                    "scan_mode": "Standard"
                }],
            ),

            # 3. Your Existing ICP Odometry Node
            Node(
                package="lidar_odometry",
                executable="lidar_odometry",
                name="lidar_odometry_node",
                output="screen",
                parameters=[
                    {
                        "icp_method": "trimmed",
                        "odom_frame": "odom",
                        "use_odom_guess": False,
                        "show_debug_scans": True,
                        "publish_tf": False,
                        "rebase_translation_min_m": 0.05,
                        "rebase_angle_min_deg": 2.0,
                        "rebase_time_min_ms": 1000,
                    }
                ],
                remappings=[("odom_guess", "/odometry/filtered")],
            ),
        ]
    )