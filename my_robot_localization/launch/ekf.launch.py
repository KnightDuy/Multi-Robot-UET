import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Đường dẫn config cho imu_complementary_filter
    imu_config = os.path.join(
        get_package_share_directory('imu_complementary_filter'),
        'config',
        'filter_config.yaml'
    )

    odom_node = Node(
        package='odom_encoder',
        executable='odom_node',       # lấy từ entry_points
        name='odom_node',
        output='screen',
    )

    fake_imu_node = Node(
        package='odom_encoder',
        executable='fake_imu_node',   # lấy từ entry_points
        name='fake_imu_node',
        output='screen',
    )
    # Node imu_complementary_filter
    imu_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[imu_config]
    )

    # Node ekf
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot2',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_localization'),
                'config',
                'ekf.yaml'
            ])
        ]
    )

    # Gộp lại
    ld = LaunchDescription()
    ld.add_action(odom_node)
    ld.add_action(fake_imu_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)

    return ld
