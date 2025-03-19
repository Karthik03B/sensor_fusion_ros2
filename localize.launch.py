from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('traj')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Navsat Transform Node
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
        remappings=[('gps/fix', 'navsat/fix'),
                    ('/imu', 'imu/data'),
                    ('odometry/filtered', 'global_ekf/odometry_filtered'),
                    ('odometry/gps', 'navsat_transform/navsat_odometry'),
                    ('gps/filtered', 'navsat_transform/filtered_fix') 
                    ]
    )

    # EKF Node (for fusing IMU and GPS data)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
        remappings=[
            ('/odometry/filtered', 'global_ekf/odometry_filtered')  # Output filtered odometry        
            ]
    )

    return LaunchDescription([
        use_sim_time,
        navsat_transform_node,
        ekf_node,
    ])