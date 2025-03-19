import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file in external package
    pkg_name = 'traj'
    urdf_pkg_name = 'traj'
    file_subpath = 'urdf/diff_drive.urdf.xacro'
    pkg_share = get_package_share_directory('traj')

    # Set ignition resource path (so it can find your world files)
    resource_paths = [os.path.join(get_package_prefix(urdf_pkg_name), 'share'), os.path.join(get_package_share_directory(pkg_name), "worlds")] #os.path.join(get_package_prefix(pkg_name), 'share')
    
    x_arg = DeclareLaunchArgument(name='x', default_value='0', description='x-position')
    y_arg = DeclareLaunchArgument(name='y', default_value='0', description='y-position')

    # Use xacro to process the URDF file
    xacro_file = os.path.join(get_package_share_directory(urdf_pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # launch_local= IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([get_package_share_directory('traj'), '/launch', '/localize.launch.py']),
    # )

     ## Arguments
    world_arg = DeclareLaunchArgument(name='world', default_value='empty_world', description='Gazebo world name')

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'gz_args': ' -r ' + pkg_share + '/worlds/empty_world.sdf'
        }.items(),
    )

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                    '-x', LaunchConfiguration('x'),
                                    '-y', LaunchConfiguration('y'),
                                   '-z', '1.0'],
                        output='screen')

    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # Clock bridge
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

        # Command velocity bridge
        '/model/diff_drive_example/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',

        # Odometry bridge
        '/model/diff_drive_example/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

        # TF bridge
        '/model/diff_drive_example/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

        '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',

        # IMU bridge
        '/model/diff_drive_example/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',

        # Joint state bridge
        '/model/diff_drive_example/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ],
    parameters=[{'qos_overrides./diff_drive_example.subscriber.reliability': 'reliable'}],
    remappings=[
        # Remap Gazebo topics to ROS 2 topics
        ('/model/diff_drive_example/cmd_vel', '/cmd_vel'),
        ('/model/diff_drive_example/odometry', '/odom'),
        ('/model/diff_drive_example/tf', '/tf'),
        ('/model/diff_drive_example/imu', '/imu/data'),
        ('/model/diff_drive_example/joint_state', '/joint_states'),
        # ('/gps/fix', '/navsat/fix'),
    ],
    output='screen'
)

    # Add actions to LaunchDescription
    # Simulation
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(world_arg)
    ld.add_action(gazebo)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_ros_gz_bridge)
    return ld