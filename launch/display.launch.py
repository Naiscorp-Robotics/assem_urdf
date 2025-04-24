import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition, AndSubstitution, NotSubstitution

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    # check if we're told to run in the headless mode
    run_headless = LaunchConfiguration ('run_headless')
    # check if we want to runt he rviz
    use_rviz = LaunchConfiguration("use_rviz")
    # the level of the gazebo verbosity 
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('assem_urdf'))
    xacro_file = os.path.join(pkg_path,'urdf','assem_urdf.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    # Define gz_models_path 
    pkg_share = FindPackageShare(package='assem_urdf').find('assem_urdf')
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    # Define the log level 
    log_level = LaunchConfiguration("log_level")
    # the path to the world file 
    world_path = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world_name")])
    # Create a joint_state_publisher_gui node 
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a rviz2 node 
    node_rviz2 = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_path, "rviz", "assem_urdf.rviz")]
    )
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]
    spawm_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "assem_urdf",
            "-topic",
            "robot_description",
            "-z",
            "0.0",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )



    # Launch!
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name='IGN_GAZEBO_RESOURCE_PATH',
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use sim time if true'
            ),
            DeclareLaunchArgument(
                'run_headless',
                default_value="false",
                description="Use the headless mode and don't override the use_rviz argument"
            ),
            DeclareLaunchArgument(
                'gz_verbosity',
                default_value="3",
                description="Gazebo verbosity"
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value="warn",
                description="Log level from info to debug"
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value="empty.sdf",
                description="The name of the world to spawn the model in"
            ),
            bridge,
            node_robot_state_publisher,
            node_rviz2,
            node_joint_state_publisher_gui,
            spawm_entity,

        ] + gazebo
    )