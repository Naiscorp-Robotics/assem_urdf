import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    # =============================================
    # Package and Path Configuration
    # =============================================
    pkg_share = FindPackageShare(
        package="assem_urdf"
    ).find("assem_urdf")
    pkg_path = os.path.join(get_package_share_directory('assem_urdf'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'assem_urdf.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # =============================================
    # Launch Arguments
    # =============================================
    # Simulation time configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_localization = LaunchConfiguration('use_localization')
    # Gazebo configuration
    run_headless = LaunchConfiguration('run_headless')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    world_file_name = LaunchConfiguration('world_file_name')
    world_path = PathJoinSubstitution([pkg_path, 'worlds', world_file_name])
    log_level = LaunchConfiguration('log_level')
    # Gazebo models path configuration
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])

    # =============================================
    # Environment Variables
    # =============================================
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }

    # =============================================
    # Node Configuration
    # =============================================
    # Robot State Publisher parameters
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # =============================================
    # Node Definitions
    # =============================================
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RViz2 Node
    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_path, "rviz", "assem_urdf.rviz")]
    )

    # =============================================
    # Gazebo Process Configuration
    # =============================================
    gazebo = [
        # Headless mode configuration
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env,
            shell=False,
        ),
        # Normal mode configuration
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env,
            shell=False,
        )
    ]
    robot_localization_node = Node(
        condition=launch.conditions.IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "assem_urdf",
            "-topic", "robot_description",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "-2.0",
            "--ros-args",
            "-log-level",
            log_level,
        ],
    )

    # =============================================
    # Bridge Configuration
    # =============================================
    # ROS2-Ignition Bridge for sensor and control topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Based on the URDF, we have a lidar sensor
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # IMU sensor is referenced in ekf.yaml configuration
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # Differential drive controller for the four wheels (left_front, right_front, left_behind, right_behind)
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # Clock message is necessary for the diff_drive_controller to accept commands
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )    

    # =============================================
    # Controller Configuration
    # =============================================
    # Load and activate joint state broadcaster
    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    # Load and activate differential drive controller
    load_joint_trajectory_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        shell=False,
        output="screen",
    )

    # =============================================
    # Topic Relay Configuration
    # =============================================
    # Relay odometry data from controller to standard topic
    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
            }
        ],
        output="screen",
    )

    # Relay velocity commands to controller
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )

    # =============================================
    # Launch Description
    # =============================================
    return LaunchDescription([
        # Set Gazebo models path
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=gz_models_path,
        ),
        
        # Launch Arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for ros_ign_sim'
        ),
        DeclareLaunchArgument(
            'use_localization',
            default_value='true',
            description='Use localization if true'
        ),
        DeclareLaunchArgument(
            'world_file_name',
            default_value='empty.sdf',
            description='SDF file name for gazebo'
        ),
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='3',
            description='Verbosity of gazebo'
        ),
        DeclareLaunchArgument(
            'run_headless',
            default_value='false',
            description='Run in headless mode if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        # Nodes and Processes
        bridge,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        node_rviz2,

        # Event Handlers for Controller Loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        # Topic Relays
        relay_odom,
        relay_cmd_vel,
    ] + gazebo)