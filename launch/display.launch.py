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
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'assem_urdf.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Controller parameters
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_control_velocity.yaml')

    # Gazebo models path configuration
   
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    
    
    
    # =============================================
    # Launch Arguments
    # =============================================
    # Simulation time configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_localization = LaunchConfiguration('use_localization')
    # # Gazebo configuration
    # run_headless = LaunchConfiguration('run_headless')
    # gz_verbosity = LaunchConfiguration('gz_verbosity')
    # world_file_name = LaunchConfiguration('world_file_name')
    # world_path = PathJoinSubstitution([pkg_path, 'worlds', world_file_name])
    # log_level = LaunchConfiguration('log_level')

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
        ]),
        'IGN_GAZEBO_RESOURCE_PATH': gz_models_path
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
            default_value='debug',
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
            default_value='debug',
            description='Verbosity of gazebo'
        ),
        DeclareLaunchArgument(
            'run_headless',
            default_value='false',
            description='Run in headless mode if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),

        # Nodes and Processes
        
        robot_state_publisher_node,
        
    ] )
    