import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()
    
    # Nodes to launch
    imu_node = Node(
        package="rsl_gps",
        executable="run_imu",
    )

    gps_node = Node(
        package="rsl_gps",
        executable="run_gps",
    )
    
    target_node = Node(
        package="rsl_gps",
        executable="push_target",
    )


    
    navigation_core = Node(
        package="navigation_core",
            executable="waypoint_controller")


    ld.add_action(target_node)
    ld.add_action(gps_node)
    ld.add_action(imu_node)
    # ld.add_action(navigation_core)

    return ld


