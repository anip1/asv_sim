import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('asv_sim')
    dunia = os.path.join(pkg, 'worlds', 'kolam_asv.sdf')
    jembatan = os.path.join(pkg, 'config', 'jembatan.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {dunia}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': jembatan}],
        output='screen',
    )

    mixer = Node(package='asv_sim', executable='mixer', output='screen')
    autopilot = Node(package='asv_sim', executable='autopilot', output='screen')

    return LaunchDescription([gazebo, bridge, mixer, autopilot])
