"""Simulasi ASV sesuai pedoman 2026.

Memakai sim.launch.py yang sudah ada (Gazebo, jembatan, mixer, autopilot), lalu:
  - mengalihkan keluaran autopilot dari /cmd_vel ke /cmd_vel_auto,
  - menambah node keselamatan (E-stop, geofence, fail-safe) yang menerbitkan /cmd_vel,
  - menambah node dock_biru (merapat ke bola biru) dan dashboard_web.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = get_package_share_directory('asv_sim')
    cfg = os.path.join(share, 'config', 'pedoman.yaml')

    sim = GroupAction([
        # aturan remap khusus node: hanya autopilot_gerbang yang dialihkan
        SetRemap(src='autopilot_gerbang:/cmd_vel', dst='/cmd_vel_auto'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'sim.launch.py'))),
    ])

    nodes = [
        Node(package='asv_sim', executable=exe, name=nama, parameters=[cfg], output='screen')
        for exe, nama in (('keselamatan', 'keselamatan'),
                          ('dock_biru', 'dock_biru'),
                          ('dashboard', 'dashboard_web'))
    ]
    return LaunchDescription([sim] + nodes)
