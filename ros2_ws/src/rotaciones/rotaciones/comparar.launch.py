"""Lanza las dos copias del nodo de RViz: una para el filtro propio (1.1)
y otra para el DMP (1.2).

Uso (desde la carpeta donde esta rotaciones.py):
    ros2 launch comparar.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

AQUI = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(AQUI, 'rotaciones.py')


def generate_launch_description():
    return LaunchDescription([
        # 1.1: filtro propio, flecha naranja.
        ExecuteProcess(
            cmd=['python3', SCRIPT, '--ros-args',
                 '-r', '__node:=rotaciones_propio',
                 '-p', 'color_flecha:=[1.0, 0.5, 0.1]'],
            output='screen'),

        # 1.2: DMP, flecha celeste.
        ExecuteProcess(
            cmd=['python3', SCRIPT, '--ros-args',
                 '-r', '__node:=rotaciones_dmp',
                 '-p', 'topico_pose:=/imu_pose_dmp',
                 '-p', 'topico_vel:=/imu_vel_dmp',
                 '-p', 'topico_flecha:=/imu_vel_marker_dmp',
                 '-p', 'topico_rastro:=/imu_path_dmp',
                 '-p', 'color_flecha:=[0.2, 0.7, 1.0]'],
            output='screen'),
    ])