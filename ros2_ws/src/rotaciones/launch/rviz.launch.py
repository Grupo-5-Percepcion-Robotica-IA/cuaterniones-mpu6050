"""
Uso:
    ros2 launch rotaciones rviz.launch.py
    ros2 launch rotaciones rviz.launch.py plotjuggler:=true   # + graficos en vivo
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rotaciones')
    rviz_config = os.path.join(share, 'rviz', 'comparar.rviz')

    # Si la configuracion no esta instalada, RViz abre vacio en vez de fallar.
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    usar_plotjuggler = LaunchConfiguration('plotjuggler')

    return LaunchDescription([
        DeclareLaunchArgument(
            'plotjuggler',
            default_value='false',
            description='true para abrir tambien PlotJuggler (graficos en vivo)'),

        # El ESP32 publica en el dominio 10: todo lo de aca tiene que estar ahi.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),

        # Frame fijo para RViz.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_a_imu_link',
            arguments=['--frame-id', 'world', '--child-frame-id', 'imu_link']),

        # 1.1: filtro propio, flecha naranja.
        Node(
            package='rotaciones',
            executable='rotaciones',
            name='rotaciones_propio',
            output='screen',
            parameters=[{
                'color_flecha': [1.0, 0.5, 0.1],
            }]),

        # 1.2: DMP, flecha celeste.
        Node(
            package='rotaciones',
            executable='rotaciones',
            name='rotaciones_dmp',
            output='screen',
            parameters=[{
                'topico_pose': '/imu_pose_dmp',
                'topico_vel': '/imu_vel_dmp',
                'topico_flecha': '/imu_vel_marker_dmp',
                'topico_rastro': '/imu_path_dmp',
                'color_flecha': [0.2, 0.7, 1.0],
            }]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            output='screen'),

        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
            condition=IfCondition(usar_plotjuggler)),
    ])
