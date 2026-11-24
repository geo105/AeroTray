"""Vista fija del conjunto CAD; no abre puertos ni inicia puentes al ESP32."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='cad_completo', name='estado_cad', output='screen',
             parameters=[{'robot_description': (folder / 'conjunto_completo.urdf').read_text()}]),
        Node(package='rviz2', executable='rviz2', name='rviz_cad_completo',
             arguments=['-d', str(folder / 'conjunto_completo.rviz')], output='screen'),
    ])
