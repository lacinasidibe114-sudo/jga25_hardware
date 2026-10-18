from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('jga25_hardware')
    
    # 1. Génération de l'URDF via xacro
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # 2. Chemin vers le fichier yaml
    controllers_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # 3. Le nœud de contrôle principal
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml
        ],
        output='screen',
    )

    # 4. Les "Spawners" (ce qui va charger les contrôleurs)
    # On attend 3 secondes que le control_node démarre avant de charger le premier
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # On attend que le joint_state_broadcaster soit chargé avant de charger le velocity_controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 5. Séquencement (On attend la fin d'un processus pour lancer le suivant)
    delay_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        delay_velocity_controller
    ])