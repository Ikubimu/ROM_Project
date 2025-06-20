from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener el path del paquete nav2_bringup
    localization_pkg = get_package_share_directory('nav2_bringup')
    localization_dir = os.path.join(localization_pkg, 'launch')

    # Definir el argumento del mapa
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Ruta al archivo .yaml del mapa'
    )

    # Crear la configuración de sustitución
    map_config = LaunchConfiguration('map')

    # Grupo de acciones para localización
    localization_cmd = GroupAction([

        # Publicar /initialpose solo una vez
        ExecuteProcess(
            cmd=[[
                'ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ',
                '"""{header: {frame_id: \'map\'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, ',
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, ',
                'covariance: [0.01,0.0,0.0,0.0,0.0,0.0, 0.0,0.01,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0076]}}"""'
            ]],
            shell=True
        ),

        # Incluir el launch de localización con el mapa como argumento
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_dir, 'localization_launch.py')),
            launch_arguments={'map': map_config}.items()
        ),

        # Lanzar el nodo amcl_odom pasándole el mapa como parámetro
        Node(
            package='cartografia_pkg',
            executable='amcl_odom',
            name='amcl_odom',
            output='screen',
            parameters=[{'map': map_config}]
        ),
    ])

    # Ensamblar el LaunchDescription final
    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(localization_cmd)

    return ld
