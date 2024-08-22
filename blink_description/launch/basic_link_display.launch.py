from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    #Ruta del archivo URDF
    urdf_path = os.path.join(get_package_share_path('blink_description'),
                             'urdf', 'basic_link.urdf')
    #Ruta del archvo RVIZ
    rviz_config_path = os.path.join(get_package_share_path('blink_description'),
                                    'rviz', 'basic_link_config.rviz')
    
    #Definicion del parametro de la ruta del archivo URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    #Ejecucion del nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    #Ejecucion del nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    #Ejecucion del nodo de RVIZ
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    #Retorno de la funcion del archivo launch
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])