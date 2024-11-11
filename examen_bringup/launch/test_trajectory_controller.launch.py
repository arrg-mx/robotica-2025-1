
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit



def generate_launch_description():

    #Ruta del archivo URDF
    urdf_path = os.path.join(get_package_share_path('examen_description'),
                             'urdf', 'scara_trajectory_controller.xacro')
    #Ruta del archvo RVIZ
    
    package_path = os.path.join( get_package_share_directory('examen_description') )

    rviz_config_path = os.path.join( package_path , 'rviz', 'urdf.rviz' )
    
    #Definicion del parametro de la ruta del archivo URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    world = os.path.join(
        get_package_share_directory('examen_bringup'),
        'world',
        'test_w_1.world'
    )

    #Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world}.items()
             )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scara'],
                        output='screen')


    #Ejecucion del nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    '''
    #Ejecucion del nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    '''
    '''
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    '''

    #Ejecucion del nodo de RVIZ
    config_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = rviz_config_path)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    scara_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'scara_trajectory_controller'],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen' 
    )
    

    #Retorno de la funcion del archivo launch
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[scara_controller],
            )
        ),
        robot_state_publisher_node,
        #joint_state_publisher,
        config_arg,
        rviz2_node,
        gazebo,
        spawn_entity
    ])