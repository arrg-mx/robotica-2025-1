from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.event_handlers import OnProcessExit





def generate_launch_description():
    

    urdf_path = os.path.join(get_package_share_path('examen_description'),
                             'urdf', 'scara_trajectory_controller.xacro')
    
    rviz_config_path = os.path.join( get_package_share_directory('examen_bringup'),
                                    'rviz', 'scara_trayectory_rviz.rviz')

    robot_description = ParameterValue(Command(['xacro', urdf_path]),value_type=str)
    
    world = os.path.join(
        get_package_share_directory('examen_bringup'),
        'world', 'test_2_world.wold'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'),
                      '/gazebo.launch.py']),
                        launch_arguments={'world':world}.items()
             )


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description} ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    join_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scara'],
                        output='screen')
    
    scara_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'scara_trajectory_controller'],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen' 
    )
    
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
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz2_node,
        join_state_publisher
        
    ])