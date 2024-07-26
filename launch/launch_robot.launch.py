import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():

    package_name='truck' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    
    joint_limits_file = os.path.join(get_package_share_directory(package_name), 'config', 'joint_limits.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file, {'ros__parameters': {'joint_limits': joint_limits_file}}],
         remappings=[('ackermann_steering_controller/odometry', '/odom'),
                    ('ackermann_steering_controller/tf_odometry', '/tf')]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
    )
    delayed_ackermann_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_steering_controller_spawner],
        )
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    serial_controller = Node(
            package='truck', 
            executable='serial_controller.py', 
            name='serial_controller',
            output='screen'
            )
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_ackermann_steering_controller_spawner,
        delayed_joint_broad_spawner,
        serial_controller
    ])