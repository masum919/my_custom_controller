import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # get urdf via xacro to show actuator movement
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('my_custom_controller'),
                    'urdf',
                    'my_custom_robot.urdf.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}



    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # configuration for rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('my_custom_controller'), 'rviz', 'my_custom_robot_rviz.rviz']
    )




    # configuration specifying type of controllers for joints and their configurable parameters
    datras_system = PathJoinSubstitution(
        [
            FindPackageShare('my_custom_controller'),
            'config',
            'my_custom_controller_config.yaml',
        ]
    )



    # node for controller manager which manages lifecycle of controllers defined for joints
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace="/my_custom_controller",
        parameters=[robot_description, datras_system],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
       
        remappings=[(
            "/my_custom_controller/my_custom_controller/commands",
            "/velocity_commands",
        )],
    )

    # node which publishes robot state including the robot description
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="/my_custom_controller",
        output='both',
        parameters=[robot_description],
        
    )

    # node for launching rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # node to broadcasting joint states of various joints listed in urdf and configuration
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace="my_custom_controller",
        #arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        arguments=["joint_state_broadcaster"],
    )
    
    # node to start controllers associated with the joints
    motor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace="my_custom_controller",
        #arguments=['motor_controller', '--controller-manager', '/controller_manager'],
        arguments=["rpm_controller", "--param-file", datras_system],  # controller name -- rpm_controller
    )




    # delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # delay start of robot_controller after `joint_state_broadcaster`
    delay_motor_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_controller_spawner],
        )
    )

    return LaunchDescription([
      
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_motor_controller_spawner_after_joint_state_broadcaster_spawner,
    ])