from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_type = LaunchConfiguration('controller_type')
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true',
    )

    controller_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='0',
    )


    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/arm.urdf.xacro')

    xacro_armando = os.path.join(get_package_share_directory('armando_description'), "urdf", "arm.urdf.xacro")

    robot_description_armando_xacro = {"robot_description": Command(['xacro ', xacro_armando])}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_armando_xacro,
                    {"use_sim_time": True},
            ],
    )

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )

    #this is not used
    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        output='screen',
    )

    position_controller = Node( package="controller_manager",
                           executable="spawner", 
                           arguments=["position_controller", "--controller-manager", "/controller_manager"], 
                           condition = IfCondition(PythonExpression([controller_type, " == 0"])))

    joint_trajectory_controller = Node( package="controller_manager",
                           executable="spawner", 
                           arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"], 
                           condition = IfCondition(PythonExpression([controller_type, " == 1"])))

    joint_state_broadcaster = Node( package="controller_manager",
                        executable="spawner", 
                        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], )


    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node,
            on_exit=[position_controller],
        )
    )

    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node,
            on_exit=[joint_trajectory_controller],
        )
    )


    delay_joint_state_broadcaster = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner_node,
                on_exit=[joint_state_broadcaster],
            )
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gui_arg,
        controller_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner_node,
        delay_joint_trajectory_controller,
        delay_position_controller,
        delay_joint_state_broadcaster
    ])