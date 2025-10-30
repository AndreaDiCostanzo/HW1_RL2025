from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    jsp_gui = DeclareLaunchArgument(name='jsp_gui', 
                                    description='Flag to enable joint_state_publisher_gui',
                                    default_value='true', 
                                    choices=['true', 'false'])
    
    rviz_config_arg = DeclareLaunchArgument(name='rviz2_config', 
                          default_value= PathJoinSubstitution([FindPackageShare('armando_description'), 'config', 'rviz','config.rviz']),
                          description='Absolute path to rviz config file')


    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/arm.urdf.xacro')

#    xacro_armando = os.path.join(get_package_share_directory('armando_description'), "urdf", "arm.urdf.xacro")

#    armando_description = ParameterValue(
#        Command(["xacro ", xacro_armando]),
#        value_type=str
#    )

#    robot_state_publisher_node = Node(
#        package="robot_state_publisher",
#        executable="robot_state_publisher",
#        output="both",
#        parameters=[{"robot_description": armando_description,
#                     "use_sim_time": True}],
#    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
        )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz2_config')],
        )

    return LaunchDescription([
        #robot_state_publisher_node,
        package_arg,
        model_arg,
        jsp_gui,
        rviz_config_arg,
        description_launch_py,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])