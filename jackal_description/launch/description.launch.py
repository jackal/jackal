from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
        
    is_sim_arg = DeclareLaunchArgument('is_sim', default_value='False')
    is_sim = LaunchConfiguration('is_sim')

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            )
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{'use_sim_time': is_sim}, {'robot_description': robot_description_content},])

    ld = LaunchDescription()
    ld.add_action(is_sim_arg)
    ld.add_action(robot_description_command_arg)
    ld.add_action(robot_state_publisher_node)
    return ld
