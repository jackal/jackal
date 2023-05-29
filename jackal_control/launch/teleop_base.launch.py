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

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'twist_mux.yaml']
    )

    filepath_config_interactive_markers = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'teleop_interactive_markers.yaml']
    )

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[{filepath_config_interactive_markers}, {'use_sim_time': is_sim}, {'robot_description': robot_description_content},],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[{filepath_config_twist_mux}, {'use_sim_time': is_sim}, {'robot_description': robot_description_content},]
    )

    ld = LaunchDescription()
    ld.add_action(is_sim_arg)
    ld.add_action(robot_description_command_arg)
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
