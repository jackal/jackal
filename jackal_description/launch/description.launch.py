from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    jackal_description_path = get_package_share_path('jackal_description')

    robot_description_content = ParameterValue(Command(['xacro ',
                                                       str(jackal_description_path / 'urdf' / 'jackal.urdf.xacro')]),
                                               value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    return LaunchDescription([robot_state_publisher_node])
