from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    jackal_description_path = get_package_share_path('jackal_description')

    # Set Environment variables based on the LaunchConfiguration "config"

    # The base Jackal configuration has no accessories at all,
    # so nothing need be specified if config==base; the defaults as given
    # in the URDF suffice to define this config.

    bb2 = SetEnvironmentVariable('JACKAL_BB2', '1',
                                 condition=LaunchConfigurationEquals('config', 'front_bumblebee2'))
    flea3 = SetEnvironmentVariable('JACKAL_FLEA3', '1',
                                   condition=LaunchConfigurationEquals('config', 'front_flea3'))

    # The front_laser configuration of Jackal is sufficient for
    # basic gmapping and navigation. It is mostly the default
    # config, but with a SICK LMS100 series LIDAR on the front,
    # pointing forward.
    laser = SetEnvironmentVariable('JACKAL_LASER', '1',
                                   condition=LaunchConfigurationEquals('config', 'front_laser'))

    robot_description_content = ParameterValue(Command(['xacro ',
                                                       str(jackal_description_path / 'urdf' / 'jackal.urdf.xacro')]),
                                               value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    return LaunchDescription([bb2, flea3, laser, robot_state_publisher_node])
