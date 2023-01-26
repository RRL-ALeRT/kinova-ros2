import os
from ament_index_python.packages import get_package_share_directory
import  launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml

configurable_parameters = [
    {'name': 'use_urdf',              'default': 'true'},
    {'name': 'kinova_robotType',      'default': "j2n6s300"},
    {'name': 'kinova_robotName',      'default': "left"},
    {'name': 'kinova_robotSerial',    'default': "not_set"},
    {'name': 'use_jaco_v1_fingers',   'default': "false"},
    {'name': 'feedback_publish_rate', 'default': "0.1"},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def generate_launch_description():
    _config_file = os.path.join(
        get_package_share_directory('kinova_bringup'),
        'launch/config',
        'robot_parameters.yaml'
    )
    params_from_file = yaml_to_dict(_config_file)

    kinova_driver = launch_ros.actions.Node(
        package='kinova_driver',
        name=LaunchConfiguration("kinova_robotName"),
        executable='kinova_arm_driver',
        parameters=[set_configurable_parameters(configurable_parameters), params_from_file],
        output='screen',
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        kinova_driver
    ])
