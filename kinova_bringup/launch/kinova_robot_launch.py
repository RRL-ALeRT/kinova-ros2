import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

import xacro
import yaml

configurable_parameters = [
    {'name': 'use_urdf',              'default': "true"},
    {'name': 'kinova_robotType',      'default': "j2n6s300"},
    {'name': 'kinova_robotName',      'default': "left"},
    {'name': 'kinova_robotSerial',    'default': "not_set"},
    {'name': 'use_jaco_v1_fingers',   'default': "true"},
    {'name': 'feedback_publish_rate', 'default': "0.1"},
    {'name': 'tolerance',             'default': "2.0"},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def launch_setup(context, *args, **kwargs):
    _config_file = os.path.join(
        get_package_share_directory('kinova_bringup'),
        'launch/config',
        'robot_parameters.yaml'
    )
    params_from_file = yaml_to_dict(_config_file)

    robot_name = LaunchConfiguration("kinova_robotName").perform(context)
    robot_type = LaunchConfiguration("kinova_robotType").perform(context)
    kinova_driver = Node(
        package='kinova_driver',
        name=robot_type+'_driver',
        executable='kinova_arm_driver',
        parameters=[set_configurable_parameters(configurable_parameters), params_from_file],
        output='screen',
    )
    
    kinova_tf_updater = Node(
        package='kinova_driver',
        name=robot_type+'_tf_updater',
        executable='kinova_tf_updater',
        parameters=[{'base_frame': 'root'}, set_configurable_parameters(configurable_parameters), params_from_file],
        remappings=[(robot_type+'_tf_updater/in/joint_angles', robot_type+'_driver/out/joint_angles')],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_urdf")),
    )
    
    xacro_file = os.path.join(get_package_share_directory('kinova_description'), 'urdf', robot_type + '_standalone.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        remappings=[('joint_states', robot_type+'_driver/out/joint_state')],
        output='screen',
        parameters=[{'robot_description': robot_desc},],
        condition=IfCondition(LaunchConfiguration("use_urdf")),
    )
    
    return [kinova_driver, kinova_tf_updater, robot_state_publisher]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function = launch_setup)
    ])
