from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    share = Path(get_package_share_directory('rawseeds_slam'))
    mode = get('mode')
    if mode not in ('data', 'student', 'demo'):
        raise ValueError('mode must be data, student, or demo')
    use_map = mode != 'data'
    nodes = []
    geometry = yaml.safe_load(Path(get('sensors')).read_text())
    for side in ('front', 'rear'):
        g = geometry[side]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name=f'{side}_extrinsic', arguments=[
                              '--x', str(g['x']), '--y', str(g['y']), '--z', str(g['z']),
                              '--yaw', str(g['yaw']), '--pitch', '0', '--roll', '0',
                              '--frame-id', 'base_link', '--child-frame-id', f'laser_{side}'],
                          parameters=[{'use_sim_time': True}], output='screen'))
    if mode in ('student', 'demo'):
        nodes.append(Node(package='rawseeds_slam', executable='slam_mapper',
                          parameters=[{'use_sim_time': True,
                                       'algorithm': 'student' if mode == 'student' else 'demo'}],
                          output='screen'))
    nodes.append(Node(package='rawseeds_slam', executable='trajectory',
                      parameters=[{'use_sim_time': True,
                                   'frame': 'map' if use_map else 'odom',
                                   'output_csv': get('trajectory')}],
                      output='screen'))
    if get('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share / 'rviz' / ('mapping.rviz' if use_map else 'data.rviz'))],
                          parameters=[{'use_sim_time': True}], output='screen'))
    bag = get('bag')
    if bag:
        bag = str(Path(bag).expanduser().resolve())
        if not (Path(bag) / 'metadata.yaml').is_file():
            raise ValueError(f'Bag directory has no metadata.yaml: {bag}')
        rate = float(get('rate'))
        if rate <= 0:
            raise ValueError('rate must be positive')
        nodes.append(TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '100', '--rate', str(rate),
                 '--qos-profile-overrides-path', str(share / 'config' / 'playback_qos.yaml')],
            output='screen')]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('rawseeds_slam'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='data'),
        DeclareLaunchArgument('bag', default_value='', description='ROS 2 bag directory'),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('trajectory', default_value='', description='Optional new CSV path'),
        DeclareLaunchArgument('sensors', default_value=str(share / 'config' / 'sensors.yaml')),
        OpaqueFunction(function=start)])
