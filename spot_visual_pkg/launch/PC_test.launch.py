import os
import sys

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Load the URDF into a parameter
    spot_visual_pkgPath = get_package_share_directory('spot_visual_pkg')
    spot_contorl_pkgPath = get_package_share_directory('spot_control_pkg')
    # spot_contorl_pkgPath = launch_ros.substitutions.FindPackageShare(package='spot_contorl_pkg').find('spot_contorl_pkg')
    urdf_path = os.path.join(spot_contorl_pkgPath, 'urdf', 'Spotmicro.urdf')
    rviz_path = os.path.join(spot_visual_pkgPath, 'config/config.rviz')
    urdf = open(urdf_path, 'r').read()
    print(rviz_path)
    params = {'robot_description': urdf}
    params2 = {'robot_description2': urdf}

    robot_state_publisher_node=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='set',
        remappings=[('tf', 'tf2'), ('tf_static', 'tf2_static')],
        parameters=[params, {'tf_prefix': 'set'}],
        arguments=[urdf_path]
    )

    robot_state_publisher_node2=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # remappings=[('robot_description', 'robot_description2'),
        #             ('joint_states', 'spot_joints'),
        #             ('tf', 'tf2'),
        #             ('tf_static', 'tf2_static')],
        remappings=[('joint_states', 'spot_joints')],
        output='screen',
        namespace='feedback',
        parameters=[params2, {'tf_prefix': 'feedback'}],
        arguments=[urdf_path]
    )

    joint_state_publisher_node=launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdf_path]
    )

    joint_state_publisher_gui_node=launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_path],
        namespace='set',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--d', rviz_path]
    )

    micro_ros_node = launch_ros.actions.Node(\
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agnet',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '230400'],
        output='screen'
        )
    
    spot_control_node = launch_ros.actions.Node(\
        package='spot_control_pkg',
        executable='spot_control_node',
        name='spot_control_node'
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True', description='this'),
        micro_ros_node,
        # robot_state_publisher_node,
        robot_state_publisher_node2,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
        spot_control_node
        
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()