"""
BMHS Path Planner — ROS2 Launch File
Launches: bmhs_planner_node, path_follower_node, bmhs_navigator_node, rviz2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('bmhs_planner_ros')

    # ─── Launch arguments ─────────────────────────────────────────────────
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(pkg_dir, 'maps', 'map_proj.pgm'),
        description='Full path to the PGM map file'
    )
    resolution_arg = DeclareLaunchArgument(
        'map_resolution', default_value='0.05',
        description='Map resolution in metres/pixel'
    )
    origin_x_arg = DeclareLaunchArgument(
        'map_origin_x', default_value='0.0',
        description='Map origin X in metres'
    )
    origin_y_arg = DeclareLaunchArgument(
        'map_origin_y', default_value='0.0',
        description='Map origin Y in metres'
    )
    vtype_arg = DeclareLaunchArgument(
        'vehicle_type', default_value='differential',
        description='Vehicle type: differential or ackermann'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/a200_0000/odom',
        description='Odometry topic (Husky namespace)'
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/a200_0000/cmd_vel',
        description='Velocity command topic (Husky namespace)'
    )

    # ─── BMHS Planner Node (Action Server: ComputePathToPose) ─────────────
    planner_node = Node(
        package='bmhs_planner_ros',
        executable='bmhs_planner_node',
        name='bmhs_planner_node',
        output='screen',
        parameters=[{
            'map_path':          LaunchConfiguration('map_path'),
            'map_resolution':    LaunchConfiguration('map_resolution'),
            'map_origin_x':      LaunchConfiguration('map_origin_x'),
            'map_origin_y':      LaunchConfiguration('map_origin_y'),
            'vehicle_type':      LaunchConfiguration('vehicle_type'),
            'vehicle_width':     0.67,   # Husky width
            'vehicle_length':    0.99,   # Husky length
            'inflation_radius':  0.3,
            'frame_id':          'map',
            'odom_topic':        LaunchConfiguration('odom_topic'),
        }],
    )

    # ─── Path Follower Node (Action Server: FollowPath) ───────────────────
    follower_node = Node(
        package='bmhs_planner_ros',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[{
            'lookahead_distance': 0.8,
            'max_linear_vel':     0.5,
            'max_angular_vel':    1.0,
            'goal_tolerance':     0.3,
            'odom_topic':         LaunchConfiguration('odom_topic'),
            'cmd_vel_topic':      LaunchConfiguration('cmd_vel_topic'),
            'control_rate':       20.0,
        }],
    )

    # ─── Navigator Node (Coordinator) ─────────────────────────────────────
    navigator_node = Node(
        package='bmhs_planner_ros',
        executable='bmhs_navigator_node',
        name='bmhs_navigator_node',
        output='screen',
        parameters=[{
            'goal_topic': '/goal_pose',
        }],
    )

    # ─── RViz2 ────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        map_path_arg,
        resolution_arg,
        origin_x_arg,
        origin_y_arg,
        vtype_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        planner_node,
        follower_node,
        navigator_node,
        rviz_node,
    ])
