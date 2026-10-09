import os
import math
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction, OpaqueFunction, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from threading import Event
import tempfile

robot_name = "ic120"
use_autostart = True
use_sim_time = True
use_respawn = True
use_namespace = True
common_prefix_val = ""
tf_prefix_val = ""

opaque_function_complete_event = Event()

def retrieve_values(context, *args, **kwargs):
    global common_prefix_val, tf_prefix_val
    common_prefix_val = LaunchConfiguration('common_prefix').perform(context)
    tf_prefix_val = common_prefix_val + '_tf'
    opaque_function_complete_event.set()
    return []

def wait_for_opaque_function(context):
    opaque_function_complete_event.wait()
    return []

def rewrite_nav_params(context, **kwargs):
    global configured_params
    ic120_navigation_dir = get_package_share_directory('ic120_navigation')
    navigation_parameters_sim_yaml_file = os.path.join(ic120_navigation_dir, 'params', 'navigation_parameters_sim.yaml')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(ic120_navigation_dir, 'map', 'map.yaml'))

    param_substitutions_nav = {
        'use_sim_time': str(use_sim_time),
        'yaml_filename': map_yaml_file,
        'robot_base_frame': tf_prefix_val + '/base_link',

        # amcl
        'amcl.ros__parameters.base_frame_id': tf_prefix_val+'/base_link',
        'amcl.ros__parameters.odom_frame_id': tf_prefix_val+'/odom',

        #component_container_isolated
        'component_container_isolated.ros__parameters.autostart': str(use_autostart),
        
        # bt_navigator
        'bt_navigator.ros__parameters.robot_base_frame': tf_prefix_val+'/base_link',
        'bt_navigator.ros__parameters.odom_topic': '/'+common_prefix_val+'/odom_nav',
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(ic120_navigation_dir, 'params', 'ic120_navigate_to_pose_w_replanning_and_recovery.xml'),
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': os.path.join(ic120_navigation_dir, 'params', 'ic120_navigate_through_poses_w_replanning_and_recovery.xml'),
        # 'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': os.path.join(ic120_navigation_dir, 'params', 'ic120_navigate_once.xml'),

        # controller_server
        'controller_server.ros__parameters.odom_topic': '/'+common_prefix_val+'/odom_nav',
        'controller_server.ros__parameters.base_global_frame': 'map',

        # local costmap
        'local_costmap.local_costmap.ros__parameters.global_frame': 'map',
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': tf_prefix_val+'/base_link',

        # global costmap                        
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': tf_prefix_val+'/base_link',

        # behavior server
        'behavior_server.ros__parameters.global_frame': 'map',
        'behavior_server.ros__parameters.robot_base_frame': tf_prefix_val+'/base_link',

        # velocity smoother
        'velocity_smoother.ros__parameters.odom_topic': '/'+common_prefix_val+'/odom_nav',
    }
    configured_params=RewrittenYaml(
        source_file=navigation_parameters_sim_yaml_file,
        root_key=common_prefix_val,
        param_rewrites=param_substitutions_nav,
        convert_types=True
    )


def rewrite_ekf_params(context, **kwargs):
    global configured_ekf_params
    ic120_navigation_dir = get_package_share_directory('ic120_navigation')
    ic120_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(ic120_navigation_dir, 'config', 'ic120_ekf.yaml'))
    configured_ekf_params=RewrittenYaml(
        source_file=ic120_ekf_yaml_file,
        root_key=common_prefix_val,
        param_rewrites={'use_sim_time': str(use_sim_time)},
        convert_types=True
    )

def process_xacro(context, *args, **kwargs):
    global params
    ic120_description_dir = get_package_share_directory("ic120_description")
    ic120_xacro_file = os.path.join(ic120_description_dir, "urdf", "ic120_sim.xacro")
    with open(ic120_xacro_file, 'r') as file:
        filedata = file.read()
    filedata = filedata.replace('<xacro:property name="tf_prefix" value="ic120_tf"/>', f'<xacro:property name="tf_prefix" value="{tf_prefix_val}"/>')
    with tempfile.NamedTemporaryFile('w+', delete=False) as temp_file:
        temp_file.write(filedata)
        temp_file_path = temp_file.name
    doc = xacro.parse(open(temp_file_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    return []



def generate_nodes(context, *args, **kwargs):
    opaque_function_complete_event.wait()
    ic120_unity_dir = get_package_share_directory("ic120_unity")
    ic120_standby_rviz_file = os.path.join(ic120_unity_dir, "rviz2", "ic120_standby.rviz")

    lifecycle_nodes_localization = [
        'map_server'
    ]
    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]    
    startup_interval = float(LaunchConfiguration('startup_interval').perform(context))
    if not math.isfinite(startup_interval) or not 0.0 <= startup_interval <= 10.0:
        raise ValueError('startup_interval must be finite and between 0 and 10 seconds')
    nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=common_prefix_val,
            name='world_to_map',
            arguments=['--x', '0', 
                        '--y', '0', 
                        '--z', '0', 
                        '--roll', '0', 
                        '--pitch', '0', 
                        '--yaw', '0', 
                        '--frame-id', 'world',
                        '--child-frame-id', 'map']),
        Node(
            package='ic120_navigation',
            executable='odom_broadcaster',
            namespace=common_prefix_val,
            name='odom_broadcaster',
            output="screen",
            parameters=[{'odom_topic': '/'+common_prefix_val+'/odom_pose'},
                        {'odom_frame': tf_prefix_val+ "/odom"},
                        {'base_link_frame': tf_prefix_val + "/base_link"},
                        {'publish_frequency': 20.0},
                        {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='ic120_navigation',
            executable='pose_velocity_odometry',
            namespace=common_prefix_val,
            name='pose_velocity_odometry',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{'input_topic': '/'+common_prefix_val+'/odom_pose',
                         'output_topic': '/'+common_prefix_val+'/odom_nav',
                         'base_frame': tf_prefix_val+'/base_link',
                         'publish_frequency': 20.0,
                         'use_sim_time': use_sim_time}]),
        # Node(
        #     package='ic120_navigation',
        #     executable='poseStamped2Odometry',
        #     namespace=common_prefix_val,
        #     name='poseStamped2ground_truth_odom',
        #     output="screen",
        #     parameters=[{'odom_header_frame': "world",
        #                     'odom_child_frame': tf_prefix_val+"/base_link",
        #                     'poseStamped_topic_name': '/'+common_prefix_val+"/base_link/pose",
        #                     'odom_topic_name': '/'+common_prefix_val+"/tracking/ground_truth",
        #                     'use_sim_time': use_sim_time}]
        # ),            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=common_prefix_val,
            output="screen",
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        #########################
        # Localization packages #
        #########################
        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=common_prefix_val,
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params]),
        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace=common_prefix_val,
            name="ekf_global",
            output="screen",
            remappings=[('odometry/filtered', '/'+common_prefix_val+'/odometry/global'),
                        ('odom0', '/'+common_prefix_val+'/odom_nav'),
                        ('odom1','/'+common_prefix_val+"/global_pose")],
            parameters=[configured_ekf_params,
                        {'map_frame': "map",
                            'world_frame': "map",
                            'odom_frame': tf_prefix_val+"/odom",
                            'base_link_frame': tf_prefix_val+"/base_link",
                            'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=common_prefix_val,
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': use_autostart},
                        {'node_names': lifecycle_nodes_localization}]),

        #######################
        # Navigation packages #
        #######################
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=common_prefix_val,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            namespace=common_prefix_val,
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace=common_prefix_val,
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params]),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace=common_prefix_val,
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel')]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace=common_prefix_val,
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[(common_prefix_val+'/goal_pose', 'goal_pose')]),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            namespace=common_prefix_val,
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params]),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            namespace=common_prefix_val,
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav'), 
                        ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=common_prefix_val,
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': use_autostart},
                        {'node_names': lifecycle_nodes_navigation}]),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package="rviz2",
            executable="rviz2",
            namespace=common_prefix_val,
            name="rviz",
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=["--display-config", ic120_standby_rviz_file]),
        
    ]
    use_composition = LaunchConfiguration('use_composition').perform(context).lower() == 'true'
    if use_composition:
        # Keep the Python pose adapters and EKF standalone. Nav2 components
        # share a DDS participant, with an isolated executor per component.
        nodes = nodes[:4] + [nodes[5], nodes[-1]]
        nodes.insert(5, Node(
            package='rclcpp_components', executable='component_container_isolated',
            name='nav2_container', namespace=common_prefix_val, output='screen',
            # Dynamically-created costmap subnodes read the process-wide YAML,
            # whereas component parameter overrides select only the parent.
            parameters=[configured_params]))
        specifications = [
            ('nav2_map_server', 'nav2_map_server::MapServer', 'map_server', []),
            ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
            ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server', []),
            ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', []),
            ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', []),
            ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', [(common_prefix_val+'/goal_pose', 'goal_pose')]),
            ('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower', 'waypoint_follower', []),
            ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        ]
        components = [ComposableNode(package=package, plugin=plugin, name=name,
                                    namespace=common_prefix_val, parameters=[configured_params],
                                    remappings=remappings)
                      for package, plugin, name, remappings in specifications]
        for name, lifecycle_nodes in [('lifecycle_manager_localization', lifecycle_nodes_localization),
                                      ('lifecycle_manager_navigation', lifecycle_nodes_navigation)]:
            components.append(ComposableNode(
                package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                name=name, namespace=common_prefix_val,
                parameters=[{'use_sim_time': use_sim_time, 'autostart': use_autostart,
                             'node_names': lifecycle_nodes}]))
        actions = [TimerAction(period=index * startup_interval, actions=[node])
                   for index, node in enumerate(nodes)]
        actions.append(TimerAction(period=6 * startup_interval, actions=[LoadComposableNodes(
            target_container='/'+common_prefix_val+'/nav2_container',
            composable_node_descriptions=components)]))
        return actions
    # Spread standalone process initialization and DDS discovery over time.
    return [TimerAction(period=index * startup_interval, actions=[node])
            for index, node in enumerate(nodes)]


def generate_launch_description():
    common_prefix = LaunchConfiguration('common_prefix')
    use_rviz = LaunchConfiguration('use_rviz')
    common_prefix_arg = DeclareLaunchArgument('common_prefix',default_value='ic120_0')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='True')
    startup_interval_arg = DeclareLaunchArgument('startup_interval', default_value='2.0')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='True')

    return LaunchDescription([
        common_prefix_arg,
        use_rviz_arg,
        startup_interval_arg,
        use_composition_arg,
        # Keep numerical libraries used by child processes from oversubscribing
        # CPU cores. ROS executor and controller update rates are unchanged.
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        OpaqueFunction(function=retrieve_values),
        OpaqueFunction(function=wait_for_opaque_function),
        OpaqueFunction(function=rewrite_nav_params),
        OpaqueFunction(function=rewrite_ekf_params),
        OpaqueFunction(function=process_xacro),
        GroupAction([
            OpaqueFunction(function=generate_nodes),
        ])
    ])
