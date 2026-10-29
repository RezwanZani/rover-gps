from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- LAUNCH ARGUMENTS ----------------
    xbee_port_arg = DeclareLaunchArgument(
        'xbee_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the XBee RTCM receiver'
    )

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the LC29H RTK GPS module'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='true',
        description='Set true to run rtk_bridge_node (direct XBee serial injection). Set false to run gps_node (ROS /rtcm topic).'
    )

    enable_plot_arg = DeclareLaunchArgument(
        'enable_plot',
        default_value='false',
        description='Set true to launch the live RTK drift visualizer GUI (map_acc)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps_link',
        description='TF frame_id for NavSatFix messages'
    )

    # ---------------- NODES ----------------

    # Node 1: Main RTK Bridge Node (XBee serial -> GPS serial + NavSatFix /fix)
    rtk_bridge_node = Node(
        package='gps_data',
        executable='rtk_bridge_node',
        name='rtk_bridge_node',
        output='screen',
        parameters=[{
            'xbee_port': LaunchConfiguration('xbee_port'),
            'gps_port': LaunchConfiguration('gps_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }],
        condition=IfCondition(LaunchConfiguration('use_bridge'))
    )

    # Node 2: Standalone GPS Node (subscribes to ROS topic /rtcm -> GPS serial -> /fix)
    gps_node = Node(
        package='gps_data',
        executable='gps_node',
        name='rover_gps_node',
        output='screen',
        parameters=[{
            'gps_port': LaunchConfiguration('gps_port'),
            'gps_baud': LaunchConfiguration('baud_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        condition=UnlessCondition(LaunchConfiguration('use_bridge'))
    )

    # Node 3: Optional Live Drift Visualizer GUI (subscribes to /fix)
    drift_plot_node = Node(
        package='gps_data',
        executable='map_acc',
        name='drift_plot',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_plot'))
    )

    return LaunchDescription([
        xbee_port_arg,
        gps_port_arg,
        baud_rate_arg,
        use_bridge_arg,
        enable_plot_arg,
        frame_id_arg,
        rtk_bridge_node,
        gps_node,
        drift_plot_node,
    ])
