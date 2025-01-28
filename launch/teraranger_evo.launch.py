from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('teraranger_evo').find('teraranger_evo')
    
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for TeraRanger Evo'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='teraranger_evo_frame',
        description='Frame ID for range measurements'
    )

    set_permissions_arg = DeclareLaunchArgument(
        'set_permissions',
        default_value='true',
        description='Whether to set serial port permissions'
    )

    # Set up serial port permissions
    setup_permission = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('set_permissions')),
        cmd=['sudo', 'chmod', 'a+rw', LaunchConfiguration('port')],
        name='setup_permission',
        output='screen'
    )

    # Add a small delay to ensure permissions are set
    delay_after_permission = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='teraranger_evo',
                executable='teraranger_evo_node',
                name='teraranger_evo',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'portname': LaunchConfiguration('port'),
                    'frame_id': LaunchConfiguration('frame_id')
                }],
                arguments=['--ros-args', '--log-level', 'info'],
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        port_arg,
        frame_id_arg,
        set_permissions_arg,
        
        # Actions
        setup_permission,
        delay_after_permission
    ])