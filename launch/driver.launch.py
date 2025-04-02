from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('device', default_value='/dev/ttyUSB_dynpick'),
    DeclareLaunchArgument('rate', default_value='1000.0'),
    DeclareLaunchArgument('sensor_frame_id', default_value='/force_sensor_link'),
    DeclareLaunchArgument('topic', default_value='/force'),
    DeclareLaunchArgument('frequency_div', default_value='1'),
    DeclareLaunchArgument('acquire_calibration', default_value='true'),

    Node(
      package='dynpick_driver',
      executable='dynpick_driver_node',
      name='dynpick_driver_node',
      parameters=[
        {'device': LaunchConfiguration('device')},
        {'rate': LaunchConfiguration('rate')},
        {'frame_id': LaunchConfiguration('sensor_frame_id')},
        {'frequency_div': LaunchConfiguration('frequency_div')},
        {'acquire_calibration': LaunchConfiguration('acquire_calibration')}
      ],
      remappings=[
        ('/force', LaunchConfiguration('topic'))
      ]
    )
  ])
