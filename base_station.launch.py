from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['./base_station.sh'], name='base_station_process', output='screen', shell=True),
        Node(
            package='joy',
            namespace='/Basestation',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {'dev': '/dev/input/js0'},
                {'coalesce_interval_ms': 10},
                {'autorepeat_rate': 20.0}
            ],
        )
    ])
