from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['./fake_blimp.sh BurnCreamBlimp'], name='fake_blimp_1', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh SillyAhBlimp'], name='fake_blimp_2', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh TurboBlimp'], name='fake_blimp_3', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh GameChamberBlimp'], name='fake_blimp_4', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh FiveGuysBlimp'], name='fake_blimp_5', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh SuperBeefBlimp'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Yoshi'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Luigi'], name='fake_blimp_6', output='screen', shell=True),
        ExecuteProcess(cmd=['./fake_blimp.sh Geoph'], name='fake_blimp_6', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh ThisGuy'], name='fake_blimp_6', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh Attack1'], name='fake_blimp_6', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh Attack2'], name='fake_blimp_6', output='screen', shell=True),
        # ExecuteProcess(cmd=['./fake_blimp.sh AdamBallGrabber'], name='fake_blimp_6', output='screen', shell=True)
    ])
