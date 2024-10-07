import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    # robot
    robot_name = 'vampire'

    # param path
    param_path = os.path.join(
        get_package_share_directory('mvp_acomms'),
        'config'
        )
    
    # different param
    evologics_param_file = os.path.join(param_path, 'evologics.yaml') 

    seatrac_param_file = os.path.join(param_path, 'seatrac.yaml') 

    goby_param_file = os.path.join(param_path, 'goby.yaml') 

    # launch the node
    return LaunchDescription([

        TimerAction(period=0.0,
            actions=[
                    Node(
                        package="mvp_acomms",
                        executable="modem_node",
                        namespace=robot_name,
                        name="modem_node",
                        prefix=['stdbuf -o L'],
                        output="screen",
                        parameters=[
                            evologics_param_file,
                            # seatrac_param_file,
                            goby_param_file
                        ]
                    )
            ])
        
])