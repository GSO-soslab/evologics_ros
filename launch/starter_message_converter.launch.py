import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():

    # robot
    robot_name = 'test_robot'

    # param path
    param_path = os.path.join(
        get_package_share_directory('evologics_ros'),
        'config'
        )
    
    # different param
    evologics_param_file = os.path.join(param_path, 'evologics_usbl1.yaml') 

    goby_param_file = os.path.join(param_path, 'goby.yaml') 

    # launch the node
    return LaunchDescription([

        TimerAction(period=0.0,
            actions=[
                    Node(
                        package="evologics_ros",
                        executable="evologics_ros_message_converter_node",
                        namespace=robot_name,
                        name="evologics_ros_message_converter_node",
                        prefix=['stdbuf -o L'],
                        output="screen",
                        parameters=[
                        {'subbufer_id': "mvp_c2"},
                        {'frame_id': "test"},
                        ],
                        remappings=[
                            ('message_converter/to_acomm_data', 'usbl/tx_bytearray'),
                            ('message_converter/to_c2_data', 'mvp_c2/traffic_control/dccl_msg_rx'),
                            ('message_converter/data_from_c2', 'mvp_c2/traffic_control/dccl_msg_controlled_tx'),
                            ('message_converter/data_from_acomm', 'usbl/rx_bytearray'),
                            ]

                    )
            ])
        
])