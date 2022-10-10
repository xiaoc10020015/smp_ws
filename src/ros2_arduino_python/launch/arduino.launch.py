import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription           
from launch_ros.actions import Node            

def generate_launch_description(): 
    arduino_pkg_name = 'ros2_arduino_python'
    config = os.path.join(              
      get_package_share_directory(arduino_pkg_name),
      'config',
      'arduino_params.yaml'
      )

    return LaunchDescription([                 
        Node(                                  
            package=arduino_pkg_name,          
            executable='arduino_node',     
            name='arduino',                 
            parameters=[config]
        )
    ])