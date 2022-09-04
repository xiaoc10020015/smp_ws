from launch import LaunchDescription           
from launch_ros.actions import Node            

def generate_launch_description(): 
    voice_pkg_name = 'voice'

    return LaunchDescription([                 
        Node(                                  
            package=voice_pkg_name,          
            executable='iat_publish',
            namespace='qianxun',       
            name='iat_publish',
            output='screen'                 
        ),
        Node(                                  
            package=voice_pkg_name,          
            executable='voice_assistant',
            namespace='qianxun',
            name='voice_assistant',
            output='screen'       
        )
    ])