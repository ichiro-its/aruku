from launch import LaunchDescription
from launch_ros.actions import Node;

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruku',
            namespace='aruku',
            executable='main',
            name='aruku',
            arguments=['src/aruku/data/'],
        ),
        Node(
            package='tachimawari',
            namespace='tachimawari',
            executable='main',
            name='tachimawari',
        )
    ])
