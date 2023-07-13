import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gui_params = os.path.join(get_package_share_directory('gsc'),'config','gui.yaml')

    gui_node = Node(
        package="gsc",
        executable="gsc",
        namespace="master",
        name='gui',
        #parameters=[gui_params],
    )

    joy_node = Node(
            
        package='joy',
        executable='joy_node',
        namespace="master",
        parameters=[gui_params],
    )

    # Add the nodes and the process to the LaunchDescription list
    ld = [
        gui_node,
        joy_node
    ]

    return LaunchDescription(ld)