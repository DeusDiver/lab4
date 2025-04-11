import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Gaussian blur node: remap input from 'image_raw' to 'image_rect', and output to 'image_blurred'
    gaussian_blur_node = Node(
        package='camera_pipeline',
        executable='gaussian_blur',  # ensure the entry_point in setup.py for gaussian_blur is correct
        name='gaussian_blur',
        remappings=[
            ('image_raw', 'image_rect'),
            ('output_image', 'image_blurred')
        ]
    )

    # Canny edge node: remap input from 'image_raw' to 'image_blurred'
    canny_edge_node = Node(
        package='camera_pipeline',
        executable='canny_edge',  # double-check your setup.py: update 'canny_edge' if it mistakenly points to gaussian_blur
        name='canny_edge',
        remappings=[
            ('image_raw', 'image_blurred')
        ]
    )

    return LaunchDescription([
        gaussian_blur_node,
        canny_edge_node,
    ])
