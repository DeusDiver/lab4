import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Rectify node from the image_proc package
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        output='screen',
        remappings=[
            ('camera_info', 'image_raw')
        ]
    )

    # Gaussian blur node: remap input from 'image_raw' to 'image_rect',
    # and output from 'output_image' to 'image_blurred'
    gaussian_blur_node = Node(
        package='camera_pipeline',
        executable='gaussian_blur',
        name='gaussian_blur',
        remappings=[
            ('image_raw', 'image_rect'),
            ('output_image', 'image_blurred')
        ]
    )

    # Canny edge node: remap input from 'image_raw' to 'image_blurred'
    canny_edge_node = Node(
        package='camera_pipeline',
        executable='canny_edge',
        name='canny_edge',
        remappings=[
            ('image_raw', 'image_blurred'),
            ('output_image', 'final_image')
        ]
    )

    return LaunchDescription([
        rectify_node,
        gaussian_blur_node,
        canny_edge_node,
    ])
