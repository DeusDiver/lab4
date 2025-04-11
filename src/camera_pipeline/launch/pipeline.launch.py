from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Image rectification node
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ]
        ),
        
        # USB camera node with calibration file
#        Node(
#            package='usb_cam',
#            executable='usb_cam_node_exe',
#            name='usb_cam_node',
#            parameters=[
#                {"video_device": "/dev/video0"},
#            ]
#        ),
        
        # Gaussian blur node
        Node(
            package='camera_pipeline',
            executable='gaussian_blur',
            name='gaussian_blur_node',
            remappings=[
                ('image_raw', 'image_rect'), 
                ('output_image', 'image_blurred')  # Publish to blurred topic
            ]
        ),
        
        # Canny edge node
        Node(
            package='camera_pipeline',
            executable='canny_edge',
            name='canny_edge_node',
            remappings=[
                ('image_raw', 'image_blurred'),  # Listen to blurred image
                ('output_image', 'image_output')  # Final output
            ]
        )
    ])