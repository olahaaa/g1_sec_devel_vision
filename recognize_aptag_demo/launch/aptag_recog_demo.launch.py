import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    apriltag_params = os.path.join(get_package_share_directory("apriltag_ros"),"cfg","tags_36h11.yaml")

    img_sub_node = Node(
        package='recognize_aptag_demo',
        executable='img_sub_node',
        namespace='self_demo',
        name='img_sub_node',
        output='screen',
        emulate_tty=True,
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        # namespace='camera',
        # name='camera',
        parameters=[{
            'enable_color': True
        }],
        output='screen',
        emulate_tty=True,
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[apriltag_params],
        remappings=[('image_rect','/camera/camera/color/image_raw'),
                    ('camera_info','/camera/camera/color/camera_info')],
    )


    # tf_sub_node = Node(
    #     package='recognize_aptag_demo',
    #     executable='tfsub_node',
    #     namespace='self_demo',
    #     name='tfsub_node',
    #     output='screen',
    # )

    return LaunchDescription([
        LogInfo(msg='开始订阅realsense rgb图像'),
        realsense_node,
        img_sub_node,
        apriltag_node,
        # tf_sub_node,
    ])