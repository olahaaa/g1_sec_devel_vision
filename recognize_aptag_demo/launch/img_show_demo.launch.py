from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    img_sub_node = Node(
        package='recognize_aptag_demo',
        executable='img_sub_node',
        name='self_img_sub_node',
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

    return LaunchDescription([
        LogInfo(msg='开始订阅realsense rgb图像'),
        realsense_node,
        img_sub_node,
    ])