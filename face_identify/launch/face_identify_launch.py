import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 声明摄像头设备参数（默认 /dev/video0）
    camera_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='usb camera device'
    )

    # 启动 hobot_usb_cam 节点，发布原始 /image 话题
    usb_cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch/hobot_usb_cam.launch.py')
        ),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_video_device': LaunchConfiguration('device')
        }.items()
    )

    # 启动 websocket 节点用于 web 展示（可选）
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')
        ),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )

    # 启动 hobot_codec 解码节点，将 /image 解码并发布到 /hbmem_img
    hobot_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')
        ),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # 启动共享内存节点，提供零拷贝支持
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_shm'),
                'launch/hobot_shm.launch.py')
        )
    )

    # 启动脸部识别节点
    face_identify_node = Node(
        package='originbot_face_identify',
        executable='originbot_face_identify_node',
        name='originbot_face_identify_node',
        output='screen',
        parameters=[
            # 这里可以配置你的参数，例如模型路径等    
            {'model_path': '/userdata/dev_ws/src/originbot/originbot_face_identify/model/yolov8n-face_640x640_nv12.bin'},
            {'model_name': 'yolov8n-face_640x640_nv12'}

        ]
    )

    return LaunchDescription([
        camera_device_arg,
        usb_cam_node,
        web_node,
        hobot_codec_node,
        shared_mem_node,
        face_identify_node
    ])
