

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # create a fake tf tree
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '-1.5708', '0', '1.5708', 'camera_optical_frame', 'camera_frame']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.215', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_optical_frame']),
            

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': False,
                'queue_size': 10
            }],
            remappings=[
                ('rgb/image', '/camera/rgb/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/rgb/image_raw/camera_info')
            ]
        ),
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{
                'subscribe_rgbd': True,
                'frame_id': 'base_link',
                'publish_tf': False,
                'publish_null_when_lost': False,
            }]),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'subscribe_odom': True,
                # 'subscribe_imu': True,
                'approx_sync': True,
            }],
            # remappings=[
            #     ('odom', '/unity_odom')
            # ],
            arguments=['-d']),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
            'frame_id':'base_link',
            # 'use_sim_time':True,
            'subscribe_depth':True 
            }],
            remappings=[
                ('rgb/image', '/camera/rgb/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
            ]
            ),
    ])
