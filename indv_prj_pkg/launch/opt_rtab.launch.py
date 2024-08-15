import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{
                'frame_id': 'camera_frame',
                # 'subscribe_rgbd': True,
                'approx_sync': True,
                'use_sim_time': True,
                'queue_size': 10
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('imu', '/camera/imu')
            ]
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id': 'camera_frame',
                # 'subscribe_rgbd': True,
                'approx_sync': True,
                'subscribe_odom_info': True,
                'use_sim_time': True,
                'queue_size': 10
            }],
            remappings=[
                ('rgb/image', '/camera/color/image'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/image/camera_info'),
                ('odom', 'odom'),
                ('odom_info', 'odom_info')
            ]
        ),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                'frame_id': 'camera_frame',
                'use_sim_time': False
            }],
            remappings=[
                ('rgb/image', '/camera/color/image'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/image/camera_info')
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
