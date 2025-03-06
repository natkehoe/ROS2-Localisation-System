from launch import LaunchDescription
from launch_ros.actions import Node

'''
This launch file is used to broadcast the static transform between the world frame and the checkerboard_marker frame,
the checkerboard_marker frame and the checkerboard frame, the checkerboard frame and the camera frame, and the marker_frame
frame and the base_link frame. 

The transform between the world frame and the checkerboard_marker frame is obtained from the
OptiTrack system. The transform between the checkerboard_marker frame and the checkerboard frame is obtained from the
calibration process. The transform between the checkerboard frame and the camera frame is obtained from the calibration
process. The transform between the marker_frame frame and the base_link frame is obtained from the calibration process.

'''

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            #namespace='turtlesim1',
            executable='static_transform_publisher',
            name='optitrack2checkerboard_marker_tf_broadcaster',
            arguments=[
                '-0.02037813700735569',
                '0.023949000984430313',
                '0.4495094120502472',
                '-0.0013561244122684002',
                '0.0009626050014048815',
                '-0.00037052115658298135',
                '-0.9999986290931702',
                'world',       # frame_id
                'checkerboard_marker'  # child_frame_id
            ]
        ),
        
        Node(
            package='tf2_ros',
            #namespace='turtlesim1',
            executable='static_transform_publisher',
            name='checkerboard_marker2checkerboard_tf_broadcaster',
            arguments=[
                '0',
                '-0.014',
                '0',
                '0',
                '-1.57079633',
                '1.57079633',
                'checkerboard_marker',       # frame_id
                'checkerboard'  # child_frame_id
            ]
        ),
        
        Node(
            package='tf2_ros',
            #namespace='turtlesim1',
            executable='static_transform_publisher',
            name='checkerboard2camera_tf_broadcaster',
            arguments=[
                '0.85672',
                '0.73191',
                '-0.74595',
                '0.4633',
                '-0.2706',
                '-0.3857',
                '0.7506',
                'checkerboard',
                'camera'
            ]
        ),
        
        Node(
            package='tf2_ros',
            #namespace='turtlesim1',
            executable='static_transform_publisher',
            name='robot_tf_broadcaster',
            arguments=[
                '-0.194'
                '0'
                '0.01'
                '0'
                '0'
                '0'
                '1'
                'marker_frame'
                'base_link'
            ]
        )
    ])
