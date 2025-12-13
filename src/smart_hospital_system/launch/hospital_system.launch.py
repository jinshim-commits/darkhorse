from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # QR API 서버
        Node(
            package='smart_hospital_system',
            executable='qr_api_node',
            output='screen'
        ),

        # 환자 UI만 실행
        Node(
            package='smart_hospital_system',
            executable='patient_ui',
            output='screen'
        ),

        # 로봇 이동 + 도착 신호 담당
        Node(
            package='smart_dispatcher',
            executable='dispatcher_node',
            output='screen'
        ),
    ])
