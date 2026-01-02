from launch import LaunchDescription
from launch_ros.actions import Node

# 이름 고정
def generate_launch_description():
    return LaunchDescription([
        # Node A 실행
        Node(
            package='test_nodes_py', # 패키지 이름
            executable='node_a', # 패키지 안에서 실행할 실행 파일 이름
            name="node_a", # 그래프 상에서 보이는 노드 이름
            output='screen'
        ),

        # Node B 실행
        Node (
            package='test_nodes_py',
            executable='node_b',
            name='node_b',
            output='screen'
        ),
    ])