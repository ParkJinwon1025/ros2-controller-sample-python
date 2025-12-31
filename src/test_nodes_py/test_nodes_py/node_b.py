#!/usr/bin/env python3
import rclpy # Python에서 ROS2를 사용하기 위한 라이브러리
from rclpy.node import Node
from rclpy.action import ActionServer

import math
import time

from test_interfaces.msg import Position
from test_interfaces.srv import CalculateDistance
from test_interfaces.action import MoveTo

class NodeB(Node):
    def __init__(self):
        super().__init__('node_b')

        # Topic Publisher
        self.topic_publisher = self.create_publisher(Position, 'position_topic', 10)

        # Topic Subscriber
        self.topic_subscriber = self.create_subscription(  # ✅ 수정됨
            Position, 'position_topic', self.topic_callback, 10)
        
        # Service Server
        self.service_server = self.create_service(  # ✅ 수정됨 (sever → server)
            CalculateDistance, 'calculate_distance', self.handle_service)


        # Action Server
        self.action_server = ActionServer(
            self, MoveTo, 'move_to_action', self.execute_callback)
        

        # Variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.angle = 0.0
        self.t = 0.0

        self.get_logger().info('Node B started (manual mode)')

    def publish_position(self):
        self.x = self.t
        self.y = self.t * 0.5
        self.z = 1.0
        self.angle = 45.0
        self.t += 1.0

        msg = Position()
        msg.sender = 'NodeB'
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z 
        msg.angle = self.angle
        msg.timestamp = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f'[TOPIC] Position: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, angle={self.angle:.2f}')

        self.topic_publisher.publish(msg)

    def topic_callback(self, msg):
        if msg.sender != 'NodeB':
            self.get_logger().info(
                f'[TOPIC] Received from {msg.sender}: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, angle={msg.angle:.2f}')
    
    # 요청 받아서 거리 계산 후 응답 보내기
    def handle_service(self, request, response):
        self.get_logger().info(
            f'[SERVICE] Request received: ({request.x1:.2f},{request.y1:.2f},{request.z1:.2f}) to ({request.x2:.2f},{request.y2:.2f},{request.z2:.2f})')

        dx = request.x2 - request.x1
        dy = request.y2 - request.y1
        dz = request.z2 - request.z1

        response.distance = math.sqrt(dx*dx + dy*dy+ dz*dz)
        response.message = 'Distance calculated'

        self.get_logger().info(f'[SERVICE] Response: {response.distance:.2f}m')  # ✅ SEVICE → SERVICE 수정
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info('[ACTION] Executing')
        
        goal = goal_handle.request
        feedback = MoveTo.Feedback()
        
        start_x = self.x
        start_y = self.y
        start_z = self.z
        start_angle = self.angle
        
        steps = 10
        for i in range(steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveTo.Result()
                result.status = 'Canceled'
                result.final_x = self.x
                result.final_y = self.y
                result.final_z = self.z
                result.final_angle = self.angle
                self.get_logger().info('[ACTION] Canceled')
                return result
            
            progress = i / steps
            
            self.x = start_x + (goal.target_x - start_x) * progress
            self.y = start_y + (goal.target_y - start_y) * progress
            self.z = start_z + (goal.target_z - start_z) * progress
            self.angle = start_angle + (goal.target_angle - start_angle) * progress
            
            feedback.current_x = self.x
            feedback.current_y = self.y
            feedback.current_z = self.z
            feedback.current_angle = self.angle
            feedback.progress_percent = progress * 100.0
            
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(
                f'[ACTION] Progress: {progress * 100.0:.1f}% ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.angle:.2f})')
            
            time.sleep(0.5)
        
        goal_handle.succeed()
        
        result = MoveTo.Result()
        result.status = 'Completed'
        result.final_x = self.x
        result.final_y = self.y
        result.final_z = self.z
        result.final_angle = self.angle
        
        self.get_logger().info('[ACTION] Completed')
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = NodeB()
    rclpy.spin(node)  # ✅ 수정됨 (Node → node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
