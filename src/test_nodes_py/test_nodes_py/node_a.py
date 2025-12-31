#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from test_interfaces.msg import Position
from test_interfaces.srv import CalculateDistance
from test_interfaces.action import MoveTo


class NodeA(Node):
    def __init__(self):
        super().__init__('node_a')
        
        # Topic Publisher
        self.topic_publisher = self.create_publisher(Position, 'position_topic', 10)
        
        # Topic Subscriber (추가!)
        self.topic_subscriber = self.create_subscription(
            Position, 'position_topic', self.topic_callback, 10)
        
        # Service Client
        self.service_client = self.create_client(CalculateDistance, 'calculate_distance')
        
        # Action Client
        self.action_client = ActionClient(self, MoveTo, 'move_to_action')
        
        # Variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.angle = 0.0
        self.t = 0.0
        
        self.get_logger().info('Node A started (manual mode)')
    
    # 여기서부터 함수들이 __init__ 밖!
    def publish_position(self):
        self.x = 5.0 * math.cos(self.t)
        self.y = 5.0 * math.sin(self.t)
        self.z = self.t * 0.1
        self.angle = self.t * 180.0 / math.pi
        self.t += 0.2
        
        msg = Position()
        msg.sender = 'NodeA'
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.angle = self.angle
        msg.timestamp = self.get_clock().now().nanoseconds
        
        self.get_logger().info(
            f'[TOPIC] Position: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, angle={self.angle:.2f}')
        self.topic_publisher.publish(msg)
    
    def topic_callback(self, msg):
        if msg.sender != 'NodeA':
            self.get_logger().info(
                f'[TOPIC] Received from {msg.sender}: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, angle={msg.angle:.2f}')
    
    def call_service(self):
        if not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service server not available')
            return
        
        request = CalculateDistance.Request()
        request.x1 = self.x
        request.y1 = self.y
        request.z1 = self.z
        request.x2 = 10.0
        request.y2 = 10.0
        request.z2 = 5.0
        
        self.get_logger().info(
            f'[SERVICE] Request: ({request.x1:.2f},{request.y1:.2f},{request.z1:.2f}) to ({request.x2:.2f},{request.y2:.2f},{request.z2:.2f})')
        
        future = self.service_client.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'[SERVICE] Response: {response.message} (distance: {response.distance:.2f}m)')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def send_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available')
            return
        
        goal_msg = MoveTo.Goal()
        goal_msg.target_x = 10.0
        goal_msg.target_y = 5.0
        goal_msg.target_z = 2.0
        goal_msg.target_angle = 90.0
        
        self.get_logger().info(
            f'[ACTION] Sending goal: ({goal_msg.target_x:.2f}, {goal_msg.target_y:.2f}, {goal_msg.target_z:.2f}, {goal_msg.target_angle:.2f})')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[ACTION] Goal rejected')
            return
        
        self.get_logger().info('[ACTION] Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[ACTION] Feedback: ({feedback.current_x:.2f}, {feedback.current_y:.2f}, {feedback.current_z:.2f}, {feedback.current_angle:.2f}) - {feedback.progress_percent:.1f}%')
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'[ACTION] Result: {result.status} - final({result.final_x:.2f}, {result.final_y:.2f}, {result.final_z:.2f}, {result.final_angle:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = NodeA()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
