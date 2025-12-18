#!/usr/bin/env python3
"""
Simulated Magnetic Gripper Node
Provides logical ON/OFF gripper control without physics simulation
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


class SimulatedGripper(Node):
    """
    Simulates a magnetic gripper for bin attachment/detachment
    State transitions: OFF → ON (attach) | ON → OFF (release)
    """
    
    def __init__(self):
        super().__init__('simulated_gripper')
        
        self.gripper_state = False  # False = OFF, True = ON
        
        # Service: Gripper control
        self.control_service = self.create_service(
            SetBool,
            '/gripper_control',
            self.handle_gripper_control
        )
        
        # Publisher: Gripper state
        self.state_pub = self.create_publisher(Bool, '/gripper_state', 10)
        
        # State publishing timer
        self.timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info("Simulated Gripper Node initialized")
        self.get_logger().info("Initial state: OFF")
    
    def handle_gripper_control(self, request, response):
        """
        Handle gripper ON/OFF requests
        
        Args:
            request.data: True = engage (ON), False = release (OFF)
        """
        requested_state = request.data
        action = "ENGAGE" if requested_state else "RELEASE"
        
        # Simulate brief actuation delay
        # In real system: check proximity to bin, validate position, etc.
        
        previous_state = self.gripper_state
        self.gripper_state = requested_state
        
        if previous_state != self.gripper_state:
            self.get_logger().info(
                f"Gripper state changed: "
                f"{'OFF' if not previous_state else 'ON'} → "
                f"{'ON' if self.gripper_state else 'OFF'}"
            )
        
        response.success = True
        response.message = f"Gripper {action} successful"
        
        self.get_logger().info(f"✓ {response.message}")
        
        return response
    
    def publish_state(self):
        """Publish current gripper state"""
        msg = Bool()
        msg.data = self.gripper_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedGripper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Gripper Simulator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
