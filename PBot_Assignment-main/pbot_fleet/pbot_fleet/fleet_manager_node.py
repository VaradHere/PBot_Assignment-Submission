#!/usr/bin/env python3
"""
Fleet Manager Node - Order dispatching and queue management
Simulates external warehouse management system
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from collections import deque
import uuid
import math
import yaml
import os

from ament_index_python.packages import get_package_share_directory
from pbot_interfaces.srv import RobotAvailable
from pbot_interfaces.action import TransportOrder
from pbot_interfaces.srv import SubmitOrder



class FleetManager(Node):
    """
    Fleet Manager coordinates transport orders to available AGVs
    Maintains order queue and tracks mission execution
    """
    
    def __init__(self):
        super().__init__('fleet_manager')
        
        # Order management
        self.order_queue = deque()
        self.active_orders = {}
        self.completed_orders = []
        self.failed_orders = []
        self.waypoints = {}  # Named locations from config
        
        # Robot availability service client
        self.availability_client = self.create_client(
            RobotAvailable,
            '/robot_available'
        )
        
        # Transport order action client
        self.transport_client = ActionClient(
            self,
            TransportOrder,
            '/transport_order'
        )
        self.interactive_order_srv = self.create_service(
            SubmitOrder,
            '/submit_interactive_order',
            self.handle_interactive_order
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_dispatch', True),
                ('queue_check_interval', 2.0),
                ('waypoints_file', ''),  # Path to waypoints YAML
                ('use_demo_orders', True),  # Load demo orders from YAML (False for interactive-only)
            ]
        )
        
        self.auto_dispatch = self.get_parameter('auto_dispatch').value
        self.use_demo_orders = self.get_parameter('use_demo_orders').value
        queue_interval = self.get_parameter('queue_check_interval').value
        
        # Queue processing timer
        if self.auto_dispatch:
            self.queue_timer = self.create_timer(queue_interval, self.process_queue)
        
        self.get_logger().info("Fleet Manager initialized")
        self.get_logger().info(f"Auto-dispatch: {self.auto_dispatch}")
        self.get_logger().info(f"Use demo orders: {self.use_demo_orders}")
        
        # Load waypoints and demo orders from config (if enabled)
        self.load_waypoints_config()
    
    def load_waypoints_config(self):
        """Load waypoints and demo orders from YAML configuration"""
        # Try parameter first, then default location
        waypoints_file = self.get_parameter('waypoints_file').value
        
        if not waypoints_file:
            try:
                pkg_share = get_package_share_directory('pbot_bringup')
                waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
            except Exception:
                self.get_logger().warn("Could not find pbot_bringup package")
                if self.use_demo_orders:
                    self.load_fallback_orders()
                else:
                    self.get_logger().info("Interactive mode - waiting for orders via /submit_interactive_order")
                return
        
        if not os.path.exists(waypoints_file):
            self.get_logger().warn(f"Waypoints file not found: {waypoints_file}")
            if self.use_demo_orders:
                self.load_fallback_orders()
            else:
                self.get_logger().info("Interactive mode - waiting for orders via /submit_interactive_order")
            return
        
        try:
            with open(waypoints_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Load named locations
            if 'locations' in config:
                self.waypoints = config['locations']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoint locations")
                for name, pose in self.waypoints.items():
                    self.get_logger().info(f"  {name}: ({pose['x']:.2f}, {pose['y']:.2f}, θ={pose['theta']:.2f})")
            
            # Load demo orders (only if use_demo_orders is True)
            if 'demo_orders' in config and self.use_demo_orders:
                for order_config in config['demo_orders']:
                    pickup_name = order_config['pickup']
                    dropoff_name = order_config['dropoff']
                    
                    if pickup_name not in self.waypoints:
                        self.get_logger().error(f"Unknown pickup location: {pickup_name}")
                        continue
                    if dropoff_name not in self.waypoints:
                        self.get_logger().error(f"Unknown dropoff location: {dropoff_name}")
                        continue
                    
                    pickup = self.waypoints[pickup_name]
                    dropoff = self.waypoints[dropoff_name]
                    
                    order = self.create_order(
                        pickup_x=pickup['x'], pickup_y=pickup['y'], pickup_theta=pickup['theta'],
                        dropoff_x=dropoff['x'], dropoff_y=dropoff['y'], dropoff_theta=dropoff['theta']
                    )
                    self.order_queue.append(order)
                    
                    self.get_logger().info(
                        f"Order: {order_config.get('name', order.order_id)} | "
                        f"{pickup_name} → {dropoff_name}"
                    )
                
                self.get_logger().info(f"Loaded {len(self.order_queue)} orders from config")
            elif not self.use_demo_orders:
                self.get_logger().info("Interactive mode - waiting for orders via /submit_interactive_order")
            else:
                self.get_logger().warn("No demo_orders in config, queue is empty")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints config: {e}")
            if self.use_demo_orders:
                self.load_fallback_orders()
            else:
                self.get_logger().info("Interactive mode - waiting for orders via /submit_interactive_order")
    
    def load_fallback_orders(self):
        """Load hardcoded fallback orders if config not available"""
        self.get_logger().info("Loading fallback demo orders...")
        
        # Verified warehouse poses from testing
        order1 = self.create_order(
            pickup_x=5.926, pickup_y=-0.173, pickup_theta=3.14,
            dropoff_x=3.255, dropoff_y=-2.773, dropoff_theta=1.4
        )
        self.order_queue.append(order1)
        
        order2 = self.create_order(
            pickup_x=4.0, pickup_y=-1.5, pickup_theta=0.0,
            dropoff_x=2.0, dropoff_y=-3.0, dropoff_theta=1.57
        )
        self.order_queue.append(order2)
        
        self.get_logger().info(f"Loaded {len(self.order_queue)} fallback orders")
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple:
        """
        Convert yaw angle (radians) to quaternion (x, y, z, w)
        Proper quaternion conversion for 2D rotation around Z-axis
        """
        half_yaw = yaw / 2.0
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    
    def create_order(
        self,
        pickup_x: float, pickup_y: float, pickup_theta: float,
        dropoff_x: float, dropoff_y: float, dropoff_theta: float
    ) -> TransportOrder.Goal:
        """
        Create a transport order with pickup and dropoff poses
        
        Args:
            pickup_x, pickup_y, pickup_theta: Pickup pose (theta in radians)
            dropoff_x, dropoff_y, dropoff_theta: Dropoff pose (theta in radians)
        
        Returns:
            TransportOrder.Goal message
        """
        order = TransportOrder.Goal()
        order.order_id = f"ORDER_{str(uuid.uuid4())[:8]}"
        
        # Pickup pose with proper quaternion
        order.pickup_pose = Pose()
        order.pickup_pose.position.x = pickup_x
        order.pickup_pose.position.y = pickup_y
        order.pickup_pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(pickup_theta)
        order.pickup_pose.orientation.x = qx
        order.pickup_pose.orientation.y = qy
        order.pickup_pose.orientation.z = qz
        order.pickup_pose.orientation.w = qw
        
        # Dropoff pose with proper quaternion
        order.dropoff_pose = Pose()
        order.dropoff_pose.position.x = dropoff_x
        order.dropoff_pose.position.y = dropoff_y
        order.dropoff_pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(dropoff_theta)
        order.dropoff_pose.orientation.x = qx
        order.dropoff_pose.orientation.y = qy
        order.dropoff_pose.orientation.z = qz
        order.dropoff_pose.orientation.w = qw
        
        return order
    
    def process_queue(self):
        """Periodically check for available robots and dispatch orders"""
        if not self.order_queue:
            return
        
        # Check robot availability
        if not self.availability_client.wait_for_service(timeout_sec=1.0):
            return
        
        future = self.availability_client.call_async(RobotAvailable.Request())
        future.add_done_callback(self.availability_callback)
    
    def availability_callback(self, future):
        """Handle robot availability response"""
        try:
            response = future.result()
            
            if response.available and self.order_queue:
                self.get_logger().info(f"Robot available: {response.reason}")
                self.dispatch_next_order()
            elif not response.available and self.order_queue:
                self.get_logger().debug(
                    f"Robot busy ({response.current_state}) - "
                    f"{len(self.order_queue)} orders waiting"
                )
        
        except Exception as e:
            self.get_logger().error(f"Availability check failed: {str(e)}")
    
    def dispatch_next_order(self):
        """Dispatch next order from queue to available robot"""
        if not self.order_queue:
            self.get_logger().info("Order queue empty - all orders dispatched")
            return
        
        order = self.order_queue.popleft()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Dispatching Order: {order.order_id}")
        self.get_logger().info(
            f"  Pickup:  ({order.pickup_pose.position.x:.2f}, {order.pickup_pose.position.y:.2f})"
        )
        self.get_logger().info(
            f"  Dropoff: ({order.dropoff_pose.position.x:.2f}, {order.dropoff_pose.position.y:.2f})"
        )
        self.get_logger().info(f"  Remaining in queue: {len(self.order_queue)}")
        self.get_logger().info("=" * 60)
        
        # Wait for action server
        if not self.transport_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Transport action server not available")
            self.order_queue.appendleft(order)  # Re-queue
            return
        
        # Send order
        send_goal_future = self.transport_client.send_goal_async(
            order,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, order.order_id)
        )
        
        self.active_orders[order.order_id] = order
    
    def goal_response_callback(self, future, order_id: str):
        """Handle goal acceptance/rejection"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Order {order_id} REJECTED by AGV")
                self.failed_orders.append(order_id)
                if order_id in self.active_orders:
                    del self.active_orders[order_id]
                return
            
            self.get_logger().info(f"Order {order_id} ACCEPTED - Mission executing")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self.result_callback(future, order_id)
            )
        
        except Exception as e:
            self.get_logger().error(f"Goal response error: {str(e)}")
    
    def feedback_callback(self, feedback_msg):
        """Handle mission feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Progress: {feedback.progress_percentage:.1f}% | "
            f"State: {feedback.current_state} | "
            f"Status: {feedback.status_detail}"
        )
    
    def result_callback(self, future, order_id: str):
        """Handle mission completion"""
        try:
            result = future.result().result
            
            if result.success:
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"✓ Order {order_id} COMPLETED")
                self.get_logger().info(f"  Execution Time: {result.execution_time:.2f}s")
                self.get_logger().info(f"  Message: {result.message}")
                self.get_logger().info("=" * 60)
                self.completed_orders.append(order_id)
            else:
                self.get_logger().error("=" * 60)
                self.get_logger().error(f"✗ Order {order_id} FAILED")
                self.get_logger().error(f"  Error: {result.message}")
                self.get_logger().error("=" * 60)
                self.failed_orders.append(order_id)
            
            # Remove from active orders
            if order_id in self.active_orders:
                del self.active_orders[order_id]
            
            # Print statistics
            total = len(self.completed_orders) + len(self.failed_orders)
            if total > 0:
                self.get_logger().info(
                    f"Statistics: {len(self.completed_orders)}/{total} orders completed successfully"
                )
        
        except Exception as e:
            self.get_logger().error(f"Result callback error: {str(e)}")

    def handle_interactive_order(self, request, response):
        """
        Handle interactive order submission from RViz marker selector
        
        Args:
            request: Contains order_id, pickup_pose, dropoff_pose
            response: Success status and message
        """
        try:
            # Create transport order from poses
            order = TransportOrder.Goal()
            order.order_id = request.order_id
            order.pickup_pose = request.pickup_pose
            order.dropoff_pose = request.dropoff_pose
            
            # Add to queue
            self.order_queue.append(order)
            
            response.success = True
            response.message = (
                f"Interactive order {request.order_id} queued | "
                f"Pickup: ({request.pickup_pose.position.x:.2f}, {request.pickup_pose.position.y:.2f}) | "
                f"Dropoff: ({request.dropoff_pose.position.x:.2f}, {request.dropoff_pose.position.y:.2f}) | "
                f"Queue position: {len(self.order_queue)}"
            )
            
            self.get_logger().info("=" * 70)
            self.get_logger().info("📥 INTERACTIVE ORDER RECEIVED")
            self.get_logger().info(f"  Order ID: {order.order_id}")
            self.get_logger().info(
                f"  Pickup:  ({request.pickup_pose.position.x:.2f}, "
                f"{request.pickup_pose.position.y:.2f})"
            )
            self.get_logger().info(
                f"  Dropoff: ({request.dropoff_pose.position.x:.2f}, "
                f"{request.dropoff_pose.position.y:.2f})"
            )
            self.get_logger().info(f"  Queue Position: {len(self.order_queue)}")
            self.get_logger().info("=" * 70)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to queue interactive order: {str(e)}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Fleet Manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
