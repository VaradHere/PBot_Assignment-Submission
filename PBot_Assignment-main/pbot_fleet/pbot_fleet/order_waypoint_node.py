#!/usr/bin/env python3
"""
Interactive Waypoint Selector Node
Allows evaluators to click pickup/dropoff points in RViz using Interactive Markers
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import math
import uuid

from pbot_interfaces.srv import SubmitOrder


class InteractiveWaypointSelector(Node):
    """
    Provides interactive markers in RViz for evaluators to:
    1. Click to set pickup location (green marker)
    2. Click to set dropoff location (red marker)
    3. Automatically submit order when both are set
    """
    
    def __init__(self):
        super().__init__('interactive_waypoint_selector')
        
        # Interactive marker server
        self.marker_server = InteractiveMarkerServer(self, 'waypoint_markers')
        
        # Marker publisher for visualization
        self.marker_pub = self.create_publisher(Marker, '/waypoint_visualization', 10)
        
        # Service client to submit orders
        self.submit_order_client = self.create_client(
            SubmitOrder,
            '/submit_interactive_order'
        )
        
        # State management
        self.pickup_pose = None
        self.dropoff_pose = None
        self.pickup_marker_set = False
        self.dropoff_marker_set = False
        
        self.marker_scale = 0.5
        self.marker_height = 0.5
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_submit', True),  # Auto-submit when both markers are set
                ('marker_scale', 0.5),
            ]
        )
        
        self.auto_submit = self.get_parameter('auto_submit').value
        self.marker_scale = self.get_parameter('marker_scale').value
        
        # Create interactive markers for pickup and dropoff
        self._create_pickup_marker()
        self._create_dropoff_marker()
        self._create_instruction_marker()
        
        # Timer to publish visualization markers
        self.viz_timer = self.create_timer(0.5, self.publish_visualization_markers)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Interactive Waypoint Selector - READY")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Instructions for evaluators:")
        self.get_logger().info("  1. In RViz, drag the GREEN marker to PICKUP location")
        self.get_logger().info("  2. Drag the RED marker to DROPOFF location")
        self.get_logger().info("  3. Order will auto-submit when both are positioned")
        self.get_logger().info("  4. Click 'Reset' button to clear and create new order")
        self.get_logger().info("=" * 70)
    
    def _create_pickup_marker(self):
        """Create interactive marker for pickup location"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "pickup_marker"
        int_marker.description = "PICKUP Location\n(Drag to set)"
        int_marker.scale = self.marker_scale
        
        # Initial position (near shelf_a in warehouse)
        int_marker.pose.position.x = 5.0
        int_marker.pose.position.y = -0.5
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0
        
        # Create arrow marker (green)
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 0.6
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        # Marker control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        
        int_marker.controls.append(marker_control)
        
        # Movement controls (XY plane)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)
        
        # Rotation control (around Z axis)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        # Add marker to server
        self.marker_server.insert(int_marker, feedback_callback=self.pickup_marker_feedback)
        self.marker_server.applyChanges()
        
        self.get_logger().info("✓ Pickup marker created (GREEN)")
    
    def _create_dropoff_marker(self):
        """Create interactive marker for dropoff location"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "dropoff_marker"
        int_marker.description = "DROPOFF Location\n(Drag to set)"
        int_marker.scale = self.marker_scale
        
        # Initial position (near assembly line in warehouse)
        int_marker.pose.position.x = 3.0
        int_marker.pose.position.y = -2.5
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0
        
        # Create arrow marker (red)
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 0.6
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        # Marker control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        
        int_marker.controls.append(marker_control)
        
        # Movement controls (XY plane)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control)
        
        # Rotation control
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        # Add marker to server
        self.marker_server.insert(int_marker, feedback_callback=self.dropoff_marker_feedback)
        self.marker_server.applyChanges()
        
        self.get_logger().info("✓ Dropoff marker created (RED)")
    
    def _create_instruction_marker(self):
        """Create a text marker with instructions"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "instruction_text"
        int_marker.description = ""
        int_marker.scale = 1.0
        
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 2.0  # High up
        int_marker.pose.orientation.w = 1.0
        
        # Text marker
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = "Interactive Order Mode\nDrag GREEN (pickup) and RED (dropoff)"
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        
        int_marker.controls.append(marker_control)
        
        self.marker_server.insert(int_marker)
        self.marker_server.applyChanges()
    
    def pickup_marker_feedback(self, feedback: InteractiveMarkerFeedback):
        """Handle pickup marker interaction"""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.pickup_pose = feedback.pose
            self.pickup_marker_set = True
            
            self.get_logger().info(
                f"✓ Pickup location set: "
                f"({feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f}, "
                f"θ={self._get_yaw_from_quaternion(feedback.pose.orientation):.2f} rad)"
            )
            
            # Check if ready to submit
            self._check_and_submit_order()
    
    def dropoff_marker_feedback(self, feedback: InteractiveMarkerFeedback):
        """Handle dropoff marker interaction"""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.dropoff_pose = feedback.pose
            self.dropoff_marker_set = True
            
            self.get_logger().info(
                f"✓ Dropoff location set: "
                f"({feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f}, "
                f"θ={self._get_yaw_from_quaternion(feedback.pose.orientation):.2f} rad)"
            )
            
            # Check if ready to submit
            self._check_and_submit_order()
    
    def _check_and_submit_order(self):
        """Check if both markers are set and submit order"""
        if not self.auto_submit:
            return
        
        if self.pickup_marker_set and self.dropoff_marker_set:
            self.get_logger().info("=" * 70)
            self.get_logger().info("Both waypoints set - Submitting interactive order...")
            self.get_logger().info("=" * 70)
            
            self._submit_order()
    
    def _submit_order(self):
        """Submit order to fleet manager"""
        if not self.submit_order_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Submit order service not available!")
            return
        
        # Create order request
        request = SubmitOrder.Request()
        request.order_id = f"INTERACTIVE_{str(uuid.uuid4())[:8]}"
        request.pickup_pose = self.pickup_pose
        request.dropoff_pose = self.dropoff_pose
        
        # Call service
        future = self.submit_order_client.call_async(request)
        future.add_done_callback(self._order_response_callback)
    
    def _order_response_callback(self, future):
        """Handle order submission response"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info("=" * 70)
                self.get_logger().info(f"✓ Order submitted successfully!")
                self.get_logger().info(f"  {response.message}")
                self.get_logger().info("=" * 70)
                self.get_logger().info("Ready for next order - reposition markers when robot is idle")
                
                # Reset for next order
                self.pickup_marker_set = False
                self.dropoff_marker_set = False
            else:
                self.get_logger().error(f"✗ Order submission failed: {response.message}")
        
        except Exception as e:
            self.get_logger().error(f"Order submission error: {str(e)}")
    
    def publish_visualization_markers(self):
        """Publish visualization markers showing current state"""
        # Pickup zone marker
        if self.pickup_pose and self.pickup_marker_set:
            marker = self._create_zone_marker(
                marker_id=1,
                pose=self.pickup_pose,
                color=(0.0, 1.0, 0.0, 0.3),
                text="PICKUP ZONE"
            )
            self.marker_pub.publish(marker)
        
        # Dropoff zone marker
        if self.dropoff_pose and self.dropoff_marker_set:
            marker = self._create_zone_marker(
                marker_id=2,
                pose=self.dropoff_pose,
                color=(1.0, 0.0, 0.0, 0.3),
                text="DROPOFF ZONE"
            )
            self.marker_pub.publish(marker)
        
        # Connection line
        if self.pickup_marker_set and self.dropoff_marker_set:
            line_marker = self._create_connection_line()
            self.marker_pub.publish(line_marker)
    
    def _create_zone_marker(self, marker_id: int, pose: Pose, color: tuple, text: str) -> Marker:
        """Create cylinder marker for pickup/dropoff zone"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "zones"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose = pose
        marker.pose.position.z = 0.01  # Slightly above ground
        
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.01
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.lifetime.sec = 1
        
        return marker
    
    def _create_connection_line(self) -> Marker:
        """Create line connecting pickup to dropoff"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "connection"
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Start point (pickup)
        p1 = Point()
        p1.x = self.pickup_pose.position.x
        p1.y = self.pickup_pose.position.y
        p1.z = 0.1
        
        # End point (dropoff)
        p2 = Point()
        p2.x = self.dropoff_pose.position.x
        p2.y = self.dropoff_pose.position.y
        p2.z = 0.1
        
        marker.points = [p1, p2]
        
        marker.scale.x = 0.05  # Line width
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        
        marker.lifetime.sec = 1
        
        return marker
    
    def _get_yaw_from_quaternion(self, quat: Quaternion) -> float:
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveWaypointSelector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Interactive Waypoint Selector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
