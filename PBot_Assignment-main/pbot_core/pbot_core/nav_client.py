#!/usr/bin/env python3
"""
Navigation client wrapper for Nav2 integration
Handles navigation goal sending, monitoring, and error handling
"""

import time
import threading
import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus


class NavigationClient:
    """Wrapper for Nav2 navigation action client"""
    
    def __init__(self, node: Node, callback_group=None):
        self.node = node
        self.logger = node.get_logger()
        
        self.nav_action_client = ActionClient(
            node,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=callback_group
        )
        
        self.current_goal_handle = None
        self.navigation_active = False
        
        # Thread-safe result handling
        self._result_event = threading.Event()
        self._navigation_status = None
    
    def navigate_to_pose(self, target_pose: Pose, timeout: float = 300.0) -> bool:
        """
        Navigate to target pose using Nav2
        
        Args:
            target_pose: Target pose in map frame
            timeout: Navigation timeout in seconds
        
        Returns:
            True if navigation succeeded, False otherwise
        """
        # Wait for action server
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.logger.error("Nav2 action server not available")
            return False
        
        # Construct goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose = target_pose
        
        self.logger.info(
            f"Sending navigation goal: "
            f"x={target_pose.position.x:.2f}, "
            f"y={target_pose.position.y:.2f}"
        )
        
        # Reset state
        self._result_event.clear()
        self._navigation_status = None
        self.navigation_active = True
        
        try:
            # Send goal asynchronously with callbacks
            send_goal_future = self.nav_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )
            send_goal_future.add_done_callback(self._goal_response_callback)
            
            # Wait for result using threading event (executor handles callbacks)
            success = self._result_event.wait(timeout=timeout)
            
            self.navigation_active = False
            
            if not success:
                self.logger.error(f"Navigation timeout after {timeout}s")
                self.cancel_navigation()
                return False
            
            # Check final status
            if self._navigation_status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info("✓ Navigation completed successfully")
                return True
            elif self._navigation_status == GoalStatus.STATUS_CANCELED:
                self.logger.warn("Navigation was canceled")
                return False
            elif self._navigation_status == GoalStatus.STATUS_ABORTED:
                self.logger.error("Navigation was aborted by Nav2")
                return False
            elif self._navigation_status == -1:  # Our custom code for rejected
                self.logger.error("Navigation goal was rejected")
                return False
            else:
                self.logger.error(f"Navigation ended with status: {self._navigation_status}")
                return False
        
        except Exception as e:
            self.logger.error(f"Navigation exception: {str(e)}")
            self.navigation_active = False
            return False
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.error("Navigation goal rejected by Nav2")
            self._navigation_status = -1  # Custom code for rejected
            self._result_event.set()
            return
        
        self.current_goal_handle = goal_handle
        self.logger.info("Navigation goal accepted, executing...")
        
        # Request result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        """Handle navigation result"""
        try:
            result = future.result()
            self._navigation_status = result.status
            self.logger.info(f"Navigation result received, status: {result.status}")
        except Exception as e:
            self.logger.error(f"Error getting navigation result: {e}")
            self._navigation_status = GoalStatus.STATUS_ABORTED
        finally:
            self._result_event.set()
    
    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional logging)"""
        pass  # Can add progress logging here if needed
    
    def cancel_navigation(self):
        """Cancel active navigation goal"""
        if self.current_goal_handle and self.navigation_active:
            self.logger.warn("Cancelling active navigation goal")
            self.current_goal_handle.cancel_goal_async()
            self.navigation_active = False
    
    def is_navigating(self) -> bool:
        """Check if navigation is currently active"""
        return self.navigation_active
