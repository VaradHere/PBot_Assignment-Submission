#!/usr/bin/env python3
"""
AGV Supervisor Node - Core FSM and mission execution logic
Manages autonomous transport missions with state-based control
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from pbot_interfaces.msg import RobotStatus
from pbot_interfaces.srv import RobotAvailable
from pbot_interfaces.action import TransportOrder

from .fsm_states import FSMState, FSMManager
from .nav_client import NavigationClient


class AGVSupervisor(Node):
    """
    Main supervisor node implementing FSM-based autonomous control
    Coordinates navigation, manipulation, and mission execution
    """
    
    def __init__(self):
        super().__init__('agv_supervisor')
        
        # Callback groups for concurrent operations
        self.cb_group = ReentrantCallbackGroup()
        
        # Initialize FSM
        self.fsm = FSMManager(self.get_logger(), FSMState.IDLE)
        
        # Mission state
        self.current_order: Optional[TransportOrder.Goal] = None
        self.current_goal_handle = None
        self.mission_start_time = None
        self.current_pose = Pose()
        self.gripper_engaged = False
        
        # Failure handling counters
        self.pickup_retry_count = 0
        self.dropoff_retry_count = 0
        self.nav_retry_count = 0
        self.max_pickup_retries = 2
        self.max_nav_retries = 1
        
        # Parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize navigation client
        self.nav_client = NavigationClient(self, callback_group=self.cb_group)
        
        # TF for pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        
        # Subscribers - Use AMCL pose for map-frame position
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Services
        self.availability_srv = self.create_service(
            RobotAvailable,
            '/robot_available',
            self.handle_availability_check,
            callback_group=self.cb_group
        )
        
        # Service clients
        self.gripper_client = self.create_client(
            SetBool,
            '/gripper_control',
            callback_group=self.cb_group
        )
        
        # Action server
        self.transport_action_server = ActionServer(
            self,
            TransportOrder,
            '/transport_order',
            execute_callback=self.execute_transport_order,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            0.5,
            self.publish_status,
            callback_group=self.cb_group
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("AGV Supervisor Node Initialized")
        self.get_logger().info(f"Initial State: {self.fsm.get_state_name()}")
        self.get_logger().info("=" * 60)
    
    def _declare_parameters(self):
        """Declare node parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dock_pose.x', 0.0),
                ('dock_pose.y', 0.0),
                ('dock_pose.z', 0.0),
                ('dock_pose.theta', 0.0),
                ('position_tolerance', 0.3),
                ('navigation_timeout', 300.0),
                ('pickup_timeout', 10.0),
            ]
        )
    
    def _load_parameters(self):
        """Load parameters from parameter server"""
        self.dock_pose = Pose()
        self.dock_pose.position.x = self.get_parameter('dock_pose.x').value
        self.dock_pose.position.y = self.get_parameter('dock_pose.y').value
        self.dock_pose.position.z = self.get_parameter('dock_pose.z').value
        
        theta = self.get_parameter('dock_pose.theta').value
        self.dock_pose.orientation.z = math.sin(theta / 2.0)
        self.dock_pose.orientation.w = math.cos(theta / 2.0)
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.nav_timeout = self.get_parameter('navigation_timeout').value
        self.pickup_timeout = self.get_parameter('pickup_timeout').value
        
        self.get_logger().info(f"Dock position: ({self.dock_pose.position.x}, {self.dock_pose.position.y})")
        self.get_logger().info(f"Position tolerance: {self.position_tolerance}m")
    
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current robot pose from AMCL (map frame)"""
        self.current_pose = msg.pose.pose
    
    def handle_availability_check(self, request, response):
        """Service callback: Check if robot is available for new orders"""
        response.available = self.fsm.can_accept_order()
        response.current_state = self.fsm.get_state_name()
        
        if response.available:
            response.reason = "Robot is idle and ready for orders"
        else:
            response.reason = f"Robot busy in state: {response.current_state}"
        
        return response
    
    def goal_callback(self, goal_request: TransportOrder.Goal):
        """Action server: Accept or reject incoming transport orders"""
        if not self.fsm.can_accept_order():
            self.get_logger().warn(
                f"Rejecting order {goal_request.order_id} - "
                f"Robot in state: {self.fsm.get_state_name()}"
            )
            return GoalResponse.REJECT
        
        self.get_logger().info(f"Accepting transport order: {goal_request.order_id}")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Action server: Handle order cancellation"""
        self.get_logger().warn(f"Cancel request received for order {self.current_order.order_id if self.current_order else 'Unknown'}")
        return CancelResponse.ACCEPT
    
    def execute_transport_order(self, goal_handle):
        """
        Main FSM execution loop for transport order
        Implements complete state machine with error handling
        """
        self.current_goal_handle = goal_handle
        self.current_order = goal_handle.request
        self.mission_start_time = time.time()
        
        # Reset retry counters
        self.pickup_retry_count = 0
        self.dropoff_retry_count = 0
        self.nav_retry_count = 0
        
        result = TransportOrder.Result()
        
        try:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"Starting Transport Mission: {self.current_order.order_id}")
            self.get_logger().info("=" * 60)
            
            # ============ STATE: AWAITING_ORDER ============
            self.fsm.transition_to(FSMState.AWAITING_ORDER, "Order received, preparing mission")
            self._send_feedback("Mission accepted", 5.0)
            time.sleep(1.0)
            
            # ============ STATE: NAVIGATING_TO_PICKUP ============
            self.fsm.transition_to(FSMState.NAVIGATING_TO_PICKUP, "Starting navigation to pickup zone")
            self._send_feedback("Navigating to pickup", 10.0)
            
            nav_success = self.nav_client.navigate_to_pose(
                self.current_order.pickup_pose,
                timeout=self.nav_timeout
            )
            
            if not nav_success:
                nav_success = self._retry_navigation(
                    self.current_order.pickup_pose,
                    "pickup"
                )
                if not nav_success:
                    raise Exception("Failed to navigate to pickup after retries")
            
            # ============ STATE: PICKUP_ACTION ============
            self.fsm.transition_to(FSMState.PICKUP_ACTION, "Arrived at pickup, engaging gripper")
            self._send_feedback("Executing pickup", 35.0)
            
            pickup_success = self._execute_pickup()
            
            if not pickup_success:
                pickup_success = self._retry_pickup()
                if not pickup_success:
                    raise Exception(f"Pickup failed after {self.max_pickup_retries} attempts")
            
            # ============ STATE: NAVIGATING_TO_DELIVERY ============
            self.fsm.transition_to(FSMState.NAVIGATING_TO_DELIVERY, "Pickup complete, navigating to delivery")
            self._send_feedback("Transporting to delivery zone", 50.0)
            
            self.nav_retry_count = 0  # Reset for delivery navigation
            nav_success = self.nav_client.navigate_to_pose(
                self.current_order.dropoff_pose,
                timeout=self.nav_timeout
            )
            
            if not nav_success:
                nav_success = self._retry_navigation(
                    self.current_order.dropoff_pose,
                    "delivery"
                )
                if not nav_success:
                    raise Exception("Failed to navigate to delivery after retries")
            
            # ============ STATE: DROPOFF_ACTION ============
            self.fsm.transition_to(FSMState.DROPOFF_ACTION, "Arrived at delivery, releasing gripper")
            self._send_feedback("Executing dropoff", 75.0)
            
            dropoff_success = self._execute_dropoff()
            if not dropoff_success:
                raise Exception("Dropoff action failed")
            
            # ============ STATE: RETURNING_TO_DOCK ============
            self.fsm.transition_to(FSMState.RETURNING_TO_DOCK, "Delivery complete, returning to dock")
            self._send_feedback("Returning to docking station", 90.0)
            
            dock_success = self.nav_client.navigate_to_pose(
                self.dock_pose,
                timeout=self.nav_timeout
            )
            
            if not dock_success:
                self.get_logger().warn("Failed to dock, but mission considered successful")
            
            # ============ MISSION SUCCESS ============
            execution_time = time.time() - self.mission_start_time
            result.success = True
            result.message = f"Transport order {self.current_order.order_id} completed successfully"
            result.execution_time = execution_time
            
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✓ Mission Completed Successfully")
            self.get_logger().info(f"  Order ID: {self.current_order.order_id}")
            self.get_logger().info(f"  Execution Time: {execution_time:.2f}s")
            self.get_logger().info("=" * 60)
            
            self._send_feedback("Mission complete", 100.0)
            goal_handle.succeed()
        
        except Exception as e:
            # ============ ERROR RECOVERY ============
            self.get_logger().error(f"Mission failed: {str(e)}")
            self.fsm.transition_to(FSMState.ERROR_RECOVERY, str(e))
            
            # Emergency docking
            self.get_logger().warn("Initiating emergency return to dock")
            self.nav_client.navigate_to_pose(self.dock_pose, timeout=self.nav_timeout)
            
            execution_time = time.time() - self.mission_start_time
            result.success = False
            result.message = f"Mission failed: {str(e)}"
            result.execution_time = execution_time
            
            goal_handle.abort()
        
        finally:
            # ============ CLEANUP AND RETURN TO IDLE ============
            self.fsm.transition_to(FSMState.IDLE, "Mission complete, ready for next order")
            self.current_order = None
            self.current_goal_handle = None
            self.gripper_engaged = False
        
        return result
    
    def _retry_navigation(self, target_pose: Pose, location: str) -> bool:
        """Retry navigation with backoff"""
        if self.nav_retry_count >= self.max_nav_retries:
            return False
        
        self.nav_retry_count += 1
        self.get_logger().warn(
            f"Navigation to {location} failed, "
            f"retry {self.nav_retry_count}/{self.max_nav_retries}"
        )
        
        time.sleep(2.0)  # Brief pause before retry
        return self.nav_client.navigate_to_pose(target_pose, timeout=self.nav_timeout)
    
    def _retry_pickup(self) -> bool:
        """Retry pickup operation"""
        if self.pickup_retry_count >= self.max_pickup_retries:
            return False
        
        self.pickup_retry_count += 1
        self.get_logger().warn(
            f"Pickup failed, retry {self.pickup_retry_count}/{self.max_pickup_retries}"
        )
        
        time.sleep(2.0)
        return self._execute_pickup()
    
    def _execute_pickup(self) -> bool:
        """Execute pickup with simulated gripper"""
        # Verify position
        if not self._is_at_target(self.current_order.pickup_pose):
            self.get_logger().error(
                f"Position check failed - not within {self.position_tolerance}m of pickup"
            )
            return False
        
        # Positioning delay
        time.sleep(1.5)
        
        # Engage gripper
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gripper service unavailable")
            return False
        
        request = SetBool.Request()
        request.data = True  # Engage
        
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.gripper_engaged = True
            self.get_logger().info("✓ Pickup successful - bin attached")
            return True
        else:
            msg = future.result().message if future.result() else "Service call failed"
            self.get_logger().error(f"Gripper engagement failed: {msg}")
            return False
    
    def _execute_dropoff(self) -> bool:
        """Execute dropoff with simulated gripper"""
        # Verify position
        if not self._is_at_target(self.current_order.dropoff_pose):
            self.get_logger().error(
                f"Position check failed - not within {self.position_tolerance}m of dropoff"
            )
            return False
        
        # Positioning delay
        time.sleep(1.5)
        
        # Release gripper
        request = SetBool.Request()
        request.data = False  # Release
        
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.gripper_engaged = False
            self.get_logger().info("✓ Dropoff successful - bin released")
            return True
        else:
            msg = future.result().message if future.result() else "Service call failed"
            self.get_logger().error(f"Gripper release failed: {msg}")
            return False
    
    def _is_at_target(self, target_pose: Pose) -> bool:
        """Check if robot is within tolerance of target"""
        dx = self.current_pose.position.x - target_pose.position.x
        dy = self.current_pose.position.y - target_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        at_target = distance < self.position_tolerance
        
        if not at_target:
            self.get_logger().debug(f"Distance to target: {distance:.3f}m (tolerance: {self.position_tolerance}m)")
        
        return at_target
    
    def _send_feedback(self, status: str, progress: float):
        """Send action feedback"""
        if self.current_goal_handle:
            feedback = TransportOrder.Feedback()
            feedback.current_state = self.fsm.get_state_name()
            feedback.progress_percentage = progress
            feedback.current_position = self.current_pose
            feedback.status_detail = status
            
            self.current_goal_handle.publish_feedback(feedback)
    
    def publish_status(self):
        """Publish robot status for system observability"""
        msg = RobotStatus()
        msg.fsm_state = self.fsm.get_state_name()
        msg.active_goal = self.current_order.order_id if self.current_order else "None"
        msg.current_pose = self.current_pose
        msg.status_message = f"State: {self.fsm.get_state_name()}"
        msg.battery_level = 100.0  # Simulated
        msg.gripper_state = self.gripper_engaged
        msg.retry_count = self.pickup_retry_count + self.nav_retry_count
        
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = AGVSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AGV Supervisor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
