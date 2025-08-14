#!/usr/bin/env python3

# Click in RViz (publish point) to send a NavigateToPose goal (in map frame)

# Usage:
#   rviz: add "Publish Point" tool, click on the map
#   this node will navigate to the goal point.
#   Clicking again cancels and re-goals

import math 
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw*0.5))

class WaypointClickNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_click_navigator')

        self.declare_parameter('goal_timeout_sec', 0.0)
        self.declare_parameter('face_goal', True)
        self.declare_parameter('default_yaw_deg', 0.0)

        self._click_lock = threading.Lock()
        self._latest_click: Optional[PointStamped] = None
        self._click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self._on_click, 10
        )

        # Initialize the Nav2 Commander
        self._nav = BasicNavigator()

        self.get_logger().info('Waiting for Nav2 to activate...')
        self._nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. Click on the /map in RViz to navigate.')

        # Main loop timer
        self._timer = self.create_timer(0.05, self._tick) # 20hz

        self._task_active = False

    def _on_click(self, msg: PointStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'/clicked_point frame_id is "{msg.header.frame_id}", expected "map". ' 
                'Continuing, but make sure TF tree is correct.'
            )
        with self._click_lock:
            self._latest_click = msg

    def _consume_click(self) -> Optional[PointStamped]:
        with self._click_lock:
            msg = self._latest_click
            self._latest_click = None
            return msg

    # The main loop... runs at 20hz
    def _tick(self):
        # If we get a new click while the task is running, cancel current task
        click = self._consume_click()
        if click is not None:
            if self._task_active:
                self.get_logger().info('New click detected. Canceling current trajectory...')
                self._nav.cancelTask()
                self._task_active = False

            goal = self._make_goal_from_click(click)
            self.get_logger().info(
                f'Navigating to x ={goal.pose.position.x:.3f}, '
                f'y={goal.pose.position.y:.3f}, '
                f'z={goal.pose.orientation.z:.3f}, '
                f'w={goal.pose.orientation.w:.3f})'
            )
            self._nav.goToPose(goal)
            self._task_active = True

        # If task is running, monitor
        if self._task_active:
            feedback = self._nav.getFeedback()
            if feedback:
                try:
                    dist = feedback.distance_remaining
                    self.get_logger().info(f'Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)
                except Exception:
                    pass # Distance remaining may not always be populated

            # Optional timeout if goal is not reached
            timeout_sec = self.get_parameter('goal_timeout_sec').get_parameter_value().double_value
            if timeout_sec > 0.0 and feedback and feedback.navigation_time > Duration(seconds=timeout_sec):
                self.get_logger().warn(f'Goal timeout ({timeout_sec:.1f}s). Cancelling.')
                self._nav.cancelTask()
                self._task_active = False
                return

            if self._nav.isTaskComplete():
                result = self._nav.getResult()
                txt = {
                    TaskResult.SUCCEEDED: 'SUCCEEDED',
                    TaskResult.CANCELED: 'CANCELED',
                    TaskResult.FAILED: 'FAILED',
                    None: 'UNKNOWN'
                }.get(result, str(result))
                self.get_logger().info(f'Navigation finished. Status: {txt}')
                self._task_active = False
                self.get_logger().info('Click another point to navigate again.')

    def _make_goal_from_click(self, click: PointStamped) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = click.point.x
        goal.pose.position.y = click.point.y
        goal.pose.position.z = 0.0

        # Set goal orientation
        face_goal = self.get_parameter('face_goal').get_parameter_value().bool_value
        if face_goal:
            current = self._nav.getCurrentPose()
            # If our current transform isn't relative to the map, then fallback to zero yaw
            if current is None or current.header.frame_id != 'map':
                yaw = 0.0
            else:
                dx = goal.pose.position.x - current.pose.position.x
                dy = goal.pose.position.y - current.pose.position.y
                yaw = math.atan2(dy, dx)
        else:
            yaw_deg = self.get_parameter('default_yaw_deg').get_parameter_value().double_value
            yaw = math.radians(yaw_deg)

        goal.pose.orientation = yaw_to_quaternion(yaw)
        return goal

def main():
    rclpy.init()
    node = WaypointClickNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
