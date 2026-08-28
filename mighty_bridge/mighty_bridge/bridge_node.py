"""Bridge between the MIGHTY planner and AirStack's local-planner seam.

Responsibilities (one node so the module adds a single process):

- odometry (nav_msgs/Odometry) -> dynus_interfaces/State on ``state``,
  rotating the body-frame twist into the world frame (nav_msgs convention
  puts twist in the child frame; MIGHTY expects world-frame velocity).
- MIGHTY's committed ``dynus_interfaces/Trajectory`` -> decimated
  airstack_msgs/TrajectoryXYZVYaw on ``trajectory_segment_to_add``. The
  trajectory controller's ADD_SEGMENT merge splices each new committed
  trajectory at the closest future point, matching MIGHTY's
  replan-from-committed-point behavior.
- NavigateTask action server (``~/navigate_task``): walks the goal path's
  poses as successive ``term_goal`` checkpoints for MIGHTY, mirroring the
  droan_gl task contract (ADD_SEGMENT while navigating, TRACK on exit).
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from airstack_msgs.srv import TrajectoryMode
from dynus_interfaces.msg import State, Trajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from task_msgs.action import NavigateTask


def quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by quaternion q (Hamilton, w last in args)."""
    # t = 2 * cross(q_vec, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w * t + cross(q_vec, t)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class MightyBridge(Node):
    def __init__(self):
        super().__init__('mighty_bridge')

        self.declare_parameter('waypoint_tolerance_m', 2.0)
        self.declare_parameter('term_goal_republish_s', 5.0)
        self.declare_parameter('segment_stride', 10)
        self.declare_parameter('twist_in_body_frame', True)
        self.declare_parameter('world_frame', 'map')

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance_m').value)
        self.republish_s = float(self.get_parameter('term_goal_republish_s').value)
        self.segment_stride = max(1, int(self.get_parameter('segment_stride').value))
        self.twist_in_body_frame = bool(self.get_parameter('twist_in_body_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)

        self._lock = threading.Lock()
        self._odom = None            # latest nav_msgs/Odometry
        self._last_traj_time = 0.0   # wall time of last MIGHTY trajectory
        self._task_active = False
        self._cancel_requested = False

        cb = ReentrantCallbackGroup()

        latest_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)

        # odometry in -> state out
        self.state_pub = self.create_publisher(State, 'state', state_qos)
        self.create_subscription(Odometry, 'odometry', self._odom_cb, latest_qos,
                                 callback_group=cb)

        # MIGHTY committed trajectory in -> controller segment out
        self.segment_pub = self.create_publisher(
            TrajectoryXYZVYaw, 'trajectory_segment_to_add', 1)
        self.create_subscription(Trajectory, 'mighty_trajectory', self._traj_cb,
                                 reliable_qos, callback_group=cb)

        # term_goal out to MIGHTY
        self.term_goal_pub = self.create_publisher(PoseStamped, 'term_goal', reliable_qos)

        # trajectory controller mode client
        self.mode_client = self.create_client(TrajectoryMode, 'set_trajectory_mode',
                                              callback_group=cb)

        self._action_server = ActionServer(
            self, NavigateTask, '~/navigate_task',
            execute_callback=self._execute_navigate,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=cb)

        self.get_logger().info(
            f'mighty_bridge up (waypoint_tolerance={self.waypoint_tolerance} m, '
            f'stride={self.segment_stride}, world_frame={self.world_frame})')

    # ------------------------------------------------------------------
    # conversions
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

        s = State()
        s.header = msg.header
        s.pos.x = msg.pose.pose.position.x
        s.pos.y = msg.pose.pose.position.y
        s.pos.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        s.quat = q
        v = msg.twist.twist.linear
        if self.twist_in_body_frame:
            wx, wy, wz = quat_rotate(q.x, q.y, q.z, q.w, v.x, v.y, v.z)
        else:
            wx, wy, wz = v.x, v.y, v.z
        s.vel.x, s.vel.y, s.vel.z = wx, wy, wz
        self.state_pub.publish(s)

    def _traj_cb(self, msg: Trajectory):
        n = len(msg.goals)
        if n == 0:
            return
        with self._lock:
            self._last_traj_time = time.monotonic()

        out = TrajectoryXYZVYaw()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or self.world_frame

        idxs = list(range(0, n, self.segment_stride))
        if idxs[-1] != n - 1:
            idxs.append(n - 1)
        for i in idxs:
            g = msg.goals[i]
            wp = WaypointXYZVYaw()
            wp.position.x = g.p.x
            wp.position.y = g.p.y
            wp.position.z = g.p.z
            wp.velocity = math.sqrt(g.v.x ** 2 + g.v.y ** 2 + g.v.z ** 2)
            wp.yaw = g.yaw
            wp.acceleration.x = g.a.x
            wp.acceleration.y = g.a.y
            wp.acceleration.z = g.a.z
            wp.jerk.x = g.j.x
            wp.jerk.y = g.j.y
            wp.jerk.z = g.j.z
            out.waypoints.append(wp)
        self.segment_pub.publish(out)

    # ------------------------------------------------------------------
    # NavigateTask
    # ------------------------------------------------------------------

    def _handle_goal(self, goal_request):
        if self._task_active:
            self.get_logger().warn('Rejecting NavigateTask goal: task already active')
            return GoalResponse.REJECT
        if not goal_request.global_plan.poses:
            self.get_logger().warn('Rejecting NavigateTask goal: empty global_plan')
            return GoalResponse.REJECT
        self._task_active = True
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def _set_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('set_trajectory_mode service not available')
            return
        req = TrajectoryMode.Request()
        req.mode = mode
        self.mode_client.call_async(req)

    def _distance_to(self, pos):
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        dx = odom.pose.pose.position.x - pos.x
        dy = odom.pose.pose.position.y - pos.y
        dz = odom.pose.pose.position.z - pos.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _publish_term_goal(self, pose_stamped):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_stamped.header.frame_id or self.world_frame
        msg.pose = pose_stamped.pose
        self.term_goal_pub.publish(msg)

    def _finish(self, goal_handle, success, message):
        self._set_mode(TrajectoryMode.Request.TRACK)
        result = NavigateTask.Result()
        result.success = success
        result.message = message
        self._task_active = False
        if success:
            goal_handle.succeed()
        elif self._cancel_requested and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_navigate(self, goal_handle):
        goal = goal_handle.request
        self._cancel_requested = False
        poses = list(goal.global_plan.poses)
        final_pos = poses[-1].pose.position
        goal_tol = max(0.1, float(goal.goal_tolerance_m))

        self.get_logger().info(
            f'NavigateTask: {len(poses)} waypoints, goal tolerance {goal_tol:.2f} m')

        self._set_mode(TrajectoryMode.Request.ADD_SEGMENT)

        idx = 0
        last_pub = 0.0
        rate_s = 0.2

        while rclpy.ok():
            if self._cancel_requested and goal_handle.is_cancel_requested:
                return self._finish(goal_handle, False, 'Canceled')

            now = time.monotonic()
            # (Re-)issue the current checkpoint: on advance, and periodically in
            # case MIGHTY missed it or has stalled (re-pinning the same goal
            # restarts its replanning timer, which is a safe unstick).
            if now - last_pub > self.republish_s:
                self._publish_term_goal(poses[idx])
                last_pub = now

            dist_final = self._distance_to(final_pos)
            if dist_final is not None:
                fb = NavigateTask.Feedback()
                fb.status = f'navigating wp {idx + 1}/{len(poses)}'
                fb.distance_to_goal = float(dist_final)
                with self._lock:
                    odom = self._odom
                fb.current_position.x = odom.pose.pose.position.x
                fb.current_position.y = odom.pose.pose.position.y
                fb.current_position.z = odom.pose.pose.position.z
                goal_handle.publish_feedback(fb)

                if dist_final < goal_tol and idx == len(poses) - 1:
                    return self._finish(goal_handle, True, 'Goal reached')

                if idx < len(poses) - 1:
                    d = self._distance_to(poses[idx].pose.position)
                    if d is not None and d < self.waypoint_tolerance:
                        idx += 1
                        self._publish_term_goal(poses[idx])
                        last_pub = now
                        self.get_logger().info(
                            f'NavigateTask: advancing to waypoint {idx + 1}/{len(poses)}')

            time.sleep(rate_s)

        return self._finish(goal_handle, False, 'Node shutting down')


def main(args=None):
    rclpy.init(args=args)
    node = MightyBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
