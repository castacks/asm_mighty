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
        self.declare_parameter('segment_stride', 5)  # 0.05 s spacing: denser segments track corners tighter
        self.declare_parameter('twist_in_body_frame', True)
        self.declare_parameter('world_frame', 'map')
        # global_plan follower (study R5-R7 contract): the route planner
        # publishes a nav_msgs/Path on global_plan; once the vehicle has
        # climbed follow_min_climb_m and settled, the bridge walks that
        # path's poses as term_goal checkpoints exactly like a NavigateTask.
        self.declare_parameter('follow_global_plan', True)
        self.declare_parameter('follow_min_climb_m', 8.0)
        self.declare_parameter('follow_settle_s', 3.0)
        # > mighty's goal_seen_radius (5.0) so the moving carrot never puts
        # the planner into GOAL_SEEN/GOAL_REACHED before the true route end
        self.declare_parameter('follow_lookahead_m', 8.0)

        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance_m').value)
        self.republish_s = float(self.get_parameter('term_goal_republish_s').value)
        self.segment_stride = max(1, int(self.get_parameter('segment_stride').value))
        self.twist_in_body_frame = bool(self.get_parameter('twist_in_body_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.follow_enabled = bool(self.get_parameter('follow_global_plan').value)
        self.follow_min_climb = float(self.get_parameter('follow_min_climb_m').value)
        self.follow_settle_s = float(self.get_parameter('follow_settle_s').value)
        self.follow_lookahead = float(self.get_parameter('follow_lookahead_m').value)

        self._lock = threading.Lock()
        self._odom = None            # latest nav_msgs/Odometry
        self._last_traj_time = 0.0   # wall time of last MIGHTY trajectory
        self._task_active = False
        self._cancel_requested = False
        self._route_active = False   # a NavigateTask or follower route is executing
        # follower state
        self._z0 = None
        self._settled_since = None
        self._airborne = False
        self._follow_plan = None     # list of PoseStamped adopted from global_plan
        self._follow_thread = None
        self._follow_done_plan = None
        self._traj_end = None        # last committed MIGHTY trajectory endpoint

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

        # global_plan follower (see class docstring)
        if self.follow_enabled:
            from nav_msgs.msg import Path
            self.create_subscription(Path, 'global_plan', self._global_plan_cb,
                                     reliable_qos, callback_group=cb)
            self.create_timer(1.0, self._follow_tick, callback_group=cb)

        self.get_logger().info(
            f'mighty_bridge up (waypoint_tolerance={self.waypoint_tolerance} m, '
            f'stride={self.segment_stride}, world_frame={self.world_frame}, '
            f'follow_global_plan={self.follow_enabled})')

    # ------------------------------------------------------------------
    # conversions
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

        # takeoff-settle detection for the follower (mirrors the mission-glue
        # trigger: climbed follow_min_climb_m and vertical speed ~0 for
        # follow_settle_s)
        if self.follow_enabled and not self._airborne:
            z = msg.pose.pose.position.z
            vz = msg.twist.twist.linear.z
            if self._z0 is None:
                self._z0 = z
            elif z - self._z0 > self.follow_min_climb and abs(vz) < 0.2:
                if self._settled_since is None:
                    self._settled_since = time.monotonic()
                elif time.monotonic() - self._settled_since > self.follow_settle_s:
                    self._airborne = True
                    self.get_logger().info('follower: takeoff settled')
            else:
                self._settled_since = None

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
        now = time.monotonic()
        g_end = msg.goals[-1]
        with self._lock:
            gap = now - self._last_traj_time if self._last_traj_time else 0.0
            self._last_traj_time = now
            route_active = self._route_active
            # committed-trajectory end: where MIGHTY believes the vehicle ends
            # up (the follower's catch-up gate compares the vehicle to this)
            self._traj_end = (g_end.p.x, g_end.p.y, g_end.p.z)
        # Never forward a segment whose START is far from the vehicle: after a
        # timeline reset the controller's tracking point jumps to the segment
        # start and the PID flies an uncommanded straight line through
        # unswept space (observed: pillar penetration). The follower's
        # catch-up gate keeps MIGHTY idle until the vehicle is close, so a
        # far-start segment here is always transient — drop it.
        s0 = msg.goals[0]
        d_start = self._distance_to_xyz(s0.p.x, s0.p.y, s0.p.z)
        if route_active and d_start is not None and d_start > 2.5:
            self.get_logger().warn(
                f'dropping segment starting {d_start:.1f} m from the vehicle '
                f'(catch-up in progress)')
            return
        # MIGHTY resuming after an idle (GOAL_REACHED at an intermediate
        # goal): the controller has meanwhile idled at its trajectory end
        # with virtual_time still advancing, so the resumed trajectory would
        # splice at a past time and merge() would silently reject it (the
        # hover-deadlock failure mode). Reset the timeline before forwarding.
        if route_active and gap > 2.0:
            self.get_logger().info(
                f'planner resumed after {gap:.1f}s idle — resetting controller timeline')
            self._set_mode(TrajectoryMode.Request.TRACK)
            time.sleep(0.25)
            self._set_mode(TrajectoryMode.Request.ADD_SEGMENT)
            time.sleep(0.1)

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
    # global_plan follower (study route contract): the route planner
    # publishes a DENSE nav_msgs/Path on global_plan, re-anchored at the
    # vehicle and republished continuously. Once airborne, the bridge
    # pure-pursuits it: term_goal = the path point a lookahead distance
    # ahead of the vehicle's projection onto the path, sliding to the
    # path end. The moving carrot keeps MIGHTY in TRAVELING (no
    # GOAL_REACHED idles mid-route), so the controller timeline extends
    # continuously and no per-leg mode resets are needed.
    # ------------------------------------------------------------------

    def _global_plan_cb(self, msg):
        if not msg.poses:
            return
        poses = list(msg.poses)
        with self._lock:
            first = self._follow_plan is None
            self._follow_plan = poses
        if first:
            self.get_logger().info(
                f'follower: adopted global_plan ({len(poses)} poses)')

    def _follow_tick(self):
        if self._task_active or not self._airborne:
            return
        with self._lock:
            plan = self._follow_plan
        if plan is None:
            return
        final = plan[-1].pose.position
        done = self._follow_done_plan
        if done is not None:
            dx = final.x - done[0]
            dy = final.y - done[1]
            dz = final.z - done[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < 2.0:
                return  # this route was already completed
        if self._follow_thread is not None and self._follow_thread.is_alive():
            return
        self._follow_thread = threading.Thread(
            target=self._follow_route, daemon=True)
        self._follow_thread.start()

    def _carrot(self, poses):
        """Path point ~follow_lookahead_m beyond the vehicle's projection.

        The walk CLAMPS at sharp path-direction reversals (route
        checkpoints on out-and-back legs): a lookahead measured purely
        along the path wraps around hairpins and can land back on the
        vehicle (observed: stable hover deadlock 3.8 m short of a
        checkpoint). Clamping makes the vehicle actually reach the
        corner — which is also what lets the route planner's own
        arrival radius trigger and drop the checkpoint from the path.
        The clamp releases once the vehicle is within 1.5 m of the
        corner (the walk then continues with a fresh direction
        reference, covering the ~1 s of stale path before the planner
        republishes without the reached checkpoint).
        """
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        p = odom.pose.pose.position
        # nearest pose index (paths from the study planner are anchored at
        # the vehicle, so this is usually 0; static paths also work)
        best_i, best_d = 0, float('inf')
        for i, ps in enumerate(poses):
            q = ps.pose.position
            d = (q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2
            if d < best_d:
                best_d, best_i = d, i

        def seg_dir(a, b):
            v = (b.x - a.x, b.y - a.y, b.z - a.z)
            n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-6 else None

        acc = 0.0
        ref_dir = None
        prev = poses[best_i].pose.position
        for ps in poses[best_i + 1:]:
            q = ps.pose.position
            d = seg_dir(prev, q)
            step = math.dist((prev.x, prev.y, prev.z), (q.x, q.y, q.z))
            prev = q
            acc += step
            if d is not None:
                if ref_dir is None:
                    ref_dir = d
                else:
                    dot = (ref_dir[0] * d[0] + ref_dir[1] * d[1]
                           + ref_dir[2] * d[2])
                    if dot < -0.17:  # direction reversed > ~100 deg
                        near = math.dist((p.x, p.y, p.z), (q.x, q.y, q.z))
                        if near > 1.5:
                            return ps  # clamp at the corner until reached
                        ref_dir = d    # corner reached: release, walk on
                        acc = 0.0
            if acc >= self.follow_lookahead:
                return ps
        return poses[-1]

    def _follow_route(self):
        self.get_logger().info('follower: engaging (lookahead '
                               f'{self.follow_lookahead} m)')
        # One-time controller timeline reset (duplicates — and therefore
        # does not require — the reference mission glue's mode switch).
        with self._lock:
            self._route_active = True
        self._set_mode(TrajectoryMode.Request.TRACK)
        time.sleep(0.2)
        self._set_mode(TrajectoryMode.Request.ADD_SEGMENT)

        last_goal = None
        last_pub = 0.0
        while rclpy.ok():
            if self._task_active:
                self.get_logger().info('follower: yielding to NavigateTask')
                with self._lock:
                    self._route_active = False
                return
            with self._lock:
                plan = self._follow_plan
            if plan is None:
                with self._lock:
                    self._route_active = False
                return
            final = plan[-1].pose.position
            d_final = self._distance_to(final)
            if d_final is not None and d_final < self.waypoint_tolerance:
                self._publish_term_goal(plan[-1])
                self.get_logger().info(
                    f'follower: route complete ({d_final:.2f} m from end)')
                with self._lock:
                    self._follow_done_plan = (final.x, final.y, final.z)
                    self._follow_plan = None
                    self._route_active = False
                return
            # Catch-up gate: if MIGHTY has gone idle (no trajectories) with
            # its committed end still far from the vehicle, withhold new
            # carrots — the controller is still flying the remaining path to
            # that end. Publishing a goal now would make MIGHTY replan from
            # the far end (spatial gap -> uncommanded straight line after the
            # timeline reset). Resume once the vehicle has caught up.
            with self._lock:
                idle = (self._last_traj_time > 0 and
                        time.monotonic() - self._last_traj_time > 2.0)
                traj_end = self._traj_end
            if idle and traj_end is not None:
                d_end = self._distance_to_xyz(*traj_end)
                if d_end is not None and d_end > 2.5:
                    time.sleep(0.3)
                    continue
            carrot = self._carrot(plan)
            if carrot is not None:
                c = carrot.pose.position
                # Defensive: never hand MIGHTY a goal at the vehicle's own
                # position (goal_radius would flip it to GOAL_REACHED and it
                # stops planning — the hover-deadlock failure mode). Skip
                # this tick; the re-anchored path resolves it next second.
                d_c = self._distance_to(c)
                if d_c is not None and d_c < 1.0 and carrot is not plan[-1]:
                    time.sleep(0.3)
                    continue
                now = time.monotonic()
                moved = (last_goal is None or
                         math.dist((c.x, c.y, c.z), last_goal) > 1.0)
                if moved or now - last_pub > self.republish_s:
                    self._publish_term_goal(carrot)
                    last_goal = (c.x, c.y, c.z)
                    last_pub = now
            time.sleep(0.3)

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

    def _distance_to_xyz(self, x, y, z):
        with self._lock:
            odom = self._odom
        if odom is None:
            return None
        p = odom.pose.pose.position
        return math.dist((p.x, p.y, p.z), (x, y, z))

    def _publish_term_goal(self, pose_stamped):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_stamped.header.frame_id or self.world_frame
        msg.pose = pose_stamped.pose
        self.term_goal_pub.publish(msg)

    def _finish(self, goal_handle, success, message):
        with self._lock:
            self._route_active = False
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

        with self._lock:
            self._route_active = True
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
                        # Reset the controller timeline before the new leg:
                        # after a leg completes the controller idles at the
                        # trajectory end while virtual_time keeps advancing,
                        # so the next leg's segment would splice at a past
                        # time and merge() would reject it forever. TRACK
                        # clears the trajectory; ADD_SEGMENT (from TRACK)
                        # zeroes virtual_time so the new leg merges cleanly.
                        self._set_mode(TrajectoryMode.Request.TRACK)
                        time.sleep(0.2)
                        self._set_mode(TrajectoryMode.Request.ADD_SEGMENT)
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
