#!/usr/bin/env python3

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from tf2_ros import Buffer, TransformException, TransformListener
    TF_AVAILABLE = True
except Exception:
    Buffer = None
    TransformException = Exception
    TransformListener = None
    TF_AVAILABLE = False

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except Exception:
    ActionClient = None
    NavigateToPose = None
    NAV2_AVAILABLE = False


class FrontierServiceNode(Node):
    def __init__(self):
        super().__init__('frontier_service_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('start_service', '/frontier/start')
        self.declare_parameter('stop_service', '/frontier/stop')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('goal_interval_s', 3.0)
        self.declare_parameter('frontier_sample_step', 3)
        self.declare_parameter('min_frontier_points', 30)
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('min_goal_distance_m', 0.4)

        self.map_topic = self.get_parameter('map_topic').value
        self.start_service_name = self.get_parameter('start_service').value
        self.stop_service_name = self.get_parameter('stop_service').value
        self.navigate_action_name = self.get_parameter('navigate_action').value
        self.goal_interval_s = float(self.get_parameter('goal_interval_s').value)
        self.frontier_sample_step = int(self.get_parameter('frontier_sample_step').value)
        self.min_frontier_points = int(self.get_parameter('min_frontier_points').value)
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.min_goal_distance_m = float(self.get_parameter('min_goal_distance_m').value)

        self.active = False
        self.last_map: Optional[OccupancyGrid] = None
        self.goal_in_progress = False
        self.goal_handle = None
        self.last_goal_time = 0.0
        self._missing_nav2_warned = False

        self.last_map_stamp_ns = 0
        self.last_frontier_count = 0
        self.prev_active = None
        self.prev_goal_in_progress = None
        self.last_diag_log_time = 0.0
        self.diag_log_period_s = 1.0
        self.last_tf_warn_time = 0.0

        self.tf_buffer = None
        self.tf_listener = None
        if TF_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn(
                'tf2_ros unavailable; frontier selection falls back to map-centroid heuristic.'
            )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )

        self.start_srv = self.create_service(Trigger, self.start_service_name, self.on_start)
        self.stop_srv = self.create_service(Trigger, self.stop_service_name, self.on_stop)

        self.nav_client = None
        if NAV2_AVAILABLE:
            self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action_name)
        else:
            self.get_logger().warn(
                'nav2_msgs/rclpy.action not available. Services will run, but no navigation goals will be sent.'
            )

        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f'FrontierServiceNode ready. map_topic={self.map_topic}, '
            f'start={self.start_service_name}, stop={self.stop_service_name}'
        )
        self.get_logger().info(
            f'[FLAG] Params | goal_interval_s={self.goal_interval_s} '
            f'frontier_sample_step={self.frontier_sample_step} '
            f'min_frontier_points={self.min_frontier_points} '
            f'navigate_action={self.navigate_action_name} nav2_available={NAV2_AVAILABLE} '
            f'robot_base_frame={self.robot_base_frame} min_goal_distance_m={self.min_goal_distance_m} '
            f'tf_available={TF_AVAILABLE}'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.last_map = msg
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns != self.last_map_stamp_ns:
            self.get_logger().info(
                f'[FLAG] map_callback | frame={msg.header.frame_id or "<empty>"} '
                f'size={msg.info.width}x{msg.info.height} res={msg.info.resolution:.4f}'
            )
            self.last_map_stamp_ns = stamp_ns

    def on_start(self, request, response):
        del request
        self.get_logger().info(
            f'[FLAG] /frontier/start called | active_before={self.active} '
            f'goal_in_progress={self.goal_in_progress}'
        )
        self.active = True
        response.success = True
        if NAV2_AVAILABLE:
            response.message = 'Frontier exploration activated.'
        else:
            response.message = 'Frontier exploration activated, but Nav2 action dependencies are unavailable.'
        self.get_logger().info(response.message)
        return response

    def on_stop(self, request, response):
        del request
        self.get_logger().info(
            f'[FLAG] /frontier/stop called | active_before={self.active} '
            f'goal_in_progress={self.goal_in_progress} has_goal_handle={self.goal_handle is not None}'
        )
        self.active = False

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.get_logger().info('Requested cancel for active NavigateToPose goal.')
            except Exception as exc:
                self.get_logger().warn(f'Failed to cancel active goal: {exc}')

        self.goal_in_progress = False
        self.goal_handle = None

        response.success = True
        response.message = 'Frontier exploration deactivated.'
        self.get_logger().info(response.message)
        return response

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.log_flag_changes(now)

        if not self.active:
            return

        if not NAV2_AVAILABLE or self.nav_client is None:
            if not self._missing_nav2_warned:
                self._missing_nav2_warned = True
                self.get_logger().warn('Exploration is active, but Nav2 is unavailable. Install nav2_msgs and run Nav2.')
            return

        if self.goal_in_progress:
            self.get_logger().debug('[FLAG] tick | goal already in progress, waiting for result.')
            return

        if (now - self.last_goal_time) < self.goal_interval_s:
            wait_left = self.goal_interval_s - (now - self.last_goal_time)
            self.get_logger().debug(
                f'[FLAG] tick | goal dispatch throttled, wait_left={max(0.0, wait_left):.2f}s'
            )
            return

        if self.last_map is None:
            self.get_logger().warn('Exploration active but no /map received yet.')
            self.last_goal_time = now
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(f'Waiting for action server: {self.navigate_action_name}')
            self.last_goal_time = now
            return

        self.get_logger().info('[FLAG] tick | attempting frontier extraction from latest map.')
        goal_pose = self.find_frontier_goal(self.last_map)
        if goal_pose is None:
            self.get_logger().warn(
                f'No valid frontier points found in current map. '
                f'last_frontier_count={self.last_frontier_count} min_required={self.min_frontier_points}'
            )
            self.last_goal_time = now
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.goal_in_progress = True
        self.last_goal_time = now

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

        self.get_logger().info(
            f'Sent frontier goal x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}, frame={goal_pose.header.frame_id}'
        )
        self.get_logger().info('[FLAG] NavigateToPose send_goal_async dispatched.')

    def on_goal_response(self, future):
        self.get_logger().info('[FLAG] on_goal_response callback invoked.')
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.goal_in_progress = False
            self.get_logger().warn(f'NavigateToPose goal send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.goal_in_progress = False
            self.goal_handle = None
            self.get_logger().warn('NavigateToPose goal was rejected.')
            return

        self.goal_handle = goal_handle
        self.get_logger().info('[FLAG] NavigateToPose goal accepted by action server.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)
        self.get_logger().info('[FLAG] Waiting for NavigateToPose result...')

    def on_goal_result(self, future):
        self.goal_in_progress = False
        self.goal_handle = None
        self.get_logger().info('[FLAG] on_goal_result callback invoked. goal_in_progress set False.')

        try:
            result = future.result()
            status = getattr(result, 'status', None)
            self.get_logger().info(f'NavigateToPose goal completed. status={status}')
        except Exception as exc:
            self.get_logger().warn(f'NavigateToPose result failed: {exc}')

    def find_frontier_goal(self, grid: OccupancyGrid) -> Optional[PoseStamped]:
        width = grid.info.width
        height = grid.info.height
        if width == 0 or height == 0:
            self.last_frontier_count = 0
            self.get_logger().warn('[FLAG] find_frontier_goal | empty map dimensions.')
            return None

        data = np.array(grid.data, dtype=np.int16).reshape((height, width))
        free = data == 0
        unknown = data == -1

        points = []
        step = max(1, self.frontier_sample_step)

        for y in range(1, height - 1, step):
            row_free = free[y]
            for x in range(1, width - 1, step):
                if not row_free[x]:
                    continue

                neighborhood = unknown[y - 1:y + 2, x - 1:x + 2]
                if np.any(neighborhood):
                    points.append((x, y))

        self.last_frontier_count = len(points)
        self.get_logger().info(
            f'[FLAG] find_frontier_goal | frontier_candidates={self.last_frontier_count} '
            f'min_required={self.min_frontier_points} sample_step={step}'
        )

        if len(points) < self.min_frontier_points:
            return None

        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        robot_map_xy = self.lookup_robot_map_xy(grid.header.frame_id if grid.header.frame_id else 'map')
        gx = None
        gy = None

        if robot_map_xy is not None:
            rx, ry = robot_map_xy
            rix = (rx - ox) / res
            riy = (ry - oy) / res

            best_cost = float('inf')
            best = None
            min_dist_cells = max(1.0, self.min_goal_distance_m / max(1e-6, res))

            for px, py in points:
                dx = float(px) - rix
                dy = float(py) - riy
                d2 = dx * dx + dy * dy
                if d2 < (min_dist_cells * min_dist_cells):
                    continue
                if d2 < best_cost:
                    best_cost = d2
                    best = (px, py)

            if best is not None:
                gx, gy = best

        if gx is None or gy is None:
            arr = np.array(points, dtype=np.float32)
            cx = float(np.mean(arr[:, 0]))
            cy = float(np.mean(arr[:, 1]))
            idx = int(np.argmin((arr[:, 0] - cx) ** 2 + (arr[:, 1] - cy) ** 2))
            gx, gy = points[idx]

        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = grid.header.frame_id if grid.header.frame_id else 'map'
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        self.get_logger().info(
            f'[FLAG] Selected frontier goal | grid=({gx},{gy}) world=({wx:.2f},{wy:.2f}) '
            f'frame={pose.header.frame_id}'
        )
        return pose

    def lookup_robot_map_xy(self, map_frame: str):
        if not TF_AVAILABLE or self.tf_buffer is None:
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.15),
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            return tx, ty
        except TransformException as exc:
            now_t = self.get_clock().now().nanoseconds * 1e-9
            if (now_t - self.last_tf_warn_time) > 2.0:
                self.last_tf_warn_time = now_t
                self.get_logger().warn(
                    f'Unable to get robot pose {map_frame}->{self.robot_base_frame}; using fallback goal selection. '
                    f'error={exc}'
                )
            return None

    def log_flag_changes(self, now_t: float):
        if self.active != self.prev_active:
            self.get_logger().info(f'[FLAG] active changed | {self.prev_active} -> {self.active}')
            self.prev_active = self.active

        if self.goal_in_progress != self.prev_goal_in_progress:
            self.get_logger().info(
                f'[FLAG] goal_in_progress changed | {self.prev_goal_in_progress} -> {self.goal_in_progress}'
            )
            self.prev_goal_in_progress = self.goal_in_progress

        if (now_t - self.last_diag_log_time) >= self.diag_log_period_s:
            map_ready = self.last_map is not None
            map_frame = self.last_map.header.frame_id if map_ready else '<none>'
            map_size = f'{self.last_map.info.width}x{self.last_map.info.height}' if map_ready else '0x0'
            self.get_logger().info(
                f'[FLAG] Snapshot | active={self.active} goal_in_progress={self.goal_in_progress} '
                f'has_goal_handle={self.goal_handle is not None} map_ready={map_ready} '
                f'map_frame={map_frame} map_size={map_size} '
                f'last_frontier_count={self.last_frontier_count} nav2_available={NAV2_AVAILABLE}'
            )
            self.last_diag_log_time = now_t


def main(args=None):
    rclpy.init(args=args)
    node = FrontierServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
