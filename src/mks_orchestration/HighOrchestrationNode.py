#!/usr/bin/env python3

import json
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
    TF_AVAILABLE = True
except Exception:
    Buffer = None
    TransformListener = None
    TransformException = Exception
    do_transform_pose_stamped = None
    TF_AVAILABLE = False


class CubeTrack:
    def __init__(self, cube_id, x, y):
        self.id = cube_id
        self.pos_xy = [x, y]
        self.hits = 1
        self.last_seen_time = time.time()
        self.status = 'NEW'


class BTStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class BTNode:
    def tick(self):
        raise NotImplementedError


class SequenceNode(BTNode):
    def __init__(self, children):
        self.children = children
        self.current_index = 0

    def tick(self):
        while self.current_index < len(self.children):
            result = self.children[self.current_index].tick()
            if result == BTStatus.SUCCESS:
                self.current_index += 1
                continue
            return result
        return BTStatus.SUCCESS


class ParallelAllNode(BTNode):
    def __init__(self, children):
        self.children = children

    def tick(self):
        saw_running = False
        for child in self.children:
            result = child.tick()
            if result == BTStatus.FAILURE:
                return BTStatus.FAILURE
            if result == BTStatus.RUNNING:
                saw_running = True
        return BTStatus.RUNNING if saw_running else BTStatus.SUCCESS


def distance2d(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return (dx * dx + dy * dy) ** 0.5


class EnsureFrontierExplorationRunning(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'FRONTIER_EXPLORATION'
        self.owner.get_logger().debug(
            f'[FLAG] EnsureFrontierExplorationRunning tick | '
            f'frontier_started={self.owner.frontier_started} '
            f'frontier_start_future_active={self.owner.frontier_start_future is not None}'
        )

        if self.owner.frontier_started:
            self.owner.get_logger().debug('[FLAG] Frontier already started. Node returns SUCCESS.')
            return BTStatus.SUCCESS

        self.owner.get_logger().debug('[FLAG] Frontier not started yet. Attempting start service call.')
        self.owner.call_frontier_start_if_needed()
        return BTStatus.RUNNING


class WaitForFourCubes(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'SEARCH_CUBES'
        confirmed = self.owner.count_confirmed_cubes()
        self.owner.get_logger().debug(
            f'[FLAG] WaitForFourCubes tick | confirmed={confirmed} target={self.owner.target_cube_count}'
        )
        if confirmed >= self.owner.target_cube_count:
            self.owner.get_logger().info('[FLAG] Target cube count reached. Node returns SUCCESS.')
            return BTStatus.SUCCESS
        return BTStatus.RUNNING


class StopFrontierExploration(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'STOP_FRONTIER_EXPLORATION'
        self.owner.get_logger().debug(
            f'[FLAG] StopFrontierExploration tick | '
            f'frontier_stopped={self.owner.frontier_stopped} '
            f'frontier_stop_future_active={self.owner.frontier_stop_future is not None}'
        )

        if self.owner.frontier_stopped:
            self.owner.get_logger().debug('[FLAG] Frontier already stopped. Node returns SUCCESS.')
            return BTStatus.SUCCESS

        self.owner.get_logger().debug('[FLAG] Frontier not stopped yet. Attempting stop service call.')
        self.owner.call_frontier_stop_if_needed()
        return BTStatus.RUNNING


class SafeStop(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'SAFE_STOP'
        self.owner.get_logger().info('[FLAG] SafeStop tick | publishing zero cmd_vel')
        self.owner.publish_cmd_vel(0.0, 0.0)
        return BTStatus.SUCCESS


class HighOrchestrationNode(Node):
    def __init__(self):
        super().__init__('high_orchestration_node')

        self.cube_sub = self.create_subscription(PoseArray, '/cube_poses', self.cube_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) #Publish to /cmd_vel
        self.state_pub = self.create_publisher(String, '/orchestrator/state', 10)
        self.cubes_json_pub = self.create_publisher(String, '/orchestrator/cubes_json', 10)
        self.frontier_start_client = self.create_client(Trigger, '/frontier/start') #Creates a service client for the /frontier/start service
        self.frontier_stop_client = self.create_client(Trigger, '/frontier/stop') #Creates a service client for the /frontier/stop service

        self.cube_tracks = []
        self.next_cube_id = 1

        self.declare_parameter('cube_publish_hz', 1.0)
        self.declare_parameter('cube_association_radius', 0.40)
        self.declare_parameter('cube_confirm_hits', 3)
        self.declare_parameter('cube_track_timeout_s', 6.0)
        self.declare_parameter('target_cube_count', 4)
        self.declare_parameter('frontier_request_retry_s', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cube_transform_timeout_s', 0.15)
        # Arena boundary parameters. Set arena_width > 0 and arena_height > 0 to enable.
        # start_x/start_y define the lower-left corner of the arena in the map frame.
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('arena_width', 0.0)
        self.declare_parameter('arena_height', 0.0)
        self.declare_parameter('boundary_safety_margin', 0.2)
        self.declare_parameter('odom_topic', '/odom')

        self.cube_publish_hz = float(self.get_parameter('cube_publish_hz').value)
        self.cube_association_radius = float(self.get_parameter('cube_association_radius').value)
        self.cube_confirm_hits = int(self.get_parameter('cube_confirm_hits').value)
        self.cube_track_timeout_s = float(self.get_parameter('cube_track_timeout_s').value)
        self.target_cube_count = int(self.get_parameter('target_cube_count').value)
        self.frontier_request_retry_s = float(self.get_parameter('frontier_request_retry_s').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.cube_transform_timeout_s = float(self.get_parameter('cube_transform_timeout_s').value)

        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        arena_width = float(self.get_parameter('arena_width').value)
        arena_height = float(self.get_parameter('arena_height').value)
        self.boundary_safety_margin = float(self.get_parameter('boundary_safety_margin').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self.boundary_enabled = (arena_width > 0.0 and arena_height > 0.0)
        self.arena_min_x = start_x
        self.arena_max_x = start_x + arena_width
        self.arena_min_y = start_y
        self.arena_max_y = start_y + arena_height

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_pose_received = False

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.last_cube_publish_time = time.time()

        self.frontier_start_request_time = 0.0
        self.frontier_stop_request_time = 0.0
        self.frontier_start_future = None
        self.frontier_stop_future = None
        self.frontier_started = False
        self.frontier_stopped = False
        self.mission_complete_logged = False

        self.prev_bt_state = ''
        self.prev_frontier_started = None
        self.prev_frontier_stopped = None
        self.prev_confirmed_count = -1
        self.last_diag_log_time = 0.0
        self.diag_log_period_s = 1.0
        self.last_tf_warn_time = 0.0
        self.last_tf_unavailable_warn_time = 0.0

        self.tf_buffer = None
        self.tf_listener = None
        
        if TF_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn(
                'tf2_ros/tf2_geometry_msgs unavailable. Cube poses are accepted only when already in map frame.'
            )

        self.current_bt_state = 'INIT'
        self.behavior_tree = SequenceNode([
            ParallelAllNode([
                EnsureFrontierExplorationRunning(self),
                WaitForFourCubes(self),
            ]),
            StopFrontierExploration(self),
            SafeStop(self),
        ])

        self.get_logger().info('HighOrchestrationNode started with behavior tree control.')
        self.get_logger().info(
            f'[FLAG] Params | cube_association_radius={self.cube_association_radius} '
            f'cube_confirm_hits={self.cube_confirm_hits} cube_track_timeout_s={self.cube_track_timeout_s} '
            f'target_cube_count={self.target_cube_count} frontier_retry_s={self.frontier_request_retry_s} '
            f'map_frame={self.map_frame} tf_available={TF_AVAILABLE}'
        )
        if self.boundary_enabled:
            self.get_logger().info(
                f'[BOUNDARY] Enabled | '
                f'x=[{self.arena_min_x:.2f}, {self.arena_max_x:.2f}] '
                f'y=[{self.arena_min_y:.2f}, {self.arena_max_y:.2f}] '
                f'margin={self.boundary_safety_margin:.2f} odom_topic={odom_topic}'
            )
        else:
            self.get_logger().info('[BOUNDARY] Disabled (set arena_width and arena_height > 0 to enable)')
        self.get_logger().info('[FLAG] Initial flags | frontier_started=False frontier_stopped=False')
        self.create_timer(0.1, self.bt_tick)

    def bt_tick(self):
        now_t = time.time()
        self.prune_cube_tracks()

        if now_t - self.last_cube_publish_time >= 1.0 / self.cube_publish_hz:
            self.publish_cubes_json_map_frame()
            self.last_cube_publish_time = now_t

        if self.is_out_of_bounds():
            self.get_logger().warn(
                f'[BOUNDARY] Out of bounds at ({self.robot_x:.2f}, {self.robot_y:.2f}) '
                f'| x=[{self.arena_min_x:.2f},{self.arena_max_x:.2f}] '
                f'y=[{self.arena_min_y:.2f},{self.arena_max_y:.2f}] '
                f'margin={self.boundary_safety_margin:.2f} — stopping rover'
            )
            self.call_frontier_stop_if_needed()
            self.publish_cmd_vel(0.0, 0.0)
            self.publish_state_string()
            return

        bt_result = self.behavior_tree.tick()
        if bt_result == BTStatus.FAILURE:
            self.get_logger().error('[FLAG] BT tick result=FAILURE — mission cannot continue')
        elif bt_result == BTStatus.SUCCESS and not self.mission_complete_logged:
            self.mission_complete_logged = True
            self.get_logger().info('[FLAG] MISSION COMPLETE | BT returned SUCCESS')
        self.log_flag_changes(now_t)
        self.publish_state_string()

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_pose_received = True
        self.get_logger().debug(
            f'[FLAG] odom_callback | x={self.robot_x:.3f} y={self.robot_y:.3f}'
        )

    def is_out_of_bounds(self):
        if not self.boundary_enabled or not self.robot_pose_received:
            return False
        m = self.boundary_safety_margin
        return (
            self.robot_x < self.arena_min_x + m
            or self.robot_x > self.arena_max_x - m
            or self.robot_y < self.arena_min_y + m
            or self.robot_y > self.arena_max_y - m
        )

    def cube_callback(self, msg: PoseArray):
        source_frame = msg.header.frame_id
        if not source_frame:
            self.get_logger().warn(
                f'[FLAG] cube_callback received message with empty frame_id — assuming map_frame={self.map_frame}'
            )
            source_frame = self.map_frame
        self.get_logger().debug(
            f'[FLAG] cube_callback received {len(msg.poses)} pose(s) frame={source_frame}'
        )
        for pose in msg.poses:
            map_xy = self.to_map_xy(pose, source_frame)
            if map_xy is None:
                continue
            self.update_cube_tracks(map_xy)

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug(
            f'[FLAG] Published /cmd_vel | linear_x={linear_x:.3f} angular_z={angular_z:.3f}'
        )

    def to_map_xy(self, pose, source_frame):
        if source_frame == self.map_frame:
            return [pose.position.x, pose.position.y]

        if not TF_AVAILABLE or self.tf_buffer is None or do_transform_pose_stamped is None:
            now_t = time.time()
            if (now_t - self.last_tf_unavailable_warn_time) > 5.0:
                self.last_tf_unavailable_warn_time = now_t
                self.get_logger().error(
                    f'[FLAG] to_map_xy: need TF transform {source_frame}->{self.map_frame} '
                    f'but tf2 is unavailable — cube poses will be dropped'
                )
            return None

        pose_in = PoseStamped()
        pose_in.header.stamp = self.get_clock().now().to_msg()
        pose_in.header.frame_id = source_frame
        pose_in.pose = pose

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.cube_transform_timeout_s),
            )
            pose_out = do_transform_pose_stamped(pose_in, transform)
            return [pose_out.pose.position.x, pose_out.pose.position.y]
        except TransformException as exc:
            now_t = time.time()
            if (now_t - self.last_tf_warn_time) > 2.0:
                self.last_tf_warn_time = now_t
                self.get_logger().warn(
                    f'Failed TF transform {source_frame}->{self.map_frame}; skipping cube update. error={exc}'
                )
            return None

    def update_cube_tracks(self, cube_xy):
        best_track = None
        best_dist = float('inf')

        for track in self.cube_tracks:
            d = distance2d(track.pos_xy[0], track.pos_xy[1], cube_xy[0], cube_xy[1])
            if d < best_dist:
                best_dist = d
                best_track = track

        if best_track is not None and best_dist <= self.cube_association_radius:
            prev_hits = best_track.hits
            was_confirmed = (best_track.status == 'CONFIRMED')
            best_track.pos_xy[0] = (
                (best_track.pos_xy[0] * best_track.hits + cube_xy[0])
                / (best_track.hits + 1)
            )
            best_track.pos_xy[1] = (
                (best_track.pos_xy[1] * best_track.hits + cube_xy[1])
                / (best_track.hits + 1)
            )
            best_track.hits += 1
            best_track.last_seen_time = time.time()
            if best_track.hits >= self.cube_confirm_hits:
                best_track.status = 'CONFIRMED'
            self.get_logger().info(
                f'[FLAG] Updated track id={best_track.id} dist={best_dist:.3f} '
                f'hits={prev_hits}->{best_track.hits} status={best_track.status} '
                f'pos=({best_track.pos_xy[0]:.3f},{best_track.pos_xy[1]:.3f})'
            )
            if (not was_confirmed) and best_track.status == 'CONFIRMED':
                self.get_logger().info(
                    f'[FLAG] Track id={best_track.id} reached CONFIRMED at hits={best_track.hits}'
                )
            return

        new_track = CubeTrack(self.next_cube_id, cube_xy[0], cube_xy[1])
        self.next_cube_id += 1
        self.cube_tracks.append(new_track)
        self.get_logger().info(
            f'[FLAG] Created new track id={new_track.id} pos=({new_track.pos_xy[0]:.3f},{new_track.pos_xy[1]:.3f})'
        )

    def prune_cube_tracks(self):
        now_t = time.time()
        before = len(self.cube_tracks)
        self.cube_tracks = [
            t for t in self.cube_tracks
            if t.status == 'CONFIRMED' or (now_t - t.last_seen_time) <= self.cube_track_timeout_s
        ]
        removed = before - len(self.cube_tracks)
        if removed > 0:
            self.get_logger().info(
                f'[FLAG] Pruned stale tracks | removed={removed} remaining={len(self.cube_tracks)}'
            )

    def publish_cubes_json_map_frame(self):
        confirmed_list = []
        for track in self.cube_tracks:
            if track.status != 'CONFIRMED':
                continue

            confirmed_list.append({
                'id': track.id,
                'x': track.pos_xy[0],
                'y': track.pos_xy[1],
                'hits': track.hits,
                'status': 'CONFIRMED',
            })

        json_obj = {
            'timestamp': time.time(),
            'reference': self.map_frame,
            'cubes': confirmed_list,
        }

        msg = String()
        msg.data = json.dumps(json_obj)
        self.cubes_json_pub.publish(msg)
        self.get_logger().info(
            f'[FLAG] Published /orchestrator/cubes_json | confirmed_count={len(confirmed_list)}'
        )

    def publish_state_string(self):
        msg = String()
        msg.data = self.current_bt_state
        self.state_pub.publish(msg)
        self.get_logger().debug(f'[FLAG] Published /orchestrator/state | state={self.current_bt_state}')

    def count_confirmed_cubes(self):
        return sum(1 for t in self.cube_tracks if t.status == 'CONFIRMED')

    def log_flag_changes(self, now_t):
        confirmed = self.count_confirmed_cubes()

        if self.current_bt_state != self.prev_bt_state:
            self.get_logger().info(
                f'[FLAG] BT state transition | {self.prev_bt_state or "<none>"} -> {self.current_bt_state}'
            )
            self.prev_bt_state = self.current_bt_state

        if self.frontier_started != self.prev_frontier_started:
            self.get_logger().info(
                f'[FLAG] frontier_started changed | {self.prev_frontier_started} -> {self.frontier_started}'
            )
            self.prev_frontier_started = self.frontier_started

        if self.frontier_stopped != self.prev_frontier_stopped:
            self.get_logger().info(
                f'[FLAG] frontier_stopped changed | {self.prev_frontier_stopped} -> {self.frontier_stopped}'
            )
            self.prev_frontier_stopped = self.frontier_stopped

        if confirmed != self.prev_confirmed_count:
            self.get_logger().info(
                f'[FLAG] confirmed cubes changed | {self.prev_confirmed_count} -> {confirmed}'
            )
            self.prev_confirmed_count = confirmed

        if (now_t - self.last_diag_log_time) >= self.diag_log_period_s:
            self.get_logger().info(
                f'[FLAG] Snapshot | state={self.current_bt_state} '
                f'frontier_started={self.frontier_started} frontier_stopped={self.frontier_stopped} '
                f'frontier_start_future_active={self.frontier_start_future is not None} '
                f'frontier_stop_future_active={self.frontier_stop_future is not None} '
                f'tracks_total={len(self.cube_tracks)} confirmed={confirmed}'
            )
            self.last_diag_log_time = now_t

    def call_frontier_start_if_needed(self):
        now_t = time.time()

        if self.frontier_start_future is not None:
            self.get_logger().debug('[FLAG] Frontier start request already in flight.')
            if self.frontier_start_future.done():
                try:
                    response = self.frontier_start_future.result()
                    self.frontier_started = bool(response.success)
                    if self.frontier_started:
                        self.get_logger().info(f'Frontier start acknowledged: {response.message}')
                    else:
                        self.get_logger().warn(f'Frontier start rejected: {response.message}')
                except Exception as exc:
                    self.get_logger().error(f'[FLAG] Frontier start call raised exception: {exc}')
                finally:
                    self.frontier_start_future = None
            return

        if (now_t - self.frontier_start_request_time) < self.frontier_request_retry_s:
            wait_left = self.frontier_request_retry_s - (now_t - self.frontier_start_request_time)
            self.get_logger().debug(f'[FLAG] Frontier start retry throttled | wait_left={max(0.0, wait_left):.2f}s')
            return

        if not self.frontier_start_client.wait_for_service(timeout_sec=0.1): #Check the service exists before calling
            self.frontier_start_request_time = now_t
            self.get_logger().warn('Waiting for /frontier/start service...')
            return

        self.frontier_start_request_time = now_t
        self.frontier_start_future = self.frontier_start_client.call_async(Trigger.Request()) #Send an asynchronous request to the /frontier/start service
        self.get_logger().info('[FLAG] Sent /frontier/start Trigger request.')

    def call_frontier_stop_if_needed(self):
        now_t = time.time()

        if self.frontier_stop_future is not None:
            self.get_logger().debug('[FLAG] Frontier stop request already in flight.')
            if self.frontier_stop_future.done():
                try:
                    response = self.frontier_stop_future.result()
                    self.frontier_stopped = bool(response.success)
                    if self.frontier_stopped:
                        self.get_logger().info(f'Frontier stop acknowledged: {response.message}')
                    else:
                        self.get_logger().warn(f'Frontier stop rejected: {response.message}')
                except Exception as exc:
                    self.get_logger().error(f'[FLAG] Frontier stop call raised exception: {exc}')
                finally:
                    self.frontier_stop_future = None
            return

        if (now_t - self.frontier_stop_request_time) < self.frontier_request_retry_s:
            wait_left = self.frontier_request_retry_s - (now_t - self.frontier_stop_request_time)
            self.get_logger().debug(f'[FLAG] Frontier stop retry throttled | wait_left={max(0.0, wait_left):.2f}s')
            return

        if not self.frontier_stop_client.wait_for_service(timeout_sec=0.1):
            self.frontier_stop_request_time = now_t
            self.get_logger().warn('Waiting for /frontier/stop service...')
            return

        self.frontier_stop_request_time = now_t
        self.frontier_stop_future = self.frontier_stop_client.call_async(Trigger.Request())
        self.get_logger().info('[FLAG] Sent /frontier/stop Trigger request.')


def main(args=None):
    rclpy.init(args=args)
    node = HighOrchestrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
