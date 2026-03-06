#!/usr/bin/env python3

import json
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Pose2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class CubeTrack:
    def __init__(self, cube_id, x, y):
        self.id = cube_id
        self.pos_odom_xy = [x, y]
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


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def distance2d(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


class WaitForOdom(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'WAIT_FOR_ODOM'
        if self.owner.odom_valid and self.owner.start_pose_valid:
            return BTStatus.SUCCESS
        self.owner.publish_cmd_vel(0.0, 0.0)
        return BTStatus.RUNNING


class DescendRamp(BTNode):
    def __init__(self, owner):
        self.owner = owner
        self.start_pose = None

    def tick(self):
        self.owner.current_bt_state = 'DESCEND_RAMP'
        if not self.owner.odom_valid:
            return BTStatus.RUNNING

        if self.start_pose is None:
            self.start_pose = Pose2D(
                self.owner.odom_pose.x,
                self.owner.odom_pose.y,
                self.owner.odom_pose.yaw,
            )

        dist = distance2d(
            self.owner.odom_pose.x,
            self.owner.odom_pose.y,
            self.start_pose.x,
            self.start_pose.y,
        )

        if dist < self.owner.ramp_exit_distance_m:
            self.owner.publish_cmd_vel(self.owner.ramp_forward_speed, 0.0)
            return BTStatus.RUNNING

        self.owner.publish_cmd_vel(0.0, 0.0)
        return BTStatus.SUCCESS


class EnsureFrontierExplorationRunning(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'FRONTIER_EXPLORATION'

        if self.owner.frontier_started:
            return BTStatus.SUCCESS

        self.owner.call_frontier_start_if_needed()
        return BTStatus.RUNNING


class WaitForFourCubes(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'SEARCH_CUBES'
        if self.owner.count_confirmed_cubes() >= self.owner.target_cube_count:
            return BTStatus.SUCCESS
        return BTStatus.RUNNING


class StopFrontierExploration(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'STOP_FRONTIER_EXPLORATION'

        if self.owner.frontier_stopped:
            return BTStatus.SUCCESS

        self.owner.call_frontier_stop_if_needed()
        return BTStatus.RUNNING


class SafeStop(BTNode):
    def __init__(self, owner):
        self.owner = owner

    def tick(self):
        self.owner.current_bt_state = 'SAFE_STOP'
        self.owner.publish_cmd_vel(0.0, 0.0)
        return BTStatus.SUCCESS


class HighOrchestrationNode(Node):
    def __init__(self):
        super().__init__('high_orchestration_node')

        self.cube_sub = self.create_subscription(PoseArray, '/cube_poses', self.cube_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/orchestrator/state', 10)
        self.cubes_json_pub = self.create_publisher(String, '/orchestrator/cubes_json', 10)
        self.frontier_start_client = self.create_client(Trigger, '/frontier/start')
        self.frontier_stop_client = self.create_client(Trigger, '/frontier/stop')

        self.odom_pose = None
        self.odom_valid = False
        self.start_odom_pose = None
        self.start_pose_valid = False

        self.cube_tracks = []
        self.next_cube_id = 1

        self.last_cube_publish_time = time.time()
        self.cube_publish_hz = 1.0

        self.ramp_exit_distance_m = 1.8
        self.ramp_forward_speed = 0.20
        self.cube_association_radius = 0.40
        self.cube_confirm_hits = 3
        self.cube_track_timeout_s = 6.0
        self.target_cube_count = 4

        self.frontier_start_request_time = 0.0
        self.frontier_stop_request_time = 0.0
        self.frontier_request_retry_s = 2.0
        self.frontier_start_future = None
        self.frontier_stop_future = None
        self.frontier_started = False
        self.frontier_stopped = False

        self.current_bt_state = 'INIT'
        self.behavior_tree = SequenceNode([
            WaitForOdom(self),
            DescendRamp(self),
            ParallelAllNode([
                EnsureFrontierExplorationRunning(self),
                WaitForFourCubes(self),
            ]),
            StopFrontierExploration(self),
            SafeStop(self),
        ])

        self.get_logger().info('HighOrchestrationNode started with behavior tree control.')
        self.create_timer(0.1, self.bt_tick)

    def bt_tick(self):
        now_t = time.time()
        self.prune_cube_tracks()

        if now_t - self.last_cube_publish_time >= 1.0 / self.cube_publish_hz:
            self.publish_cubes_json_relative_to_start()
            self.last_cube_publish_time = now_t

        _ = self.behavior_tree.tick()
        self.publish_state_string()

    def cube_callback(self, msg: PoseArray):
        if not self.odom_valid:
            return

        for pose in msg.poses:
            cube_odom_xy = [pose.position.x, pose.position.y]
            self.update_cube_tracks(cube_odom_xy)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_pose = Pose2D(x, y, yaw)
        self.odom_valid = True

        if not self.start_pose_valid:
            self.start_odom_pose = Pose2D(x, y, yaw)
            self.start_pose_valid = True

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def update_cube_tracks(self, cube_odom_xy):
        best_track = None
        best_dist = float('inf')

        for track in self.cube_tracks:
            d = distance2d(track.pos_odom_xy[0], track.pos_odom_xy[1], cube_odom_xy[0], cube_odom_xy[1])
            if d < best_dist:
                best_dist = d
                best_track = track

        if best_track is not None and best_dist <= self.cube_association_radius:
            best_track.pos_odom_xy[0] = (
                (best_track.pos_odom_xy[0] * best_track.hits + cube_odom_xy[0])
                / (best_track.hits + 1)
            )
            best_track.pos_odom_xy[1] = (
                (best_track.pos_odom_xy[1] * best_track.hits + cube_odom_xy[1])
                / (best_track.hits + 1)
            )
            best_track.hits += 1
            best_track.last_seen_time = time.time()
            if best_track.hits >= self.cube_confirm_hits:
                best_track.status = 'CONFIRMED'
            return

        new_track = CubeTrack(self.next_cube_id, cube_odom_xy[0], cube_odom_xy[1])
        self.next_cube_id += 1
        self.cube_tracks.append(new_track)

    def prune_cube_tracks(self):
        now_t = time.time()
        self.cube_tracks = [
            t for t in self.cube_tracks
            if (now_t - t.last_seen_time) <= self.cube_track_timeout_s
        ]

    def publish_cubes_json_relative_to_start(self):
        if not self.start_pose_valid:
            return

        confirmed_list = []
        for track in self.cube_tracks:
            if track.status != 'CONFIRMED':
                continue

            rel_x = track.pos_odom_xy[0] - self.start_odom_pose.x
            rel_y = track.pos_odom_xy[1] - self.start_odom_pose.y
            confirmed_list.append({
                'id': track.id,
                'x': rel_x,
                'y': rel_y,
                'hits': track.hits,
                'status': 'CONFIRMED',
            })

        json_obj = {
            'timestamp': time.time(),
            'reference': 'start_odom_pose',
            'start_odom': {
                'x': self.start_odom_pose.x,
                'y': self.start_odom_pose.y,
                'yaw': self.start_odom_pose.yaw,
            },
            'cubes': confirmed_list,
        }

        msg = String()
        msg.data = json.dumps(json_obj)
        self.cubes_json_pub.publish(msg)

    def publish_state_string(self):
        msg = String()
        msg.data = self.current_bt_state
        self.state_pub.publish(msg)

    def count_confirmed_cubes(self):
        return sum(1 for t in self.cube_tracks if t.status == 'CONFIRMED')

    def call_frontier_start_if_needed(self):
        now_t = time.time()

        if self.frontier_start_future is not None:
            if self.frontier_start_future.done():
                try:
                    response = self.frontier_start_future.result()
                    self.frontier_started = bool(response.success)
                    if self.frontier_started:
                        self.get_logger().info(f'Frontier start acknowledged: {response.message}')
                    else:
                        self.get_logger().warn(f'Frontier start rejected: {response.message}')
                except Exception as exc:
                    self.get_logger().warn(f'Frontier start call failed: {exc}')
                finally:
                    self.frontier_start_future = None
            return

        if (now_t - self.frontier_start_request_time) < self.frontier_request_retry_s:
            return

        if not self.frontier_start_client.wait_for_service(timeout_sec=0.1):
            self.frontier_start_request_time = now_t
            self.get_logger().warn('Waiting for /frontier/start service...')
            return

        self.frontier_start_request_time = now_t
        self.frontier_start_future = self.frontier_start_client.call_async(Trigger.Request())

    def call_frontier_stop_if_needed(self):
        now_t = time.time()

        if self.frontier_stop_future is not None:
            if self.frontier_stop_future.done():
                try:
                    response = self.frontier_stop_future.result()
                    self.frontier_stopped = bool(response.success)
                    if self.frontier_stopped:
                        self.get_logger().info(f'Frontier stop acknowledged: {response.message}')
                    else:
                        self.get_logger().warn(f'Frontier stop rejected: {response.message}')
                except Exception as exc:
                    self.get_logger().warn(f'Frontier stop call failed: {exc}')
                finally:
                    self.frontier_stop_future = None
            return

        if (now_t - self.frontier_stop_request_time) < self.frontier_request_retry_s:
            return

        if not self.frontier_stop_client.wait_for_service(timeout_sec=0.1):
            self.frontier_stop_request_time = now_t
            self.get_logger().warn('Waiting for /frontier/stop service...')
            return

        self.frontier_stop_request_time = now_t
        self.frontier_stop_future = self.frontier_stop_client.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = HighOrchestrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
