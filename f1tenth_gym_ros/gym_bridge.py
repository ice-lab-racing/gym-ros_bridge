# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from f1tenth_gym_ros.sensor_model import measured_speed, noisy_scan, odom_step

import gymnasium as gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # defaults mirror config/sim.yaml, they also fix the type of each parameter
        self.declare_parameters('', [
            ('ego_namespace', 'ego_racecar'),
            ('ego_gt_topic', 'ground_truth'),
            ('ego_opp_gt_topic', 'opp_ground_truth'),
            ('ego_scan_topic', 'scan'),
            ('ego_drive_topic', 'drive'),
            ('opp_namespace', 'opp_racecar'),
            ('opp_gt_topic', 'ground_truth'),
            ('opp_ego_gt_topic', 'opp_ground_truth'),
            ('opp_scan_topic', 'opp_scan'),
            ('opp_drive_topic', 'opp_drive'),
            ('scan_distance_to_base_link', 0.275),
            ('scan_fov', 4.7),
            ('scan_beams', 1080),
            ('map_path', '/sim_ws/src/gym-ros_bridge/maps/levine'),
            ('map_img_ext', '.png'),
            ('num_agent', 1),
            ('sx', 0.0),
            ('sy', 0.0),
            ('stheta', 0.0),
            ('sx1', 2.0),
            ('sy1', 0.5),
            ('stheta1', 0.0),
            ('kb_teleop', True),
            # localization and odometry
            ('localization', 'ground_truth'),
            ('odom_frame', 'odom'),
            ('ego_reset_topic', '/sim/reset_pose'),
            ('opp_reset_topic', '/sim/opp_reset_pose'),
            ('localizer_reset_topic', '/initialpose'),
            # episode end
            ('reset_on_laps', True),
            ('odom_topic', '/odom'),
            ('odom_rate', 50.0),
            ('wheelbase', 0.3302),
            # sensor errors
            ('sensor_noise', True),
            ('noise_seed', 0),
            ('odom_speed_scale_error', 0.02),
            ('odom_speed_noise', 0.02),
            ('odom_steer_offset', 0.01),
            ('scan_rate', 40.0),
            ('scan_range_std', 0.03),
            ('scan_range_max', 10.0),
            ('scan_dropout', 0.01),
        ])

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        self.localization = self.get_parameter('localization').value
        self.reset_on_laps = self.get_parameter('reset_on_laps').value
        if self.localization not in ['ground_truth', 'odometry']:
            raise ValueError('localization should be either ground_truth or odometry.')
        self.odom_frame = self.get_parameter('odom_frame').value

        noise = self.get_parameter('sensor_noise').value
        seed = self.get_parameter('noise_seed').value
        self.odom_rng = np.random.default_rng(None if seed < 0 else seed)
        self.scan_rng = np.random.default_rng(None if seed < 0 else seed + 1)
        self.wheelbase = self.get_parameter('wheelbase').value
        self.speed_scale_error = self.get_parameter('odom_speed_scale_error').value if noise else 0.0
        self.speed_noise = self.get_parameter('odom_speed_noise').value if noise else 0.0
        self.steer_offset = self.get_parameter('odom_steer_offset').value if noise else 0.0
        self.scan_range_std = self.get_parameter('scan_range_std').value if noise else 0.0
        self.scan_dropout = self.get_parameter('scan_dropout').value if noise else 0.0
        self.scan_range_max = self.get_parameter('scan_range_max').value

        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents,
                            lidar_dist=self.get_parameter("scan_distance_to_base_link").value,
                            num_beams=scan_beams,
                            fov=scan_fov
                            )

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.odom_pose = [0.0, 0.0, 0.0]        # VESC odometry: starts at 0 wherever the car is
        self.odom_speed = 0.0
        self.odom_yaw_rate = 0.0
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        # the first and last beam sit on the fov limits, as in the gym's scan simulator
        self.angle_inc = scan_fov / (scan_beams - 1)
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_gt_topic = self.ego_namespace + '/' + self.get_parameter('ego_gt_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            # poses the sim goes back to when an episode terminates
            self.start_poses = [[sx, sy, stheta], [sx1, sy1, stheta1]]
            self._reset(self.start_poses)

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_gt_topic = self.opp_namespace + '/' + self.get_parameter('opp_gt_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_gt_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_gt_topic').value
            opp_ego_gt_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_gt_topic').value
        else:
            self.has_opp = False
            self.start_poses = [[sx, sy, stheta]]
            self._reset(self.start_poses)

        # sim physical step timer
        self.sim_dt = 0.01
        self.drive_timer = self.create_timer(self.sim_dt, self.drive_timer_callback)
        # topic publishing timer (tf, ground truth)
        self.timer = self.create_timer(0.004, self.timer_callback)
        # sensors at their real rates
        self.scan_timer = self.create_timer(1.0 / self.get_parameter('scan_rate').value, self.scan_timer_callback)
        self.odom_timer = self.create_timer(1.0 / self.get_parameter('odom_rate').value, self.odom_timer_callback)

        # When the sim state was computed: the stamp of the messages about it (ground truth, /odom, /scan).
        # Stamped when published instead, they would be up to a sim step (10 ms) old at their stamp, by a
        # varying amount: at speed the pose would jitter by up to 0.17 m against its stamp.
        self.state_stamp = self.get_clock().now()
        self.last_stamps = {}

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        if self.localization == 'ground_truth':
            self.static_br = StaticTransformBroadcaster(self)
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = 'map'
            map_to_odom.child_frame_id = self.odom_frame
            map_to_odom.transform.rotation.w = 1.0
            self.static_br.sendTransform(map_to_odom)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_gt_pub = self.create_publisher(Odometry, ego_gt_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 10)
        # where the ego car is after a teleport, for the localizer (sim only: on the car, RViz's 2D Pose Estimate)
        localizer_reset_topic = self.get_parameter('localizer_reset_topic').value
        self.localizer_reset_pub = self.create_publisher(PoseWithCovarianceStamped, localizer_reset_topic, 10) \
            if localizer_reset_topic else None
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_gt_pub = self.create_publisher(Odometry, ego_opp_gt_topic, 10)
            self.opp_gt_pub = self.create_publisher(Odometry, opp_gt_topic, 10)
            self.opp_ego_gt_pub = self.create_publisher(Odometry, opp_ego_gt_topic, 10)
            self.opp_drive_published = False

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('ego_reset_topic').value,
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                self.get_parameter('opp_reset_topic').value,
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

        # Log now, before the sim runs: rclpy's first log call in a process takes ~30 ms (it looks up the
        # caller's module among all the loaded ones), and in the middle of a run the sim would miss 3 steps.
        self.get_logger().info(
            f"map {self.get_parameter('map_path').value}, localization {self.localization}, "
            f"reset after 2 laps: {'yes' if self.reset_on_laps else 'no'}")


    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            self._reset([[rx, ry, rtheta], list(self.opp_pose)])
        else:
            self._reset([[rx, ry, rtheta]])
        self._reset_localizer()

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self._reset([list(self.ego_pose), [rx, ry, rtheta]])

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        stepped = False
        if self.ego_drive_published and not self.has_opp:
            self._step(np.array([[self.ego_steer, self.ego_requested_speed]]))
            stepped = True
        elif self.ego_drive_published and self.has_opp and self.opp_drive_published:
            self._step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
            stepped = True
        self._update_sim_state()
        self.state_stamp = self.get_clock().now()
        if stepped:
            self._update_odometry()

    def _measurement_stamp(self, topic, idle=0.1):
        """The stamp for a message about the latest sim state: the time it was computed, once per state.
        Without a new state (the sim stalled) the same data again only after `idle` s, stamped now.
        None: nothing new to publish."""
        last = self.last_stamps.get(topic)
        if last is None or self.state_stamp > last:
            stamp = self.state_stamp
        else:
            now = self.get_clock().now()
            if (now - last).nanoseconds < idle * 1e9:
                return None
            stamp = now
        self.last_stamps[topic] = stamp
        return stamp.to_msg()

    def _update_odometry(self):
        """One sim step of VESC odometry, from what the VESC can measure: wheel speed and servo command."""
        steer = np.clip(self.ego_steer, -0.4189, 0.4189) + self.steer_offset    # gym's s_max, the servo limit
        self.odom_speed = measured_speed(self.ego_speed[0], self.odom_rng, self.speed_scale_error, self.speed_noise)
        self.odom_pose, self.odom_yaw_rate = odom_step(self.odom_pose, self.odom_speed, steer, self.sim_dt, self.wheelbase)

    def odom_timer_callback(self):
        """VESC odometry, as vesc_to_odom publishes it on the car."""
        stamp = self._measurement_stamp('odom')
        if stamp is None:
            return
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.ego_namespace + '/base_link'
        odom.pose.pose.position.x = self.odom_pose[0]
        odom.pose.pose.position.y = self.odom_pose[1]
        quat = euler.euler2quat(0., 0., self.odom_pose[2], axes='sxyz')
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]
        odom.twist.twist.linear.x = self.odom_speed
        odom.twist.twist.angular.z = self.odom_yaw_rate
        self.odom_pub.publish(odom)

    def _step(self, action):
        self.obs, _, terminated, truncated, info = self.env.step(action)
        if terminated or truncated:
            if self.obs['collisions'][0]:
                reason = 'ego collision'
            elif info['rollover'][0]:
                reason = 'ego rollover'
            elif terminated:
                reason = 'laps completed'
            else:
                reason = 'time limit reached'
            if reason == 'laps completed' and not self.reset_on_laps:
                # Keep driving: clear the gym's lap counters (the episode ends when they reach 2 laps)
                # instead of moving the car back to the start. Not logged as 'laps completed', which
                # means a reset to the telemetry recorder.
                self.env.unwrapped.toggle_list[:] = 0
                self.get_logger().info('2 laps done, the car keeps going (reset_on_laps false)', once=True)
                return
            self.get_logger().info(reason + ', resetting to the starting poses', throttle_duration_sec=1.0)
            # stop the cars, they move again with the next drive command
            self.ego_requested_speed = 0.0
            self.ego_steer = 0.0
            if self.has_opp:
                self.opp_requested_speed = 0.0
                self.opp_steer = 0.0
            self._reset(self.start_poses)
            self._reset_localizer()

    def _reset(self, poses):
        self.obs, _ = self.env.reset(options={'poses': np.array(poses)})
        self._update_sim_state()

    def _reset_localizer(self):
        """The ego car was teleported: give the localizer its new pose, as one would with RViz's 2D Pose
        Estimate after putting the real car down somewhere else. Otherwise it keeps the car where it was."""
        if self.localizer_reset_pub is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.ego_pose[0]
        msg.pose.pose.position.y = self.ego_pose[1]
        quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]
        msg.pose.pose.orientation.w = quat[0]
        self.localizer_reset_pub.publish(msg)

    def scan_timer_callback(self):
        ts = self._measurement_stamp('scan')
        if ts is None:
            return

        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = self.scan_range_max
        scan.ranges = noisy_scan(self.ego_scan, self.scan_rng, self.scan_range_std,
                                 self.scan_range_max, self.scan_dropout).tolist()
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = self.scan_range_max
            opp_scan.ranges = noisy_scan(self.opp_scan, self.scan_rng, self.scan_range_std,
                                         self.scan_range_max, self.scan_dropout).tolist()
            self.opp_scan_pub.publish(opp_scan)

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # ground truth: once per sim step (this timer is faster), stamped with the step's time
        gt_stamp = self._measurement_stamp('ground_truth')
        if gt_stamp is not None:
            self._publish_ground_truth(gt_stamp)
        # pub tf
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        # the gym returns float32 numpy values, ROS messages only take python floats
        self.ego_scan = self.obs['scans'][0].tolist()
        if self.has_opp:
            self.opp_scan = self.obs['scans'][1].tolist()
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])

        

    def _publish_ground_truth(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_gt_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_gt_pub.publish(opp_odom)
            self.opp_ego_gt_pub.publish(ego_odom)
            self.ego_opp_gt_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        x, y, theta = self._ego_odom_pose()
        ego_t = Transform()
        ego_t.translation.x = x
        ego_t.translation.y = y
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, theta, axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = self.odom_frame
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _ego_odom_pose(self):
        """Pose of base_link in the odom frame."""
        if self.localization == 'ground_truth':
            return self.ego_pose                    # map == odom
        return self.odom_pose                       # VESC dead reckoning: drifts, the localizer corrects it

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()