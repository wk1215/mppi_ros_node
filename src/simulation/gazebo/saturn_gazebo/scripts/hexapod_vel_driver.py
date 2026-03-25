#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hexapod Velocity Driver - Tripod Gait Controller

Converts geometry_msgs/Twist (/cmd_vel) commands into 18 joint position
commands for the Saturn Lite hexapod robot using a tripod gait pattern.

Tripod Gait:
  - Group A (stance when Group B swings): FL, MR, RL
  - Group B (stance when Group A swings): FR, ML, RR
  - Three joints per leg: hip (roll/lateral), thigh (pitch/forward), shank (pitch/knee)

Data Flow:
  /cmd_vel (vx, vy, omega)  -- this node -->  18x /saturn/*_controller/command

Author: Auto-generated for Saturn Lite integration
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class TripodGaitController:
    """
    Tripod gait controller for hexapod robot.

    Converts velocity commands to joint angles using a phase-based
    tripod gait with sinusoidal trajectories.
    """

    # Leg names
    LEG_NAMES = ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']
    JOINT_TYPES = ['hip', 'thigh', 'shank']

    # Tripod gait groupings
    GROUP_A = {'fl', 'mr', 'rl'}   # front-left, mid-right, rear-left
    GROUP_B = {'fr', 'ml', 'rr'}   # front-right, mid-left, rear-right

    # Left-side legs
    LEFT_LEGS = {'fl', 'ml', 'rl'}

    # Joint limits (from URDF)
    HIP_LIMITS = (-0.67, 0.67)
    THIGH_LIMITS = (-0.785, 3.14)
    SHANK_LIMITS = (-2.68, 0.0)

    # Default standing joint positions (from Isaac Lab config)
    DEFAULT_HIP = 0.0
    DEFAULT_THIGH = 0.74
    DEFAULT_SHANK = -1.4

    def __init__(self):
        rospy.init_node('hexapod_vel_driver')

        # === Load parameters ===
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.gait_frequency = rospy.get_param('~gait_frequency', 2.0)     # Hz
        self.stride_length = rospy.get_param('~stride_length', 0.25)      # rad
        self.lift_height = rospy.get_param('~lift_height', 0.25)          # rad
        self.lateral_gain = rospy.get_param('~lateral_gain', 0.15)        # rad/(m/s)
        self.turn_gain = rospy.get_param('~turn_gain', 0.12)             # rad/(rad/s)
        self.max_vx = rospy.get_param('~max_vx', 0.5)                    # m/s
        self.max_vy = rospy.get_param('~max_vy', 0.3)                    # m/s
        self.max_omega = rospy.get_param('~max_omega', 1.0)              # rad/s
        self.velocity_deadzone = rospy.get_param('~velocity_deadzone', 0.02)
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)          # seconds
        self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.8)

        # === State ===
        self.phase = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self.smooth_omega = 0.0
        self.last_cmd_time = rospy.Time.now()

        # Joint state feedback (optional, for RL controller integration)
        self.joint_positions = {}
        self.joint_velocities = {}

        # === Publishers: one per joint ===
        self.joint_pubs = {}
        for leg in self.LEG_NAMES:
            for jtype in self.JOINT_TYPES:
                joint_name = f'{leg}_{jtype}'
                topic = f'/saturn/{joint_name}_controller/command'
                self.joint_pubs[joint_name] = rospy.Publisher(
                    topic, Float64, queue_size=1)

        # === Subscribers ===
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(
            '/saturn/joint_states', JointState, self.joint_state_callback, queue_size=1)

        # === Control loop timer ===
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop)

        rospy.loginfo("[HexapodVelDriver] Initialized - tripod gait controller")
        rospy.loginfo("[HexapodVelDriver] max_vel: vx=%.2f, vy=%.2f, omega=%.2f",
                      self.max_vx, self.max_vy, self.max_omega)
        rospy.loginfo("[HexapodVelDriver] gait_freq=%.1fHz, stride=%.2frad, lift=%.2frad",
                      self.gait_frequency, self.stride_length, self.lift_height)

    def cmd_vel_callback(self, msg):
        """Receive and clamp velocity commands."""
        if math.isnan(msg.linear.x) or math.isnan(msg.linear.y) or math.isnan(msg.angular.z):
            rospy.logwarn("[HexapodVelDriver] Received NaN in cmd_vel. Ignoring.")
            return

        self.cmd_vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        self.cmd_vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        self.cmd_omega = max(-self.max_omega, min(self.max_omega, msg.angular.z))
        self.last_cmd_time = rospy.Time.now()

    def joint_state_callback(self, msg):
        """Cache joint state feedback (for future RL controller integration)."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    @staticmethod
    def clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def smooth_velocity(self):
        """Exponential smoothing for velocity commands."""
        alpha = self.smoothing_factor
        self.smooth_vx = alpha * self.smooth_vx + (1.0 - alpha) * self.cmd_vx
        self.smooth_vy = alpha * self.smooth_vy + (1.0 - alpha) * self.cmd_vy
        self.smooth_omega = alpha * self.smooth_omega + (1.0 - alpha) * self.cmd_omega

    def control_loop(self, event):
        """Main gait control loop at control_rate Hz."""
        dt = 1.0 / self.control_rate

        # Timeout: zero velocity if no recent command
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_omega = 0.0

        # Apply velocity smoothing
        self.smooth_velocity()
        vx = self.smooth_vx
        vy = self.smooth_vy
        omega = self.smooth_omega

        # Compute speed magnitude for gait modulation
        speed = math.sqrt(vx ** 2 + vy ** 2 + (omega * 0.3) ** 2)

        if speed < self.velocity_deadzone:
            # Standing still: publish default pose
            self._publish_standing_pose()
            return

        # Advance gait phase
        # Gait frequency scales with speed for natural motion
        freq_scale = min(speed / (self.max_vx * 0.5), 1.5)
        effective_freq = self.gait_frequency * max(freq_scale, 0.5)
        self.phase += 2.0 * math.pi * effective_freq * dt
        if self.phase > 2.0 * math.pi:
            self.phase -= 2.0 * math.pi

        # Compute and publish joint angles for each leg
        for leg in self.LEG_NAMES:
            hip, thigh, shank = self._compute_leg_angles(
                leg, vx, vy, omega, speed)
            self._publish_joint(leg, 'hip', hip)
            self._publish_joint(leg, 'thigh', thigh)
            self._publish_joint(leg, 'shank', shank)

    def _compute_leg_angles(self, leg, vx, vy, omega, speed):
        """
        Compute joint angles for a single leg based on tripod gait.

        Returns:
            (hip_angle, thigh_angle, shank_angle) in radians
        """
        # Determine gait phase for this leg
        if leg in self.GROUP_A:
            leg_phase = self.phase
        else:
            leg_phase = self.phase + math.pi  # opposite phase

        # Normalized speed [0, 1]
        speed_norm = min(speed / self.max_vx, 1.0)

        # Scale stride and lift by speed
        stride = self.stride_length * speed_norm
        lift = self.lift_height * speed_norm

        # Phase decomposition
        sin_phase = math.sin(leg_phase)
        cos_phase = math.cos(leg_phase)
        in_swing = sin_phase > 0  # swing when sin > 0

        # === Thigh joint: forward/backward swing ===
        # Forward velocity contribution
        fwd_factor = vx / self.max_vx if abs(vx) > 0.01 else 0.0
        fwd_stride = stride * fwd_factor

        # Turning contribution: differential stride between left and right
        is_left = leg in self.LEFT_LEGS
        turn_stride = self.turn_gain * omega * (1.0 if is_left else -1.0)

        # Thigh angle: sinusoidal swing + default position
        thigh_angle = self.DEFAULT_THIGH - (fwd_stride + turn_stride) * cos_phase

        # === Shank joint: ground clearance during swing ===
        if in_swing:
            # Lift foot during swing phase
            swing_height = lift * sin_phase
            shank_angle = self.DEFAULT_SHANK + swing_height
        else:
            # Stance phase: maintain ground contact
            shank_angle = self.DEFAULT_SHANK

        # === Hip joint: lateral movement + stabilization ===
        lat_offset = self.lateral_gain * vy
        hip_sign = 1.0 if is_left else -1.0
        hip_angle = self.DEFAULT_HIP + lat_offset * hip_sign

        # Apply joint limits
        hip_angle = self.clamp(hip_angle, *self.HIP_LIMITS)
        thigh_angle = self.clamp(thigh_angle, *self.THIGH_LIMITS)
        shank_angle = self.clamp(shank_angle, *self.SHANK_LIMITS)

        return hip_angle, thigh_angle, shank_angle

    def _publish_standing_pose(self):
        """Publish the default standing pose for all joints."""
        for leg in self.LEG_NAMES:
            self._publish_joint(leg, 'hip', self.DEFAULT_HIP)
            self._publish_joint(leg, 'thigh', self.DEFAULT_THIGH)
            self._publish_joint(leg, 'shank', self.DEFAULT_SHANK)

    def _publish_joint(self, leg, joint_type, angle):
        """Publish a single joint command."""
        key = f'{leg}_{joint_type}'
        msg = Float64()
        msg.data = angle
        self.joint_pubs[key].publish(msg)


def main():
    try:
        controller = TripodGaitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
