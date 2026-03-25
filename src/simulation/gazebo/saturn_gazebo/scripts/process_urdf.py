#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Process Saturn Lite URDF for Gazebo simulation.

Reads the original SolidWorks-exported URDF and adds:
1. Fixed mesh paths (package:// URIs)
2. base_link frame (for navigation stack compatibility)
3. Laser scanner link + Gazebo plugin
4. Transmissions for all 18 revolute joints (gazebo_ros_control)
5. Gazebo ros_control plugin
6. Ground truth odometry (p3d plugin)
7. IMU sensor
8. Foot contact friction properties

Usage:
    python3 process_urdf.py <input_urdf_path> [mesh_package_prefix]

    mesh_package_prefix: defaults to "package://saturn_gazebo"
    Output: processed URDF written to stdout
"""

import sys
import os


def generate_gazebo_additions():
    """Generate all Gazebo-specific URDF additions."""

    # All 18 revolute joint names
    joints = [
        'fl_hip_joint', 'fl_thigh_joint', 'fl_shank_joint',
        'fr_hip_joint', 'fr_thigh_joint', 'fr_shank_joint',
        'ml_hip_joint', 'ml_thigh_joint', 'ml_shank_joint',
        'mr_hip_joint', 'mr_thigh_joint', 'mr_shank_joint',
        'rl_hip_joint', 'rl_thigh_joint', 'rl_shank_joint',
        'rr_hip_joint', 'rr_thigh_joint', 'rr_shank_joint',
    ]

    leg_prefixes = ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']

    parts = []

    # ============================================================
    # 1. base_link (required by move_base / amcl / nav stack)
    # ============================================================
    parts.append("""
  <!-- ===== base_link for navigation stack ===== -->
  <link name="base_link"/>
  <joint name="base_fixed_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="base_body"/>
  </joint>
""")

    # ============================================================
    # 2. 2D Lidar link + joint (mounted on front of body)
    # ============================================================
    parts.append("""
  <!-- ===== 2D Lidar ===== -->
  <link name="laser_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" iyy="0.00001" izz="0.00001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.025"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="laser_joint" type="fixed">
    <origin xyz="0.35 0.0 0.08" rpy="0 0 0"/>
    <parent link="base_body"/>
    <child link="laser_link"/>
  </joint>
""")

    # ============================================================
    # 3. Transmissions for all 18 revolute joints
    # ============================================================
    parts.append("  <!-- ===== Transmissions (18 joints) ===== -->")
    for jname in joints:
        parts.append(f"""
  <transmission name="{jname}_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="{jname}">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="{jname}_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>""")

    # ============================================================
    # 4. Gazebo ros_control plugin
    # ============================================================
    parts.append("""
  <!-- ===== Gazebo ros_control plugin ===== -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/saturn</robotNamespace>
    </plugin>
  </gazebo>
""")

    # ============================================================
    # 5. Gazebo 2D Lidar sensor
    # ============================================================
    parts.append("""
  <!-- ===== Gazebo Lidar sensor ===== -->
  <gazebo reference="laser_link">
    <material>Gazebo/Blue</material>
    <sensor type="ray" name="laser">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>800</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.05</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_laser.so">
        <topicName>/laser_link/scan</topicName>
        <frameName>laser_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
""")

    # ============================================================
    # 6. Ground truth odometry (p3d plugin)
    # ============================================================
    parts.append("""
  <!-- ===== Ground truth odometry (p3d) ===== -->
  <gazebo>
    <plugin name="groundtruth" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>50.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>groundtruth_odom</topicName>
      <frameName>map</frameName>
      <xyzOffsets>0 0 0</xyzOffsets>
      <rpyOffsets>0 0 0</rpyOffsets>
      <gaussianNoise>0.02</gaussianNoise>
    </plugin>
  </gazebo>
""")

    # ============================================================
    # 7. IMU sensor
    # ============================================================
    parts.append("""
  <!-- ===== IMU sensor ===== -->
  <gazebo reference="base_body">
    <gravity>true</gravity>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>200</update_rate>
      <visualize>false</visualize>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>/saturn/imu</topicName>
        <bodyName>base_body</bodyName>
        <updateRateHZ>200.0</updateRateHZ>
        <gaussianNoise>0.001</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>base_body</frameName>
      </plugin>
    </sensor>
  </gazebo>
""")

    # ============================================================
    # 8. Foot contact friction properties
    # ============================================================
    parts.append("  <!-- ===== Foot contact properties ===== -->")
    for prefix in leg_prefixes:
        parts.append(f"""
  <gazebo reference="{prefix}_foot_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000</kp>
    <kd>100</kd>
    <material>Gazebo/DarkGrey</material>
  </gazebo>""")

    # ============================================================
    # 9. Body and leg link Gazebo material settings
    # ============================================================
    parts.append("""
  <!-- ===== Body material ===== -->
  <gazebo reference="base_body">
    <material>Gazebo/Grey</material>
  </gazebo>
""")
    for prefix in leg_prefixes:
        parts.append(f"""  <gazebo reference="{prefix}_hip_link">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>
  <gazebo reference="{prefix}_thigh_link">
    <material>Gazebo/White</material>
  </gazebo>
  <gazebo reference="{prefix}_shank_link">
    <material>Gazebo/LightGrey</material>
  </gazebo>
""")

    return '\n'.join(parts)


def process_urdf(urdf_path, mesh_prefix):
    """
    Process the Saturn Lite URDF file.
    
    Args:
        urdf_path: Path to the original URDF file
        mesh_prefix: Package prefix for mesh files (e.g., "package://saturn_gazebo")
    
    Returns:
        Processed URDF string
    """
    with open(urdf_path, 'r') as f:
        urdf = f.read()

    # Fix mesh paths: replace relative "../meshes/" with package:// URI
    urdf = urdf.replace('../meshes/', f'{mesh_prefix}/meshes/')

    # Generate and insert Gazebo additions before closing </robot> tag
    additions = generate_gazebo_additions()
    urdf = urdf.replace('</robot>', additions + '\n</robot>')

    return urdf


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: process_urdf.py <urdf_path> [mesh_package_prefix]",
              file=sys.stderr)
        print("  mesh_package_prefix: defaults to 'package://saturn_gazebo'",
              file=sys.stderr)
        sys.exit(1)

    urdf_path = sys.argv[1]
    mesh_prefix = sys.argv[2] if len(sys.argv) > 2 else 'package://saturn_gazebo'

    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    result = process_urdf(urdf_path, mesh_prefix)
    print(result)
