# scout.inc
# Second robotics project - Stage robot model definition
#
# Robot: AgileX Scout Mini with omnidirectional kinematics.
# This file defines a reusable model included by scout.world.
#
# Key points:
#   - drive "omni" enables omnidirectional motion, not differential drive.
#   - The laser sensor parameters approximate the LaserScan converted from RSLidar point cloud data.
#   - localization "odom" plus odom_error simulates odometry noise.

# -------------------- LiDAR sensor definition --------------------
# Model the RSLidar, after conversion to a 2D LaserScan, as a Stage ranger sensor.
# The 360 degree FOV, sample count, and range approximate the real sensor.
define rslidar ranger
(
  sensor
  (
    # range [minimum maximum], in meters. RSLidar usually has a long range.
    range [ 0.05 30.0 ]
    # Field of view: full 360 degree scan.
    fov 360.0
    # Number of rays per scan. Adjust this to match the actual LaserScan if needed.
    samples 720
  )
  # Sensor appearance, used only for visualization.
  color "black"
  size [ 0.06 0.06 0.05 ]
)

# -------------------- Scout Mini robot body definition --------------------
define scout_mini position
(
  # Initial pose, overridden by the instance in the .world file.
  pose [ 0.0 0.0 0.0 0.0 ]

  # Odometry source: use Stage's built-in odom and add noise to mimic real drift.
  localization "odom"
  # odom_error [x_error y_error z_error theta_error], used to simulate odometry noise.
  # Real /odom is noisy, so this makes the simulation more realistic.
  odom_error [ 0.03 0.03 0.0 0.05 ]

  # Robot size [length width height], in meters, close to the real Scout Mini.
  size [ 0.612 0.580 0.245 ]
  # Rotation center, placed at the geometric center.
  origin [ 0.0 0.0 0.0 0.0 ]

  # Show a nose marker to make the heading easier to see.
  gui_nose 1

  # Omnidirectional kinematics.
  # The course example may use "diff", but Scout Mini requires "omni".
  drive "omni"

  color "blue"

  # Mount the LiDAR near the center of the robot, slightly above the body.
  rslidar( pose [ 0.0 0.0 0.15 0.0 ] )
)
