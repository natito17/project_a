# =============================================================================
# reactive_node.py  —  Follow the Gap (FTG) obstacle avoidance
#
# WHAT IS THIS FILE?
#   This is a ROS2 Python *node*. A "node" is the ROS2 word for a program that
#   participates in the ROS2 communication network. It receives sensor data,
#   does computation, and sends commands — all through "topics".
#
# HOW DOES IT FIT IN THE SIMULATOR?
#   The f1tenth_gym_ros simulator (GitHub: f1tenth/f1tenth_gym_ros) is a ROS2
#   package that wraps a Python physics simulation. It publishes the LiDAR
#   sensor as a ROS2 message on the "/scan" topic, and it listens for driving
#   commands on the "/drive" topic. This node sits in the middle:
#
#       Simulator  →  /scan (LaserScan)  →  [this node]
#       [this node] →  /drive (AckermannDriveStamped)  →  Simulator
#
# HOW DO WE KNOW THESE ARE THE CORRECT ROS2 APIS?
#   Every import and function call here comes directly from the official ROS2
#   documentation (docs.ros.org) and the f1tenth_gym_ros source code:
#     - rclpy: the official ROS2 Python client library (docs.ros.org/en/humble/p/rclpy)
#     - sensor_msgs/LaserScan: standard ROS2 laser scanner message
#       (docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
#     - ackermann_msgs/AckermannDriveStamped: standard message for car-like
#       (Ackermann-steered) robots (wiki.ros.org/ackermann_msgs)
#     - Topic names /scan and /drive: read directly from
#       f1tenth_gym_ros/config/sim.yaml in the simulator source code
#
# WHERE DO THE PARAMETER VALUES COME FROM?
#   See the "Parameters and Tuning Guide" section of NOTES.md. Short version:
#     - BUBBLE_RADIUS: derived from measured car width (0.31m → half = 0.155m)
#       plus a 4.5cm safety margin → 0.20m
#     - MAX_RANGE_CLIP: F1TENTH lecture recommendation; 3m ≈ 0.75s lookahead at 4m/s
#     - Speed values: conservative starting point for simulation;
#       F1TENTH course convention is to start slow and tune up
#     - Thresholds: chosen so the car has roughly one car-length of reaction
#       distance at each speed tier
# =============================================================================

# ── Imports ────────────────────────────────────────────────────────────────────

# rclpy is the ROS2 Python client library. It must be imported before any
# ROS2 activity can happen. It provides:
#   - rclpy.init()        : start the ROS2 runtime
#   - rclpy.spin()        : enter the event loop (keep the node alive, process
#                           incoming messages)
#   - rclpy.shutdown()    : cleanly stop the ROS2 runtime
import rclpy

# Node is the base class for every ROS2 Python node. Our class inherits from it
# to get the ability to create subscribers, publishers, timers, and loggers.
# Source: docs.ros.org/en/humble/p/rclpy/source/rclpy/node.html
from rclpy.node import Node

# NumPy for fast array math. The LiDAR gives us ~1080 float values per scan;
# using Python lists with loops would be too slow at 40Hz. NumPy operates on
# entire arrays in C, which is fast enough.
import numpy as np

# LaserScan is the standard ROS2 message for any laser range scanner.
# It is defined in the 'sensor_msgs' package, which ships with every ROS2
# installation (it is a "common interface" package, not simulator-specific).
#
# Relevant fields we use:
#   msg.ranges          – tuple of ~1080 floats; each is the measured distance
#                         (in metres) of one LiDAR beam. Index 0 = rightmost
#                         beam, index ~540 = straight ahead, index ~1079 =
#                         leftmost beam.
#   msg.angle_min       – angle (radians) of the first beam (index 0).
#                         Typically ≈ -2.35 rad (≈ -135°), i.e. hard right.
#   msg.angle_max       – angle of the last beam. Typically ≈ +2.35 rad.
#   msg.angle_increment – angle between consecutive beams (≈ 0.00436 rad).
#                         Used to convert a beam index to an actual angle.
#   msg.range_min/max   – hardware limits. Values outside these are invalid.
#
# Source: docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
from sensor_msgs.msg import LaserScan

# AckermannDriveStamped is the standard ROS2 message for Ackermann-steered
# vehicles (i.e. car-like vehicles where the front wheels turn, unlike
# differential-drive robots that spin in place).
#
# The "Stamped" suffix means the message includes a Header (timestamp + frame
# id). The simulator expects this stamped version on /drive.
#
# Relevant fields we use:
#   msg.drive.speed           – desired forward speed in m/s. Positive = forward.
#   msg.drive.steering_angle  – desired steering angle in radians.
#                               Positive = left (towards +Y in the car frame).
#                               The hardware clamps this to roughly ±0.4189 rad
#                               (±24°) — the physical steering limit of the car.
#
# Source: wiki.ros.org/ackermann_msgs
from ackermann_msgs.msg import AckermannDriveStamped


# ── Tunable global parameters ──────────────────────────────────────────────────
#
# These are module-level constants (ALL_CAPS by Python convention).
# They are intentionally at the top of the file so a tuner can find and change
# them without reading the algorithm code. See NOTES.md for full tuning guide.
#
# WHY ARE THEY GLOBAL INSTEAD OF CLASS ATTRIBUTES?
#   Pure stylistic choice. Either works. Globals here means they are easy to
#   spot and edit; they never change at runtime.

# BUBBLE_RADIUS — physical safety bubble radius in metres.
#
# Background: the car body is ~0.31m wide. Half that is 0.155m. If we only
# zero out LiDAR rays within 0.155m of the closest point, the car BODY would
# fit through the gap but its WHEELS would clip the obstacle. We add a 4.5cm
# safety margin: 0.155 + 0.045 = 0.20m.
#
# How it is used: given the closest obstacle at distance d, we zero out all
# beam indices within arc-length BUBBLE_RADIUS of the closest beam. The
# arc-length formula gives: half_index_count = BUBBLE_RADIUS / (d × Δθ)
#   where Δθ = angle_increment per beam.
#
# Tune: increase if the car clips walls; decrease if it stops before gaps.
BUBBLE_RADIUS    = 0.20   # [m]

# MAX_RANGE_CLIP — all preprocessed ranges are clipped to this value.
#
# The raw LiDAR can report up to ~30m (or inf). Gaps far away are irrelevant
# when the car is doing 3m/s — it only needs to see the next ~3m of track.
# Clipping also normalises values so the bubble math is not dominated by
# distant open-air readings.
#
# Tune: raise for faster / smoother behaviour in long straights; lower if the
# car reacts too late to corners.
MAX_RANGE_CLIP   = 3.0    # [m]

# SMOOTH_WINDOW — kernel size for the windowed-mean noise filter.
#
# Real (and simulated) LiDAR has occasional "noise spikes" — single beams that
# read a much smaller or larger value than their neighbours due to sensor noise,
# reflective surfaces, or beam divergence. A windowed mean replaces each value
# with the average of its SMOOTH_WINDOW neighbours, smoothing out spikes.
#
# Must be odd so there is a true centre sample.
# Higher = more smoothing but slightly delayed response.
SMOOTH_WINDOW    = 3      # number of beams

# Speed constants (m/s). The simulator and real car both use the same unit.
# These are NOT automatically safe on the real car — start at ≤1.0 m/s
# on physical hardware until the algorithm is validated.
FAST_SPEED       = 3.0    # [m/s] — open track, nearest obstacle is far
MID_SPEED        = 1.5    # [m/s] — obstacle moderately close
SLOW_SPEED       = 0.5    # [m/s] — obstacle very close (near-crawl)

# Distance thresholds that select which speed tier to use.
# The speed decision is based on the minimum non-zero range in the scan after
# the safety bubble has been applied.
FAST_THRESHOLD   = 1.5    # [m] — if min_range > this, use FAST_SPEED
MID_THRESHOLD    = 0.5    # [m] — elif min_range > this, use MID_SPEED
#                                  else use SLOW_SPEED

# Restrict planning to a forward cone (degrees). This avoids choosing targets
# from side/rear beams in wide-open but non-drivable directions.
FORWARD_FOV_DEG = 160.0   # [deg] e.g., ±80° around straight ahead

# Steering-aware speed scaling parameters.
STEERING_LIMIT = 0.4189         # [rad] ≈ 24°, F1TENTH steering limit
STEER_SLOWDOWN_START = 0.1745   # [rad] ≈ 10°, start reducing speed
STEER_SLOWDOWN_END = 0.4189     # [rad] ≈ 24°, max slowdown at this angle
MIN_STEER_SPEED_SCALE = 0.35    # [ratio] speed floor for sharp steering

# Turn-commitment hysteresis at corner entry.
TURN_COMMIT_FRONT_MIN_THRESHOLD = 1.0  # [m] front blockage trigger
TURN_COMMIT_FRONT_SECTOR_DEG = 30.0    # [deg] min range sector around 0 rad
TURN_COMMIT_SIDE_SECTOR_DEG = 70.0     # [deg] left/right score sectors from center
TURN_COMMIT_HOLD_SCANS = 8             # [scans] keep chosen side for N scans
TURN_COMMIT_DEBUG = True               # emit lightweight commit debug logs
TURN_COMMIT_LOG_STRIDE = 8             # log every N scans while active/blocked

# Stage-2 disparity-aware masking before max-gap search.
# Detect sharp beam-to-beam jumps and extend obstacle influence by half-car width
# into the farther side so narrow pseudo-gaps are not treated as drivable.
DISPARITY_ENABLE = True
DISPARITY_THRESHOLD = 0.35             # [m] minimum adjacent-beam jump to trigger
DISPARITY_EXTEND_WIDTH = 0.16          # [m] half-width to project obstacle influence
DISPARITY_MAX_EXTENSION_BEAMS = 120    # [beams] cap for very close obstacles
DISPARITY_FRONT_MAX_RANGE = 0.95       # [m] only apply when front is this constrained
DISPARITY_MIN_SIDE_CLEARANCE = 0.95    # [m] require at least one side with this clearance
DISPARITY_DEBUG = False                # optional lightweight disparity logs
DISPARITY_LOG_STRIDE = 20              # log every N scans when disparities found
# ───────────────────────────────────────────────────────────────────────────────


# =============================================================================
# ReactiveFollowGap  —  the ROS2 node class
# =============================================================================

class ReactiveFollowGap(Node):
    """
    A ROS2 node that implements the Follow the Gap (FTG) reactive obstacle-
    avoidance algorithm for the RoboRacer / F1TENTH platform.

    WHAT IS A ROS2 NODE?
    A node is a single executable process in the ROS2 ecosystem. Nodes
    communicate by publishing and subscribing to "topics". A topic is a named
    channel that carries a specific message type. Multiple nodes can publish
    to or subscribe from the same topic — this is how sensor data flows from
    the hardware driver (or simulator) to algorithm nodes to the actuator
    driver.

    WHY INHERIT FROM Node?
    rclpy.node.Node provides all the ROS2 plumbing:
      - self.create_subscription(...) — register a callback for incoming msgs
      - self.create_publisher(...)    — get a handle to publish messages
      - self.get_logger()             — ROS2-aware logging (visible in ros2 log)
      - self.create_timer(...)        — periodic callbacks (not used here)
    Without inheriting from Node, we would have no way to connect to the ROS2
    network.

    ALGORITHM OVERVIEW (Follow the Gap):
      1. Subscribe to /scan → receive LaserScan ~40 times per second
      2. preprocess_lidar():
           a. Replace inf/nan → clip to MAX_RANGE_CLIP → smooth with window mean
      3. Find the closest obstacle; draw a "safety bubble" of BUBBLE_RADIUS
         around it by zeroing all beam indices within that arc
        4. Apply disparity-aware masking to close non-reachable pseudo-gaps
        5. find_max_gap(): scan the zeroed array for the longest consecutive run
         of non-zero values → that is the "biggest free corridor"
        6. find_best_point(): take the CENTER of that corridor as the aim point
        7. Convert aim-point index → steering angle using angle_min + idx × Δθ
        8. Choose speed based on nearest obstacle distance
        9. Publish AckermannDriveStamped to /drive
    """

    def __init__(self):
        # ── super().__init__('reactive_node') ─────────────────────────────────
        # This MUST be the first call in __init__. It calls the constructor of
        # rclpy.node.Node with the node name 'reactive_node'.
        #
        # The node name is what appears in:
        #   ros2 node list          (shows all running nodes)
        #   ros2 node info <name>   (shows topics this node uses)
        #
        # It must be unique within the ROS2 graph (you cannot run two nodes
        # with the same name simultaneously — the second one would crash).
        super().__init__('reactive_node')

        # ── Topic names ────────────────────────────────────────────────────────
        # These strings are the ROS2 "topic names". A topic name is like a
        # phone number — publishers and subscribers must use the SAME string
        # to talk to each other. These specific names come from the simulator's
        # configuration file: f1tenth_gym_ros/config/sim.yaml.
        #
        # /scan  → the simulator publishes LaserScan data here
        # /drive → the simulator subscribes to drive commands here
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        lidarscan_topic = str(self.get_parameter('scan_topic').value)
        drive_topic = str(self.get_parameter('drive_topic').value)

        # ── FTG tuning parameters (ROS2 params) ─────────────────────────────
        # These default to the module-level constants so existing behavior
        # remains unchanged unless explicitly overridden.
        self.declare_parameter('bubble_radius', BUBBLE_RADIUS)
        self.declare_parameter('max_range_clip', MAX_RANGE_CLIP)
        self.declare_parameter('smooth_window', SMOOTH_WINDOW)
        self.declare_parameter('fast_speed', FAST_SPEED)
        self.declare_parameter('mid_speed', MID_SPEED)
        self.declare_parameter('slow_speed', SLOW_SPEED)
        self.declare_parameter('fast_threshold', FAST_THRESHOLD)
        self.declare_parameter('mid_threshold', MID_THRESHOLD)
        self.declare_parameter('forward_fov_deg', FORWARD_FOV_DEG)
        self.declare_parameter('steering_limit', STEERING_LIMIT)
        self.declare_parameter('steer_slowdown_start', STEER_SLOWDOWN_START)
        self.declare_parameter('steer_slowdown_end', STEER_SLOWDOWN_END)
        self.declare_parameter('min_steer_speed_scale', MIN_STEER_SPEED_SCALE)
        self.declare_parameter('turn_commit_front_min_threshold', TURN_COMMIT_FRONT_MIN_THRESHOLD)
        self.declare_parameter('turn_commit_front_sector_deg', TURN_COMMIT_FRONT_SECTOR_DEG)
        self.declare_parameter('turn_commit_side_sector_deg', TURN_COMMIT_SIDE_SECTOR_DEG)
        self.declare_parameter('turn_commit_hold_scans', TURN_COMMIT_HOLD_SCANS)
        self.declare_parameter('turn_commit_debug', TURN_COMMIT_DEBUG)
        self.declare_parameter('turn_commit_log_stride', TURN_COMMIT_LOG_STRIDE)
        self.declare_parameter('disparity_enable', DISPARITY_ENABLE)
        self.declare_parameter('disparity_threshold', DISPARITY_THRESHOLD)
        self.declare_parameter('disparity_extend_width', DISPARITY_EXTEND_WIDTH)
        self.declare_parameter('disparity_max_extension_beams', DISPARITY_MAX_EXTENSION_BEAMS)
        self.declare_parameter('disparity_front_max_range', DISPARITY_FRONT_MAX_RANGE)
        self.declare_parameter('disparity_min_side_clearance', DISPARITY_MIN_SIDE_CLEARANCE)
        self.declare_parameter('disparity_debug', DISPARITY_DEBUG)
        self.declare_parameter('disparity_log_stride', DISPARITY_LOG_STRIDE)

        self.bubble_radius = float(self.get_parameter('bubble_radius').value)
        self.max_range_clip = float(self.get_parameter('max_range_clip').value)
        self.smooth_window = int(self.get_parameter('smooth_window').value)
        self.fast_speed = float(self.get_parameter('fast_speed').value)
        self.mid_speed = float(self.get_parameter('mid_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.fast_threshold = float(self.get_parameter('fast_threshold').value)
        self.mid_threshold = float(self.get_parameter('mid_threshold').value)
        self.forward_fov_deg = float(self.get_parameter('forward_fov_deg').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.steer_slowdown_start = float(self.get_parameter('steer_slowdown_start').value)
        self.steer_slowdown_end = float(self.get_parameter('steer_slowdown_end').value)
        self.min_steer_speed_scale = float(self.get_parameter('min_steer_speed_scale').value)
        self.turn_commit_front_min_threshold = float(
            self.get_parameter('turn_commit_front_min_threshold').value
        )
        self.turn_commit_front_sector_deg = float(
            self.get_parameter('turn_commit_front_sector_deg').value
        )
        self.turn_commit_side_sector_deg = float(
            self.get_parameter('turn_commit_side_sector_deg').value
        )
        self.turn_commit_hold_scans = int(self.get_parameter('turn_commit_hold_scans').value)
        self.turn_commit_debug = bool(self.get_parameter('turn_commit_debug').value)
        self.turn_commit_log_stride = int(self.get_parameter('turn_commit_log_stride').value)
        self.disparity_enable = bool(self.get_parameter('disparity_enable').value)
        self.disparity_threshold = float(self.get_parameter('disparity_threshold').value)
        self.disparity_extend_width = float(self.get_parameter('disparity_extend_width').value)
        self.disparity_max_extension_beams = int(
            self.get_parameter('disparity_max_extension_beams').value
        )
        self.disparity_front_max_range = float(
            self.get_parameter('disparity_front_max_range').value
        )
        self.disparity_min_side_clearance = float(
            self.get_parameter('disparity_min_side_clearance').value
        )
        self.disparity_debug = bool(self.get_parameter('disparity_debug').value)
        self.disparity_log_stride = int(self.get_parameter('disparity_log_stride').value)

        # Keep the filter valid for all configs.
        if self.smooth_window < 1:
            self.smooth_window = 1
            self.get_logger().warn('smooth_window < 1; using 1 instead.')
        if self.smooth_window % 2 == 0:
            self.smooth_window += 1
            self.get_logger().warn(
                f'smooth_window must be odd; using {self.smooth_window}.'
            )
        if self.forward_fov_deg <= 0.0:
            self.forward_fov_deg = FORWARD_FOV_DEG
            self.get_logger().warn(
                f'forward_fov_deg <= 0; using {self.forward_fov_deg} degrees.'
            )
        self.forward_fov_deg = min(self.forward_fov_deg, 270.0)
        if self.steering_limit <= 0.0:
            self.steering_limit = STEERING_LIMIT
            self.get_logger().warn(
                f'steering_limit <= 0; using {self.steering_limit} rad.'
            )
        if self.steer_slowdown_end < self.steer_slowdown_start:
            self.steer_slowdown_end = self.steer_slowdown_start
            self.get_logger().warn(
                'steer_slowdown_end < steer_slowdown_start; clamping to start value.'
            )
        self.min_steer_speed_scale = float(np.clip(self.min_steer_speed_scale, 0.0, 1.0))
        if self.turn_commit_front_sector_deg <= 0.0:
            self.turn_commit_front_sector_deg = TURN_COMMIT_FRONT_SECTOR_DEG
            self.get_logger().warn(
                f'turn_commit_front_sector_deg <= 0; using {self.turn_commit_front_sector_deg}.'
            )
        if self.turn_commit_side_sector_deg <= 0.0:
            self.turn_commit_side_sector_deg = TURN_COMMIT_SIDE_SECTOR_DEG
            self.get_logger().warn(
                f'turn_commit_side_sector_deg <= 0; using {self.turn_commit_side_sector_deg}.'
            )
        if self.turn_commit_hold_scans < 0:
            self.turn_commit_hold_scans = 0
            self.get_logger().warn('turn_commit_hold_scans < 0; using 0.')
        if self.turn_commit_log_stride < 1:
            self.turn_commit_log_stride = 1
            self.get_logger().warn('turn_commit_log_stride < 1; using 1.')
        if self.disparity_threshold <= 0.0:
            self.disparity_threshold = DISPARITY_THRESHOLD
            self.get_logger().warn(
                f'disparity_threshold <= 0; using {self.disparity_threshold}.'
            )
        if self.disparity_extend_width <= 0.0:
            self.disparity_extend_width = DISPARITY_EXTEND_WIDTH
            self.get_logger().warn(
                f'disparity_extend_width <= 0; using {self.disparity_extend_width}.'
            )
        if self.disparity_max_extension_beams < 1:
            self.disparity_max_extension_beams = 1
            self.get_logger().warn('disparity_max_extension_beams < 1; using 1.')
        if self.disparity_front_max_range <= 0.0:
            self.disparity_front_max_range = DISPARITY_FRONT_MAX_RANGE
            self.get_logger().warn(
                f'disparity_front_max_range <= 0; using {self.disparity_front_max_range}.'
            )
        if self.disparity_min_side_clearance <= 0.0:
            self.disparity_min_side_clearance = DISPARITY_MIN_SIDE_CLEARANCE
            self.get_logger().warn(
                f'disparity_min_side_clearance <= 0; using {self.disparity_min_side_clearance}.'
            )
        if self.disparity_log_stride < 1:
            self.disparity_log_stride = 1
            self.get_logger().warn('disparity_log_stride < 1; using 1.')

        # Hysteresis state for turn commitment.
        self.turn_commit_side = 0         # -1 = right, +1 = left, 0 = none
        self.turn_commit_counter = 0      # scans remaining for current commitment
        self.scan_counter = 0             # used for throttled debug output

        # ── self.create_subscription(...) ─────────────────────────────────────
        # Registers this node as a *subscriber* to a topic. Whenever a new
        # message arrives on that topic, ROS2 automatically calls our callback
        # function.
        #
        # Arguments:
        #   1. LaserScan         — the MESSAGE TYPE we expect on this topic.
        #                          ROS2 uses this to deserialise the binary data
        #                          on the wire into a Python object with fields
        #                          like .ranges, .angle_min, etc.
        #                          Must exactly match what the publisher sends.
        #
        #   2. lidarscan_topic   — the TOPIC NAME to subscribe to ('/scan').
        #
        #   3. self.lidar_callback  — the CALLBACK FUNCTION. Every time a new
        #                          LaserScan message arrives, ROS2 calls
        #                          lidar_callback(msg) with the message as the
        #                          argument. This is where our algorithm runs.
        #
        #   4. 10                — the QoS (Quality of Service) DEPTH, also
        #                          called the "history queue size". ROS2 stores
        #                          up to 10 unprocessed messages in an internal
        #                          queue. If our callback is slower than the
        #                          publish rate, old messages are dropped once
        #                          the queue is full. 10 is the recommended
        #                          default for sensor streams.
        #                          Source: docs.ros.org/en/humble/Concepts/
        #                                  About-Quality-of-Service-Settings.html
        #
        # The return value is stored in self.subscription. This is REQUIRED.
        # If we did not store it, Python's garbage collector would destroy the
        # subscription object and we would stop receiving messages.
        self.subscription = self.create_subscription(
            LaserScan,           # message type
            lidarscan_topic,     # topic name
            self.lidar_callback, # callback function
            10                   # QoS queue depth
        )

        # ── self.create_publisher(...) ─────────────────────────────────────────
        # Registers this node as a *publisher* on a topic. Returns a publisher
        # object we use later to send messages.
        #
        # Arguments:
        #   1. AckermannDriveStamped — the MESSAGE TYPE we will publish.
        #                              The simulator expects exactly this type
        #                              on /drive.
        #
        #   2. drive_topic           — the TOPIC NAME ('/drive').
        #
        #   3. 10                    — QoS depth (same concept as above, but
        #                              this is the outgoing queue). 10 is fine
        #                              for drive commands.
        #
        # We store this as self.publisher so we can call
        # self.publisher.publish(msg) from inside lidar_callback.
        self.publisher = self.create_publisher(
            AckermannDriveStamped, # message type
            drive_topic,           # topic name
            10                     # QoS queue depth
        )

        # ── self.get_logger().info(...) ────────────────────────────────────────
        # get_logger() returns the node's ROS2 logger. .info() prints at INFO
        # severity level. This appears in the terminal where you launched the
        # node AND in the ROS2 log system (viewable with `ros2 log`).
        # Other severity levels: .debug(), .warn(), .error(), .fatal()
        self.get_logger().info('ReactiveFollowGap node started.')


    # ==========================================================================
    # preprocess_lidar — Step 1: clean the raw sensor data
    # ==========================================================================

    def preprocess_lidar(self, ranges):
        """
        Convert raw LiDAR readings into a clean, noise-filtered array.

        WHY DO WE NEED TO PREPROCESS?
        The raw LaserScan.ranges tuple has several problems:
          1. inf values: a beam with no return within range_max is reported as
             float('inf'). NumPy math on inf gives inf or nan, which breaks
             argmin, clip, and the bubble formula.
          2. nan values: some LiDAR drivers report invalid beams as NaN instead
             of inf. Same problem.
          3. Noise spikes: individual beams can read abnormally high or low due
             to reflective surfaces or beam divergence. A single false-low spike
             would incorrectly trigger a huge safety bubble.
          4. Far readings: a beam that sees 30m of empty space is not useful
             for immediate obstacle avoidance. Clipping focuses attention.

        Args:
            ranges: the raw LaserScan.ranges field — a tuple of ~1080 floats,
                    each representing a measured distance in metres.

        Returns:
            proc_ranges: np.ndarray of shape (N,), dtype float64.
                         Same length as the input. All values are finite,
                         in [0, MAX_RANGE_CLIP], and noise-smoothed.
        """

        # ── Step 1a: Convert to NumPy float64 array ────────────────────────────
        # LaserScan.ranges arrives as a Python tuple of float32 values (or
        # sometimes a Python list). We convert to float64 for numerical
        # stability — float32 has only ~7 significant digits, which is marginal
        # for the bubble-radius arc-length formula.
        proc_ranges = np.array(ranges, dtype=np.float64)

        # ── Step 1b: Replace inf and nan with MAX_RANGE_CLIP ──────────────────
        # np.isfinite() returns True for normal numbers and False for inf/nan.
        # np.where(condition, value_if_true, value_if_false) acts element-wise:
        #   - If the value is finite, keep it unchanged.
        #   - If the value is inf or nan, replace it with MAX_RANGE_CLIP.
        #
        # We treat "no return" (inf) as "the maximum safe distance" — a
        # reasonable approximation since it means the path is clear as far as
        # the LiDAR can see.
        proc_ranges = np.where(np.isfinite(proc_ranges), proc_ranges, self.max_range_clip)

        # ── Step 1c: Clip to [0, MAX_RANGE_CLIP] ──────────────────────────────
        # np.clip(array, min_val, max_val) clamps every element.
        # This handles any remaining out-of-range values (e.g. negative ranges
        # from a mis-calibrated sensor) and enforces the upper bound we want.
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range_clip)

        # ── Step 1d: Windowed mean smoothing ──────────────────────────────────
        # We replace each sample with the average of its SMOOTH_WINDOW nearest
        # neighbours (including itself). This is called a "box filter" or
        # "moving average" and is the simplest noise-reduction technique.
        #
        # Example with SMOOTH_WINDOW=3 and array [1, 5, 1, 1, 1]:
        #   index 1 (spike at 5) → (1 + 5 + 1) / 3 = 2.33  (spike reduced)
        #
        # IMPLEMENTATION DETAIL — why not np.convolve(mode='same')?
        #   np.convolve with mode='same' uses ZERO-PADDING at the array edges.
        #   Zero-padding means the first and last few samples are averaged with
        #   imaginary "0" neighbours, making them appear artificially small.
        #   In the bubble formula: bubble_half = BUBBLE_RADIUS / (d × Δθ)
        #   If d is falsely small (e.g. 0.001 instead of 2.0), bubble_half
        #   becomes enormous and zeros out the ENTIRE array → the car stops.
        #   Fix: use np.pad(mode='edge') which repeats the first/last real
        #   sample instead of inserting zeros.
        #
        # kernel = [1/3, 1/3, 1/3] for SMOOTH_WINDOW=3 (equal-weight average)
        kernel = np.ones(self.smooth_window) / self.smooth_window
        # half_w = 1 for SMOOTH_WINDOW=3; used as the pad width
        half_w = self.smooth_window // 2
        # Pad the array at both ends by repeating the edge value
        padded = np.pad(proc_ranges, half_w, mode='edge')
        # Convolve (slide the kernel across the array). mode='valid' means
        # only compute where the kernel fully overlaps the data — combined with
        # our manual padding, this gives an output the same length as the input.
        proc_ranges = np.convolve(padded, kernel, mode='valid')

        return proc_ranges


    # ==========================================================================
    # find_max_gap — Step 3: find the largest free corridor
    # ==========================================================================

    def find_max_gap(self, free_space_ranges):
        """
        Scan the array for the longest consecutive run of non-zero values.

        WHAT IS A "GAP"?
        After the safety bubble step, the array has two kinds of values:
          - 0.0: "blocked" — this direction is too close to an obstacle
          - > 0.0: "free" — the path in this direction is clear

        A "gap" is a consecutive sequence of free-space beams. The longest
        such sequence is the widest open corridor the car can aim for.

        WHY LONGEST RUN (not deepest gap, not widest angle)?
        Longest run = widest angular corridor = most physical space to drive
        through. Depth (distance) is already incorporated because the bubble
        radius zeroed out nearby obstacles — any remaining non-zero beam is
        implicitly "far enough away".

        ALGORITHM — single linear O(N) scan:
          Keep track of:
            cur_start  — start index of the current non-zero run (None if none)
            best_start — start of the best run seen so far
            best_len   — length of the best run seen so far
          For each index:
            non-zero → start or extend current run; update best if longer
            zero     → reset cur_start

        Args:
            free_space_ranges: np.ndarray where 0 = blocked, >0 = free.
                               Modified in-place by the bubble step; we only
                               read it here.

        Returns:
            (start_i, end_i): inclusive indices of the longest non-zero run.
                              If no non-zero values exist, returns (0, -1).
                              The caller (lidar_callback) checks for end_i < start_i.
        """
        best_start = 0
        best_len   = 0
        cur_start  = None   # None means "not currently in a free-space run"

        for i, val in enumerate(free_space_ranges):
            if val > 0.0:
                # We are in a free-space run (or just entered one).
                if cur_start is None:
                    cur_start = i           # record where this run started
                run_len = i - cur_start + 1 # current run length
                if run_len > best_len:
                    best_len   = run_len    # new longest run
                    best_start = cur_start
            else:
                # This beam is blocked (bubble or obstacle) — end the run.
                cur_start = None

        # The run ends at best_start + best_len - 1 (inclusive).
        # If best_len is still 0 (all beams blocked), this returns (0, -1).
        best_end = best_start + best_len - 1
        return best_start, best_end


    # ==========================================================================
    # find_best_point — Step 4: choose the aim point within the gap
    # ==========================================================================

    def find_best_point(self, start_i, end_i, ranges):
        """
        Select the beam index the car should steer toward within the max gap.

        WHY THE CENTER AND NOT THE FURTHEST POINT?
        The naive choice is "aim for the deepest (furthest) point in the gap".
        This FAILS in straight corridors:

          Imagine the car slightly to the left of a straight hall. The rightmost
          far corner is the "furthest point" → the car steers right. Now it is
          slightly to the right → left corner is furthest → steers left. This
          alternates every scan at 40Hz and causes an "S-shape wiggle" (visible
          oscillation). The car wastes speed and risks wall contact.

        The CENTER of the gap is stable: as the car shifts laterally, the center
        angle changes smoothly and gradually, giving smooth, continuous steering.

        This is "FTG Tweak 4" from the F1TENTH lecture slides (referred to in
        the README as the "Better Idea" method).

        Args:
            start_i: first index of the max gap (inclusive).
            end_i:   last  index of the max gap (inclusive).
            ranges:  the preprocessed ranges array (available for depth
                     computations if you want to add furthest-point logic later).

        Returns:
            best_idx: integer index to use as the steering target.
        """
        # Integer midpoint using floor division.
        # Example: gap from index 300 to 700 → best_idx = 500 (dead center).
        return (start_i + end_i) // 2


    # ==========================================================================
    # _choose_speed — Step 5: speed based on nearest obstacle
    # ==========================================================================

    def _choose_speed(self, min_range):
        """
        Return a target speed based on how close the nearest obstacle is.

        WHY PIECEWISE SPEED CONTROL?
        A fixed speed would either be too slow on open sections (wastes time)
        or too fast near obstacles (crashes). Reducing speed when obstacles
        are close gives more reaction time. This is a simple three-tier
        piecewise-constant approximation of a smooth speed curve.

        HOW IT MAPS:
          min_range > FAST_THRESHOLD (1.5m)  →  FAST_SPEED (3.0 m/s)
          min_range > MID_THRESHOLD  (0.5m)  →  MID_SPEED  (1.5 m/s)
          min_range ≤ 0.5m                   →  SLOW_SPEED (0.5 m/s)

        NOTE: min_range here is the minimum of the NON-ZERO preprocessed ranges
        AFTER the safety bubble has been applied. This means we are measuring
        "how far away is the nearest real obstacle that is NOT already inside
        the bubble zone", which is a better proxy for danger than the raw min.

        Args:
            min_range: float, minimum non-zero preprocessed range in metres.

        Returns:
            speed: float in m/s.
        """
        if min_range > self.fast_threshold:
            return self.fast_speed
        elif min_range > self.mid_threshold:
            return self.mid_speed
        else:
            return self.slow_speed


    def _compute_forward_window(self, angle_min, angle_increment, beam_count):
        """Return [start, end) indices for the configured forward field-of-view."""
        if beam_count <= 0 or angle_increment <= 0.0:
            return 0, beam_count

        center_idx = int(round((0.0 - angle_min) / angle_increment))
        center_idx = max(0, min(beam_count - 1, center_idx))
        half_fov_rad = np.deg2rad(self.forward_fov_deg * 0.5)
        half_beams = int(np.ceil(half_fov_rad / angle_increment))

        start = max(0, center_idx - half_beams)
        end = min(beam_count, center_idx + half_beams + 1)
        if end <= start:
            return 0, beam_count
        return start, end


    def _scale_speed_for_steering(self, base_speed, steering_angle):
        """Reduce speed as steering angle magnitude increases."""
        abs_angle = abs(steering_angle)
        if abs_angle <= self.steer_slowdown_start:
            return base_speed

        if self.steer_slowdown_end <= self.steer_slowdown_start:
            return base_speed

        if abs_angle >= self.steer_slowdown_end:
            scale = self.min_steer_speed_scale
        else:
            t = (abs_angle - self.steer_slowdown_start) / (
                self.steer_slowdown_end - self.steer_slowdown_start
            )
            scale = 1.0 - t * (1.0 - self.min_steer_speed_scale)

        return max(self.slow_speed, base_speed * scale)


    @staticmethod
    def _side_name(side):
        if side > 0:
            return 'left'
        if side < 0:
            return 'right'
        return 'none'


    def _sector_min(self, sector_ranges):
        """Return the minimum value in a sector, or max_range_clip when empty."""
        if len(sector_ranges) == 0:
            return self.max_range_clip
        return float(np.min(sector_ranges))


    def _side_sector_score(self, sector_ranges):
        """Score a side sector using finite non-zero means (higher is better)."""
        if len(sector_ranges) == 0:
            return 0.0
        valid = sector_ranges[np.isfinite(sector_ranges) & (sector_ranges > 0.0)]
        if len(valid) == 0:
            return 0.0
        return float(np.mean(valid))


    def _tick_turn_commit(self):
        """Advance commitment hysteresis by one scan."""
        if self.turn_commit_counter > 0:
            self.turn_commit_counter -= 1
            if self.turn_commit_counter == 0:
                self.turn_commit_side = 0


    def _apply_disparity_mask(self, ranges, angle_increment):
        """
        Mask far-side beams around disparity jumps to remove non-reachable gaps.

        Returns:
            tuple[int, int]: (disparity_count, masked_beam_count)
        """
        if (
            not self.disparity_enable
            or angle_increment <= 0.0
            or len(ranges) < 2
        ):
            return 0, 0

        disparity_count = 0
        masked_beam_count = 0
        beam_count = len(ranges)

        for i in range(beam_count - 1):
            left = float(ranges[i])
            right = float(ranges[i + 1])

            # Ignore blocked beams (bubble/masked beams) to avoid cascading.
            if left <= 0.0 or right <= 0.0:
                continue

            if abs(left - right) < self.disparity_threshold:
                continue

            disparity_count += 1
            near_range = min(left, right)
            if near_range <= 0.0:
                continue

            extend_beams = int(
                np.ceil(self.disparity_extend_width / (near_range * angle_increment))
            )
            extend_beams = int(
                np.clip(extend_beams, 1, self.disparity_max_extension_beams)
            )

            if left < right:
                # Obstacle edge with farther space on the right; mask into right side.
                start = i + 1
                end = min(beam_count, start + extend_beams)
            else:
                # Obstacle edge with farther space on the left; mask into left side.
                end = i + 1
                start = max(0, end - extend_beams)

            segment = ranges[start:end]
            masked_beam_count += int(np.count_nonzero(segment > 0.0))
            ranges[start:end] = 0.0

        return disparity_count, masked_beam_count


    # ==========================================================================
    # lidar_callback — the main per-scan entry point
    # ==========================================================================

    def lidar_callback(self, data):
        """
        Called by ROS2 every time a new LaserScan message arrives on /scan.

        WHEN IS THIS CALLED?
        The f1tenth_gym_ros simulator (and the real Hokuyo LiDAR) publish at
        ~40Hz. ROS2 queues incoming messages and calls this function once per
        message, in the order they arrived. Each call must finish quickly
        (ideally < 5ms) so we don't fall behind. NumPy operations on 1080
        elements take well under 1ms.

        WHAT IS 'data'?
        'data' is a LaserScan message object. It has these fields we use:
          data.ranges          — tuple of ~1080 float32 distance readings (m)
          data.angle_min       — angle of index 0 in radians (≈ -2.35 rad)
          data.angle_increment — angular step between beams (≈ 0.00436 rad)
        We do NOT hardcode these values — we read them from the message itself
        so the node works with any LiDAR or simulator configuration.

        FULL PIPELINE:
          Step 1: preprocess_lidar()  — sanitise and smooth
          Step 2: safety bubble       — zero beams near closest obstacle
                    Step 3: disparity masking   — close non-reachable pseudo-gaps
                    Step 4: find_max_gap()      — find widest free corridor
                    Step 5: find_best_point()   — pick aim index (center of gap)
                    Step 6: angle math          — index → steering angle in radians
                    Step 7: speed choice        — nearest obstacle → speed tier
                    Step 8: publish             — send AckermannDriveStamped to /drive

        Args:
            data: sensor_msgs.msg.LaserScan — the incoming LiDAR message.
        """

        # ── Read scan metadata from the message ────────────────────────────────
        # We read these from the message rather than hardcoding because:
        #   - The simulator and the real car may have slightly different configs
        #   - If someone changes sim.yaml, the node still works correctly
        self.scan_counter += 1

        angle_min       = data.angle_min        # radians; index 0 beam direction
        angle_increment = data.angle_increment  # radians per index step

        # ── Step 1: Preprocess the raw scan ───────────────────────────────────
        # Pass the raw tuple to our preprocessing method. Returns a clean
        # numpy array of the same length with: no inf/nan, clipped to
        # MAX_RANGE_CLIP, and smoothed with a windowed mean.
        proc_ranges = self.preprocess_lidar(data.ranges)

        # Restrict FTG processing to a forward cone around 0 rad.
        fov_start, fov_end = self._compute_forward_window(
            angle_min, angle_increment, len(proc_ranges)
        )
        fov_ranges = np.copy(proc_ranges[fov_start:fov_end])
        if len(fov_ranges) == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = self.slow_speed
            self.publisher.publish(drive_msg)
            self._tick_turn_commit()
            return

        # Front/side sector stats from pre-bubble data drive commit decisions.
        fov_center_global = int(round((0.0 - angle_min) / angle_increment))
        fov_center_global = max(fov_start, min(fov_end - 1, fov_center_global))
        fov_center_local = fov_center_global - fov_start

        front_half_beams = int(
            np.ceil(np.deg2rad(self.turn_commit_front_sector_deg * 0.5) / angle_increment)
        )
        side_beams = int(
            np.ceil(np.deg2rad(self.turn_commit_side_sector_deg) / angle_increment)
        )
        front_start = max(0, fov_center_local - front_half_beams)
        front_end = min(len(fov_ranges), fov_center_local + front_half_beams + 1)
        left_end = min(len(fov_ranges), fov_center_local + side_beams + 1)
        right_start = max(0, fov_center_local - side_beams)

        front_sector = fov_ranges[front_start:front_end]
        left_sector = fov_ranges[fov_center_local + 1:left_end]
        right_sector = fov_ranges[right_start:fov_center_local]

        front_min = self._sector_min(front_sector)
        left_min = self._sector_min(left_sector)
        right_min = self._sector_min(right_sector)
        left_score = self._side_sector_score(left_sector)
        right_score = self._side_sector_score(right_sector)

        blocked_front = front_min < self.turn_commit_front_min_threshold
        triggered_commit = False
        if (
            blocked_front
            and self.turn_commit_counter == 0
            and self.turn_commit_hold_scans > 0
        ):
            if left_score > right_score:
                self.turn_commit_side = 1
            elif right_score > left_score:
                self.turn_commit_side = -1
            else:
                self.turn_commit_side = 1 if left_min >= right_min else -1
            self.turn_commit_counter = self.turn_commit_hold_scans
            triggered_commit = True

        # ── Step 2: Safety bubble ──────────────────────────────────────────────

        # Find the index of the minimum (closest) range.
        # np.argmin returns the index of the smallest value in the array.
        # int() converts from numpy int64 to Python int (needed for slicing).
        min_idx = int(np.argmin(fov_ranges))
        min_range = fov_ranges[min_idx]   # the actual distance to that obstacle

        # Convert physical bubble radius to an index half-width.
        #
        # GEOMETRY:
        #   The LiDAR beams fan out from the sensor. Two adjacent beams at
        #   distance d are separated by arc length ≈ d × angle_increment.
        #   To cover a physical arc of BUBBLE_RADIUS metres at distance d,
        #   we need ≈ BUBBLE_RADIUS / (d × angle_increment) beam indices.
        #
        #   Formula: bubble_half = ceil( BUBBLE_RADIUS / (d × Δθ) )
        #
        #   ceil() so we always zero AT LEAST the required number of beams.
        #   +1 is implicit because we use a slice from -bubble_half to +bubble_half.
        #
        # EDGE CASE: if min_range == 0 (obstacle is touching the sensor), the
        # formula would divide by zero. We instead zero a quarter of the scan
        # (very conservative — the car is essentially in a crash scenario).
        if min_range > 0.0:
            bubble_half = int(np.ceil(self.bubble_radius / (min_range * angle_increment)))
        else:
            bubble_half = len(fov_ranges) // 4

        # Compute the slice bounds, clamped to valid array indices.
        # max(0, ...) prevents negative indices.
        # min(len(...), ...) prevents going past the end.
        bubble_start = max(0, min_idx - bubble_half)
        bubble_end = min(len(fov_ranges), min_idx + bubble_half + 1)

        # Zero out the bubble: set all beams within this angular arc to 0.
        # After this, proc_ranges represents "free space only" — any index
        # still > 0 is guaranteed to have BUBBLE_RADIUS of clearance.
        fov_ranges[bubble_start:bubble_end] = 0.0

        # ── Step 3: Disparity-aware masking (context-gated) ──────────────────
        side_clearance = max(left_min, right_min)
        disparity_active = (
            self.disparity_enable
            and front_min <= self.disparity_front_max_range
            and side_clearance >= self.disparity_min_side_clearance
        )

        if disparity_active:
            disparity_count, disparity_masked_beams = self._apply_disparity_mask(
                fov_ranges,
                angle_increment,
            )
        else:
            disparity_count, disparity_masked_beams = 0, 0

        if self.disparity_debug and (self.scan_counter % self.disparity_log_stride == 0):
            if disparity_active and disparity_count > 0:
                self.get_logger().info(
                    'disparity active count=%d masked_beams=%d threshold=%.2f extend_width=%.2f '
                    'front_min=%.2f side_clearance=%.2f'
                    % (
                        disparity_count,
                        disparity_masked_beams,
                        self.disparity_threshold,
                        self.disparity_extend_width,
                        front_min,
                        side_clearance,
                    )
                )
            elif not disparity_active:
                self.get_logger().info(
                    'disparity skipped front_min=%.2f side_clearance=%.2f gate_front<=%.2f gate_side>=%.2f'
                    % (
                        front_min,
                        side_clearance,
                        self.disparity_front_max_range,
                        self.disparity_min_side_clearance,
                    )
                )

        # ── Step 4: Find the max gap ───────────────────────────────────────────
        # Scan the zeroed array for the longest consecutive run of non-zeros.
        # Returns inclusive (start, end) indices of that run.
        gap_ranges = fov_ranges
        commit_side = self.turn_commit_side if self.turn_commit_counter > 0 else 0
        if commit_side > 0:
            gap_ranges = np.copy(fov_ranges)
            gap_ranges[:fov_center_local] = 0.0
        elif commit_side < 0:
            gap_ranges = np.copy(fov_ranges)
            gap_ranges[fov_center_local + 1:] = 0.0

        start_i, end_i = self.find_max_gap(gap_ranges)

        if end_i < start_i and commit_side != 0:
            start_i, end_i = self.find_max_gap(fov_ranges)

        # EDGE CASE: the bubble consumed every non-zero element, meaning the
        # car is completely surrounded by close obstacles (crash scenario).
        # find_max_gap returns (0, -1) when best_len == 0.
        # We detect this as end_i < start_i and publish a safe "go straight
        # slowly" command, then return early.
        if end_i < start_i:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0       # straight
            drive_msg.drive.speed          = self.slow_speed # crawl
            self.publisher.publish(drive_msg)
            if (
                self.turn_commit_debug
                and (blocked_front or commit_side != 0)
                and (
                    triggered_commit
                    or (self.scan_counter % self.turn_commit_log_stride == 0)
                )
            ):
                self.get_logger().info(
                    'turn_commit side=%s counter=%d blocked=%s front_min=%.2f left_min=%.2f right_min=%.2f '
                    'left_score=%.2f right_score=%.2f'
                    % (
                        self._side_name(commit_side),
                        self.turn_commit_counter,
                        blocked_front,
                        front_min,
                        left_min,
                        right_min,
                        left_score,
                        right_score,
                    )
                )
            self._tick_turn_commit()
            return

        # ── Step 5: Best point within the gap → steering angle ────────────────
        # find_best_point returns the center index of the gap.
        best_idx = fov_start + self.find_best_point(start_i, end_i, fov_ranges)

        # Convert beam index to angle in radians.
        # Derivation:
        #   angle_min is the angle for index 0 (the rightmost beam, ≈ -2.35 rad).
        #   Each successive index adds angle_increment radians.
        #   Therefore: angle(idx) = angle_min + idx × angle_increment
        #
        # Result:
        #   best_angle ≈ 0      → steer straight ahead
        #   best_angle > 0      → steer left  (gap is to the left)
        #   best_angle < 0      → steer right (gap is to the right)
        #   Hardware clamp: ±0.4189 rad (±24°) — physical steering limit
        best_angle = angle_min + best_idx * angle_increment
        best_angle = float(np.clip(best_angle, -self.steering_limit, self.steering_limit))

        # ── Step 6: Choose speed ───────────────────────────────────────────────
        # After applying the bubble, take the minimum of the non-zero ranges.
        # Using non-zero values avoids counting bubble zeros as "obstacle at 0m".
        #
        # proc_ranges > 0.0 produces a boolean mask.
        # proc_ranges[mask] selects only the free-space values.
        nonzero = fov_ranges[fov_ranges > 0.0]
        # If nonzero is empty (all blocked), effective_min = 0 → SLOW_SPEED.
        effective_min = float(np.min(nonzero)) if len(nonzero) > 0 else 0.0
        speed = self._choose_speed(effective_min)
        speed = self._scale_speed_for_steering(speed, best_angle)

        if (
            self.turn_commit_debug
            and (blocked_front or commit_side != 0)
            and (
                triggered_commit
                or (self.scan_counter % self.turn_commit_log_stride == 0)
            )
        ):
            self.get_logger().info(
                'turn_commit side=%s counter=%d blocked=%s front_min=%.2f left_min=%.2f right_min=%.2f '
                'left_score=%.2f right_score=%.2f'
                % (
                    self._side_name(commit_side),
                    self.turn_commit_counter,
                    blocked_front,
                    front_min,
                    left_min,
                    right_min,
                    left_score,
                    right_score,
                )
            )

        # ── Step 8: Build and publish the drive command ────────────────────────
        # AckermannDriveStamped is a ROS2 message with this structure:
        #   AckermannDriveStamped
        #     header                  (timestamp + frame_id — filled automatically)
        #     drive: AckermannDrive
        #       steering_angle        (radians; + = left; clamped by hardware)
        #       steering_angle_velocity (rad/s; 0 = instantaneous, we don't use)
        #       speed                 (m/s; + = forward)
        #       acceleration          (m/s²; 0 = let VESC decide, we don't use)
        #       jerk                  (m/s³; we don't use)
        #
        # We only set the two fields the simulator (and VESC) actually use:
        #   .drive.steering_angle  and  .drive.speed
        drive_msg = AckermannDriveStamped()

        # float() ensures we pass a plain Python float, not a numpy scalar.
        # Some ROS2 type checkers are strict about this.
        drive_msg.drive.steering_angle = float(best_angle)
        drive_msg.drive.speed          = float(speed)

        # self.publisher.publish(msg) serialises the message into the ROS2
        # DDS (Data Distribution Service) transport layer and delivers it to
        # all subscribers on /drive — in our case, the simulator's bridge node,
        # which translates it into a velocity command for the physics engine.
        self.publisher.publish(drive_msg)
        self._tick_turn_commit()


# =============================================================================
# main — ROS2 node entry point
# =============================================================================

def main(args=None):
    """
    Entry point for the ROS2 node. Called by:
      - ros2 run gap_follow reactive_node   (via setup.py console_scripts)
      - python3 scripts/reactive_node.py    (direct execution for testing)
      - the if __name__ == '__main__' block below

    WHAT DOES rclpy.init() DO?
    Initialises the ROS2 middleware (DDS). Must be called exactly once per
    process, before any Node is created. 'args' allows passing command-line
    arguments (e.g. remapping topics) — passing None means use sys.argv.

    WHAT DOES rclpy.spin() DO?
    Enters the ROS2 event loop. The process blocks here indefinitely,
    processing incoming messages and calling callbacks (lidar_callback).
    This is the ROS2 equivalent of "while True: check_for_messages()".
    Returns only when rclpy.shutdown() is called (e.g. Ctrl+C → SIGINT).

    WHAT DOES destroy_node() + rclpy.shutdown() DO?
    Clean teardown: unregisters the node from the DDS graph, closes sockets,
    and frees resources. Without this, the next ros2 run command might see a
    stale node entry.
    """
    # Start the ROS2 runtime. args=None → read sys.argv for ROS2 remappings.
    rclpy.init(args=args)

    print("ReactiveFollowGap node initialized")

    # Instantiate our node. __init__ registers the subscriber and publisher.
    reactive_node = ReactiveFollowGap()

    # Block here and process messages until Ctrl+C or ros2 lifecycle stop.
    rclpy.spin(reactive_node)

    # Reached only after spin() returns (i.e. after shutdown signal).
    reactive_node.destroy_node()
    rclpy.shutdown()


# Standard Python idiom: only run main() if this file is executed directly
# (not when it is imported as a module by, e.g., unit tests).
if __name__ == '__main__':
    main()