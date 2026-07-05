import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import torch
import math
from pytorch_mppi import MPPI

WHEELBASE = 0.33 
DT = 0.1 

def vehicle_dynamics(state, action):
    x = state[:, 0]
    y = state[:, 1]
    yaw = state[:, 2]
    v = state[:, 3]

    steering_angle = action[:, 0]
    acceleration = action[:, 1]

    next_x = x + v * torch.cos(yaw) * DT
    next_y = y + v * torch.sin(yaw) * DT
    next_yaw = yaw + (v / WHEELBASE) * torch.tan(steering_angle) * DT
    next_v = v + acceleration * DT

    next_state = torch.stack([next_x, next_y, next_yaw, next_v], dim=1)
    return next_state


class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_node')
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"MPPI Initialized on device: {self.device}")

        self.mppi_ctrl = MPPI(
            dynamics=vehicle_dynamics,
            running_cost=self.running_cost,
            nx=4, 
            noise_sigma=torch.tensor([[0.15, 0.0], [0.0, 0.5]], device=self.device), 
            num_samples=200, 
            horizon=20,      
            lambda_=1.0,
            device=self.device,
            u_min=torch.tensor([-0.4, -1.5], device=self.device), 
            u_max=torch.tensor([0.4, 1.5], device=self.device)    
        )

        self.current_state = None
        self.latest_scan = None
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.obstacles = None 
        self.target_yaw = 0.0 

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.timer = self.create_timer(DT, self.control_loop)

    def scan_callback(self, msg):
        self.latest_scan = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        v = msg.twist.twist.linear.x

        self.current_state = torch.tensor([x, y, yaw, v], dtype=torch.float32, device=self.device)

    def running_cost(self, state, action):
        v = state[:, 3]
        steering_angle = action[:, 0]
        pred_yaw = state[:, 2]

        speed_cost = -20.0 * v 
        steering_cost = 3.0 * (steering_angle ** 2)

        yaw_error = pred_yaw - self.target_yaw
        yaw_error = torch.atan2(torch.sin(yaw_error), torch.cos(yaw_error)) 
        heading_cost = 60.0 * (yaw_error ** 2)

        stop_penalty = torch.where(v < 0.2, 500.0 * (0.2 - v), 0.0)

        crash_cost = torch.zeros_like(v)
        if self.obstacles is not None and self.obstacles.shape[0] > 0:
            pred_x = state[:, 0:1] 
            pred_y = state[:, 1:2] 
            
            obs_x = self.obstacles[:, 0].unsqueeze(0) 
            obs_y = self.obstacles[:, 1].unsqueeze(0)
            
            dists = torch.sqrt((pred_x - obs_x)**2 + (pred_y - obs_y)**2)
            min_dists, _ = torch.min(dists, dim=1)
            
            safety_threshold = 1.5
            wall_penalty = torch.where(min_dists < safety_threshold, 600.0 * (safety_threshold - min_dists) ** 2, 0.0)
            critical_penalty = torch.where(min_dists < 0.35, 8000.0, 0.0)
            
            crash_cost = wall_penalty + critical_penalty

        return speed_cost + steering_cost + heading_cost + stop_penalty + crash_cost
    
    def control_loop(self):
        if self.current_state is None or self.latest_scan is None:
            return 

        curr_x = self.current_state[0].item()
        curr_y = self.current_state[1].item()
        curr_yaw = self.current_state[2].item()

        valid_ranges = np.where((self.latest_scan > 0.1) & (self.latest_scan < 10.0), self.latest_scan, 0.0)
        if len(valid_ranges) > 0 and np.max(valid_ranges) > 0.0:
            best_idx = np.argmax(valid_ranges)
            target_local_angle = self.angle_min + best_idx * self.angle_inc
            self.target_yaw = curr_yaw + target_local_angle

        step = 20
        ranges = self.latest_scan[::step]
        angles = self.angle_min + np.arange(0, len(self.latest_scan), step) * self.angle_inc

        valid = (ranges > 0.1) & (ranges < 10.0)
        ranges = ranges[valid]
        angles = angles[valid]

        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)

        glob_x = curr_x + local_x * np.cos(curr_yaw) - local_y * np.sin(curr_yaw)
        glob_y = curr_y + local_x * np.sin(curr_yaw) + local_y * np.cos(curr_yaw)

        self.obstacles = torch.tensor(np.vstack((glob_x, glob_y)).T, dtype=torch.float32, device=self.device)

        action = self.mppi_ctrl.command(self.current_state)
        best_steering = action[0].item()
        best_acceleration = action[1].item()

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = best_steering
        
        current_v = self.current_state[3].item()
        drive_msg.drive.speed = max(0.0, current_v + best_acceleration * DT) 
        
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    mppi_node = MPPIController()
    rclpy.spin(mppi_node)
    mppi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()