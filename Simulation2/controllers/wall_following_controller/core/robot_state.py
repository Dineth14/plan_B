import numpy as np

class RobotState:
    def __init__(self):
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation (in radians)
        self.velocity = 0.0  # Robot's linear velocity
        self.angular_velocity = 0.0  # Robot's angular velocity
        self.lidar_scan = []  # LIDAR scan data

    def update(self, x, y, theta, velocity, angular_velocity, lidar_scan):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.lidar_scan = lidar_scan