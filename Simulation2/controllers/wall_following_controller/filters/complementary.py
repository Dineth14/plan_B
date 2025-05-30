import numpy as np

class ComplementaryFilter:
    def __init__(self, alpha=0.98, timestep=1.0/60.0):
        self.alpha = alpha
        self.angle = 0.0
        self.timestep = timestep
        self.last_gyro_angle = 0.0

    def update(self, accel, gyro, orientation):
        # Get yaw angle from orientation (roll, pitch, yaw)
        current_orientation = orientation[2]  # yaw angle
        
        # Calculate angle from accelerometer
        accel_angle = self.calculate_accel_angle(accel)
        
        # Integrate gyro data
        gyro_rate = gyro[2]  # angular velocity around z-axis
        gyro_angle = self.last_gyro_angle + gyro_rate * self.timestep
        self.last_gyro_angle = gyro_angle
        
        # Complementary filter
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        
        return self.angle

    def calculate_accel_angle(self, accel):
        # Calculate angle from accelerometer data
        # Using atan2 to get angle in correct quadrant
        return np.arctan2(accel[1], accel[0])

    def reset(self):
        """Reset the filter state."""
        self.angle = 0.0
        self.last_gyro_angle = 0.0