class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, accel, gyro, orientation):
        accel_angle = self.calculate_accel_angle(accel)
        gyro_angle = orientation[2] + gyro[2] * (1.0 / 60.0)  # Assuming a timestep of 1/60 seconds

        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        return self.angle,

    def calculate_accel_angle(self, accel):
        return np.arctan2(accel[1], accel[0])