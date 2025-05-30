class ComplementaryFilter:
    def __init__(self, alpha=0.98, timestep=1.0/60.0):
        self.alpha = alpha
        self.angle = 0.0
        self.timestep = timestep

    def update(self, accel, gyro, orientation):
        accel_angle = self.calculate_accel_angle(accel)
        gyro_angle = orientation[2] + gyro[2] * self.timestep

        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        return self.angle

    def calculate_accel_angle(self, accel):
        return np.arctan2(accel[1], accel[0])