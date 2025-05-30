class WebotsEncoderAdapter:
    def __init__(self, left_wheel_sensor, right_wheel_sensor, wheel_radius, wheel_distance, config=None):
        self.left_wheel_sensor = left_wheel_sensor
        self.right_wheel_sensor = right_wheel_sensor
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.config = config
        
        # Enable sensors with default timestep if config is not provided
        timestep = config.TIMESTEP if config else 32
        self.enable(timestep)

    def enable(self, timestep):
        """Enable the wheel sensors with the specified timestep."""
        self.left_wheel_sensor.enable(timestep)
        self.right_wheel_sensor.enable(timestep)

    def get_data(self):
        left_distance = self.left_wheel_sensor.getValue() * self.wheel_radius
        right_distance = self.right_wheel_sensor.getValue() * self.wheel_radius
        velocity = (left_distance + right_distance) / 2.0  # Average velocity
        
        return {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'velocity': velocity
        }