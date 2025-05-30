class WebotsEncoderAdapter:
    def __init__(self, left_wheel_sensor, right_wheel_sensor, wheel_radius, wheel_distance):
        self.left_wheel_sensor = left_wheel_sensor
        self.right_wheel_sensor = right_wheel_sensor
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        
        self.left_wheel_sensor.enable(1)  # Enable the left wheel sensor
        self.right_wheel_sensor.enable(1)  # Enable the right wheel sensor

    def get_data(self):
        left_distance = self.left_wheel_sensor.getValue() * self.wheel_radius
        right_distance = self.right_wheel_sensor.getValue() * self.wheel_radius
        velocity = (left_distance + right_distance) / 2.0  # Average velocity
        
        return {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'velocity': velocity
        }