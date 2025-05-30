class SimulationConfig:
    def __init__(self):
        # Sensor settings
        self.LIDAR_ENABLE_TIME = 32  # ms, for compatibility with adapters
        self.LIDAR_TIMESTEP = 32     # ms, for compatibility with adapters
        self.LIDAR_FOV = 6.28319     # 2*pi, full circle
        self.LIDAR_MAX_DISTANCE = 5.0
        self.WHEEL_RADIUS = 0.03     # meters, match robot
        self.WHEEL_DISTANCE = 0.1    # meters, match robot

        # PID controller gains
        self.PID_KP = 1.0
        self.PID_KI = 0.1
        self.PID_KD = 0.05

        # Simulation parameters
        self.DISPLAY_WIDTH = 800
        self.DISPLAY_HEIGHT = 600
        self.MAX_MOTOR_SPEED = 6.28  # rad/s

        # Wall following parameters
        self.WALL_DISTANCE_THRESHOLD = 0.5  # meters
        self.DEFAULT_SPEED = 1.0            # m/s

        # Time step for sensors and control loop
        self.TIMESTEP = 32                  # ms