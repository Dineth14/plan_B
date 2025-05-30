class SimulationConfig:
    PID_KP = 1.0
    PID_KI = 0.1
    PID_KD = 0.05
    WHEEL_RADIUS = 0.03  # in meters
    WHEEL_DISTANCE = 0.1  # distance between wheels in meters
    WALL_DISTANCE_THRESHOLD = 0.5  # threshold distance to the wall in meters
    DEFAULT_SPEED = 1.0  # default speed of the robot in m/s
    TIMESTEP = 32  # time step in milliseconds for the robot simulation