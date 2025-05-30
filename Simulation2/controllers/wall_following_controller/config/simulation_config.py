class SimulationConfig:
    # PID Controller Parameters
    PID_KP = 0.2  # Further reduced for even gentler control
    PID_KI = 0.005  # Reduced to prevent integral windup
    PID_KD = 0.02  # Reduced for smoother control

    # Robot Physical Parameters
    WHEEL_RADIUS = 0.03  # in meters
    WHEEL_DISTANCE = 0.1  # distance between wheels in meters
    
    # Sensor Parameters
    LIDAR_ENABLE_TIME = 32  # LIDAR update time in milliseconds (same as timestep)
    LIDAR_FOV = 3.14159  # LIDAR field of view in radians (180 degrees)
    LIDAR_RESOLUTION = 360  # Number of beams in the LIDAR sensor
    
    # Control Parameters
    WALL_DISTANCE_THRESHOLD = 0.35  # Slightly reduced threshold distance
    DEFAULT_SPEED = 2.5  # Reduced default speed for better control
    MIN_SPEED = 1.5  # Increased minimum speed to prevent stalling
    MAX_SPEED = 3.5  # Reduced maximum speed for better control
    MAX_CORRECTION = 0.8  # Reduced maximum PID correction
    
    # Safety Parameters
    MIN_SAFE_DISTANCE = 0.15  # minimum safe distance in meters
    MAX_LIDAR_RANGE = 3.0  # reduced maximum expected LIDAR range
    
    # Simulation Parameters
    TIMESTEP = 32  # time step in milliseconds for the robot simulation