# Wall Following and Obstacle Avoiding Controller for Two-Wheeled Robot

## Overview
This project implements a wall-following and obstacle-avoiding controller for a two-wheeled robot. The controller utilizes various sensors, including LIDAR and IMU, to navigate and maintain a safe distance from walls and obstacles. The robot is designed to follow the inner wall of an environment while avoiding collisions.

## Features
- Wall following behavior using LIDAR distance measurements.
- Obstacle avoidance by detecting nearby obstacles and adjusting motor speeds accordingly.
- Integration of IMU data for improved orientation estimation.
- PID control for smooth motor speed adjustments.

## Project Structure
```
two_wheel_wall_following
├── src
│   ├── wall_following_controller.py       # Main controller for wall following and obstacle avoidance
│   ├── adapters
│   │   ├── webots_lidar_adapter.py        # LIDAR sensor interface
│   │   ├── webots_imu_adapter.py          # IMU sensor interface
│   │   └── webots_encoder_adapter.py      # Wheel encoder interface
│   ├── core
│   │   └── robot_state.py                  # Maintains robot's state information
│   ├── filters
│   │   └── complementary.py                 # Implements complementary filter for orientation
│   ├── motor_control
│   │   └── pid_controller.py                # PID controller for motor speed management
│   └── config
│       └── simulation_config.py             # Configuration constants
├── requirements.txt                        # Python package dependencies
└── README.md                               # Project documentation
```

## Setup Instructions
1. Clone the repository to your local machine:
   ```
   git clone <repository-url>
   ```
2. Navigate to the project directory:
   ```
   cd two_wheel_wall_following
   ```
3. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage
To run the wall-following controller, execute the following command:
```
python src/wall_following_controller.py
```

Ensure that your robot simulation environment is properly set up to utilize the LIDAR and IMU sensors.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.