# plan_B

A Python-based maze-solving robot using a LIDAR sensor.

## Description

plan_B is a robotics project designed to autonomously solve mazes by utilizing real-time distance and mapping data from a LIDAR sensor. The robot navigates unknown environments using pathfinding algorithms, making it ideal for educational purposes, robotics competitions, or as a foundation for advanced autonomous systems.

## Features

- Autonomous maze navigation
- LIDAR-based mapping and obstacle detection
- Pathfinding algorithms (e.g., DFS, BFS, A*)
- Real-time data processing
- Modular Python codebase

## Requirements

- Python 3.x
- LIDAR sensor (e.g., RPLIDAR)
- [List your microcontroller or robot platform, e.g., Raspberry Pi, Arduino, etc.]
- Required Python libraries:  
  - numpy  
  - matplotlib  
  - [Add others, e.g., rplidar, etc.]

## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/Dineth14/plan_B.git
    cd plan_B
    ```
2. Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3. Connect your LIDAR sensor and ensure drivers are installed.

## Usage

1. Set up your hardware according to the schematic in the `docs/` folder (add if available).
2. Modify configuration files as needed.
3. Run the main program:
    ```bash
    python main.py
    ```
4. Monitor output/logs for navigation status.

## Project Structure
plan_B/
│
├── main.py
├── lidar/
├── navigation/
├── utils/
├── requirements.txt
└── README.md
## Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](LICENSE)

## Author

- Dineth14
