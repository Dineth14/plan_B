"""
Intelligent Maze Solving Robot Controller
This controller implements a comprehensive mapping and navigation system using:
- Occupancy grid mapping
- A* path planning
- Improved obstacle detection and avoidance
- State-based navigation with memory
"""

from controller import Robot
import numpy as np
import math
import os
import sys
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import List, Tuple, Dict, Set, Optional
import heapq

# Add the controller directory to Python path
controller_dir = os.path.dirname(os.path.abspath(__file__))
if controller_dir not in sys.path:
    sys.path.append(controller_dir)

from core.robot_state import RobotState
from adapters.webots_lidar_adapter import WebotsLidarAdapter
from adapters.webots_encoder_adapter import WebotsEncoderAdapter
from config.simulation_config import SimulationConfig

@dataclass
class GridCell:
    """Represents a cell in the occupancy grid."""
    x: int
    y: int
    is_obstacle: bool = False
    is_visited: bool = False
    confidence: float = 0.0  # Confidence in obstacle detection
    last_update: float = 0.0  # Time of last update

@dataclass
class PathNode:
    """Represents a node in the path planning graph."""
    x: float
    y: float
    g_cost: float = float('inf')  # Cost from start
    h_cost: float = 0.0  # Heuristic cost to goal
    parent: Optional['PathNode'] = None
    
    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class OccupancyGrid:
    """Implements an occupancy grid map with probabilistic updates."""
    def __init__(self, resolution: float = 0.1, width: int = 100, height: int = 100):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid: Dict[Tuple[int, int], GridCell] = {}
        self.origin_x = width // 2
        self.origin_y = height // 2
        self.log_odds_prior = 0.0
        self.log_odds_hit = 0.7
        self.log_odds_miss = -0.4
        self.max_log_odds = 5.0
        self.min_log_odds = -5.0
        self.current_time = 0.0  # Initialize current_time
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int(x / self.resolution) + self.origin_x
        grid_y = int(y / self.resolution) + self.origin_y
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        x = (grid_x - self.origin_x) * self.resolution
        y = (grid_y - self.origin_y) * self.resolution
        return x, y
    
    def update_cell(self, x: float, y: float, is_obstacle: bool, confidence: float = 1.0):
        """Update a cell in the grid with new sensor data."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return
            
        key = (grid_x, grid_y)
        if key not in self.grid:
            self.grid[key] = GridCell(grid_x, grid_y)
            
        cell = self.grid[key]
        log_odds = self.log_odds_hit if is_obstacle else self.log_odds_miss
        cell.confidence = np.clip(cell.confidence + log_odds * confidence,
                                self.min_log_odds, self.max_log_odds)
        cell.is_obstacle = cell.confidence > 0
        cell.last_update = self.current_time
    
    def get_cell(self, x: float, y: float) -> Optional[GridCell]:
        """Get cell at world coordinates."""
        grid_x, grid_y = self.world_to_grid(x, y)
        return self.grid.get((grid_x, grid_y))
    
    def is_obstacle(self, x: float, y: float) -> bool:
        """Check if a world coordinate is occupied by an obstacle."""
        cell = self.get_cell(x, y)
        return cell is not None and cell.is_obstacle
    
    def mark_visited(self, x: float, y: float):
        """Mark a cell as visited."""
        cell = self.get_cell(x, y)
        if cell:
            cell.is_visited = True
    
    def get_neighbors(self, x: float, y: float) -> List[Tuple[float, float]]:
        """Get valid neighboring cells for path planning."""
        grid_x, grid_y = self.world_to_grid(x, y)
        neighbors = []
        
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = grid_x + dx, grid_y + dy
            if 0 <= new_x < self.width and 0 <= new_y < self.height:
                world_x, world_y = self.grid_to_world(new_x, new_y)
                if not self.is_obstacle(world_x, world_y):
                    neighbors.append((world_x, world_y))
        
        return neighbors

    def update_time(self, time: float):
        """Update the current time of the grid."""
        self.current_time = time

class PathPlanner:
    """Implements A* path planning on the occupancy grid."""
    def __init__(self, grid: OccupancyGrid):
        self.grid = grid
        
    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate heuristic cost between two points."""
        return math.hypot(b[0] - a[0], b[1] - a[1])
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path using A* algorithm."""
        start_node = PathNode(start[0], start[1], 0, self.heuristic(start, goal))
        open_set = [start_node]
        closed_set = set()
        node_dict = {start: start_node}
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if self.heuristic((current.x, current.y), goal) < self.grid.resolution:
                # Reconstruct path
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]
            
            closed_set.add((current.x, current.y))
            
            for next_pos in self.grid.get_neighbors(current.x, current.y):
                if next_pos in closed_set:
                    continue
                    
                g_cost = current.g_cost + self.heuristic((current.x, current.y), next_pos)
                
                if next_pos not in node_dict:
                    node_dict[next_pos] = PathNode(next_pos[0], next_pos[1])
                
                neighbor = node_dict[next_pos]
                if g_cost < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = g_cost
                    neighbor.h_cost = self.heuristic(next_pos, goal)
                    
                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)
        
        return []  # No path found

class IntelligentMazeSolver:
    """Main controller class implementing intelligent maze solving."""
    def __init__(self):
        # Initialize robot and get timestep
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize configuration
        self.config = SimulationConfig()
        
        # Initialize sensors and motors
        self._init_sensors()
        self._init_motors()
        
        # Initialize mapping and planning
        self.grid = OccupancyGrid()
        self.planner = PathPlanner(self.grid)
        self.current_path = []
        self.path_index = 0
        
        # Robot state
        self.state = RobotState()
        self.current_state = 'MAPPING'  # States: MAPPING, PLANNING, MOVING, RECOVERY
        
        # Navigation parameters
        self.target_position = None
        self.last_position = None
        self.position_history = deque(maxlen=50)
        self.stuck_counter = 0
        self.MAX_STUCK_COUNT = 10
        
        # Safety parameters
        self.EMERGENCY_DISTANCE = 0.15
        self.SLOW_DOWN_DISTANCE = 0.30
        self.MIN_SAFE_DISTANCE = 0.20
        self.OBSTACLE_AVOIDANCE_DISTANCE = 0.25  # Distance to start avoiding obstacles
        
        # Movement parameters
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.MAX_SPEED = self.config.MAX_SPEED
        self.BASE_SPEED = self.config.DEFAULT_SPEED
        self.MIN_SPEED = self.BASE_SPEED * 0.2  # Minimum speed to maintain movement
        
        # Recovery parameters
        self.recovery_counter = 0
        self.MAX_RECOVERY_STEPS = 20
        self.recovery_direction = 1  # 1 for right, -1 for left
        self.last_recovery_time = 0
        self.RECOVERY_COOLDOWN = 1000  # ms between recovery attempts
        
        # Sensor sectors
        self.FRONT_SECTOR = (-0.15, 0.15)
        self.LEFT_SECTOR = (0.15, 0.45)
        self.RIGHT_SECTOR = (-0.45, -0.15)
        self.FRONT_LEFT_SECTOR = (0.15, 0.30)  # New sector for better obstacle detection
        self.FRONT_RIGHT_SECTOR = (-0.30, -0.15)  # New sector for better obstacle detection
        
        # LIDAR parameters
        self.MIN_LIDAR_RANGE = 0.08  # Increased to avoid getting too close to obstacles
        self.MAX_LIDAR_RANGE = 5.0
        self.MIN_VALID_POINTS = 40
        self.LIDAR_SCAN_TIMEOUT = 1000
        self.last_valid_scan_time = 0
        self.consecutive_invalid_scans = 0
        self.MAX_INVALID_SCANS = 5
        self.initial_scan_done = False
        self.initial_cells_marked = False
        
        # State management
        self.state_timeout = 0
        self.STATE_TIMEOUT = 5000  # ms
        self.mapping_failures = 0
        self.MAX_MAPPING_FAILURES = 10
        self.visited_cells = set()
        self.position_history = deque(maxlen=100)
        self.orientation_history = deque(maxlen=100)
        self.last_state_change_time = 0
        self.last_debug_print_time = 0
        self.DEBUG_PRINT_INTERVAL = 1000  # ms
        
        # Recovery parameters
        self.recovery_attempts = 0
        self.MAX_RECOVERY_ATTEMPTS = 3  # Reduced from 5 to 3 as requested
        self.escape_rotation_direction = 1
        self.escape_rotation_time = 0
        self.ESCAPE_ROTATION_DURATION = 2000
        self.recovery_rotation_speed = 1.0
        
        # Grid parameters
        self.grid.resolution = 0.2
        self.frontier_cells = set()
        self.last_frontier_update = 0
        self.FRONTIER_UPDATE_INTERVAL = 1000
        self.frontier_search_radius = 5
        self.INITIAL_GRID_SIZE = 5  # Increased from 3 to 5 for larger initial area
        
        # Movement parameters
        self.last_position = None
        self.position_history = deque(maxlen=50)
        self.stuck_counter = 0
        self.MAX_STUCK_COUNT = 10
        self.MIN_MOVEMENT_THRESHOLD = 0.05  # Minimum movement to not be considered stuck
        self.exploratory_rotation_speed = 1.5  # Speed for exploratory rotation
        self.exploratory_rotation_time = 0
        self.EXPLORATORY_ROTATION_DURATION = 3000  # ms
        
    def _init_sensors(self):
        """Initialize and enable all sensors."""
        # LIDAR
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        # Wheel encoders
        self.left_wheel_sensor = self.robot.getDevice('left_wheel_sensor')
        self.right_wheel_sensor = self.robot.getDevice('right_wheel_sensor')
        self.left_wheel_sensor.enable(self.timestep)
        self.right_wheel_sensor.enable(self.timestep)
        
        # Inertial unit
        self.inertial_unit = self.robot.getDevice('inertial_unit')
        self.inertial_unit.enable(self.timestep)
        
        # Initialize adapters
        self.lidar_adapter = WebotsLidarAdapter(self.lidar, self.config)
        self.encoder_adapter = WebotsEncoderAdapter(
            self.left_wheel_sensor,
            self.right_wheel_sensor,
            self.config.WHEEL_RADIUS,
            self.config.WHEEL_DISTANCE,
            self.config
        )
    
    def _init_motors(self):
        """Initialize and configure motors."""
        self.left_motor = self.robot.getDevice('left_wheel_motor')
        self.right_motor = self.robot.getDevice('right_wheel_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    def get_valid_ranges(self, ranges) -> List[float]:
        """Filter LIDAR ranges with more lenient criteria."""
        if ranges is None or len(ranges) == 0:
            return []
        
        # Convert to numpy array if it's a list
        if isinstance(ranges, list):
            ranges = np.array(ranges)
        
        # More lenient filtering:
        # 1. Accept points very close to robot (0.05m)
        # 2. Use median filtering to remove outliers
        # 3. Accept more points in the valid range
        valid_mask = (
            ~np.isnan(ranges) & 
            ~np.isinf(ranges) & 
            (ranges >= self.MIN_LIDAR_RANGE) & 
            (ranges <= self.MAX_LIDAR_RANGE)
        )
        
        # Apply median filter to remove outliers
        if np.sum(valid_mask) > 0:
            median_dist = np.median(ranges[valid_mask])
            std_dist = np.std(ranges[valid_mask])
            valid_mask &= (abs(ranges - median_dist) <= 3 * std_dist)
        
        valid_ranges = ranges[valid_mask].tolist()
        return valid_ranges

    def get_sector_distances(self, start_angle: float, end_angle: float) -> List[float]:
        """Get valid distances from LIDAR scan for a given sector."""
        ranges = self.state.lidar_scan
        if ranges is None or len(ranges) == 0:
            return []
            
        # Convert to numpy array if it's a list
        if isinstance(ranges, list):
            ranges = np.array(ranges)
            
        angle_step = 2 * np.pi / len(ranges)
        start_idx = int((start_angle + np.pi) / angle_step)
        end_idx = int((end_angle + np.pi) / angle_step)
        
        if start_idx > end_idx:
            sector = np.concatenate([ranges[start_idx:], ranges[:end_idx]])
        else:
            sector = ranges[start_idx:end_idx]
        
        # Use the same validation as get_valid_ranges
        valid_mask = (
            ~np.isnan(sector) & 
            ~np.isinf(sector) & 
            (sector >= self.MIN_LIDAR_RANGE) & 
            (sector <= self.MAX_LIDAR_RANGE)
        )
        return sector[valid_mask].tolist()

    def update_robot_state(self):
        """Update robot state with improved LIDAR handling."""
        current_time = self.robot.getTime() * 1000
        
        # Get sensor data
        lidar_scan = self.lidar_adapter.get_distances()
        encoder_data = self.encoder_adapter.get_data()
        orientation = self.inertial_unit.getRollPitchYaw()
        
        # Validate LIDAR scan
        valid_ranges = self.get_valid_ranges(lidar_scan)
        valid_count = len(valid_ranges)
        
        if valid_count < self.MIN_VALID_POINTS:
            self.consecutive_invalid_scans += 1
            if self.consecutive_invalid_scans >= self.MAX_INVALID_SCANS:
                print(f"Warning: Too many invalid scans ({self.consecutive_invalid_scans})")
                self.current_state = 'RECOVERY'
                return
        else:
            self.consecutive_invalid_scans = 0
            self.last_valid_scan_time = current_time
            lidar_scan = valid_ranges
        
        # Update state
        self.state.update(
            x=self.state.x + encoder_data['velocity'] * math.cos(orientation[2]) * self.timestep / 1000.0,
            y=self.state.y + encoder_data['velocity'] * math.sin(orientation[2]) * self.timestep / 1000.0,
            theta=orientation[2],
            velocity=encoder_data['velocity'],
            angular_velocity=(encoder_data['right_distance'] - encoder_data['left_distance']) / self.config.WHEEL_DISTANCE,
            lidar_scan=lidar_scan
        )
        
        # Update history
        self.position_history.append((self.state.x, self.state.y))
        self.orientation_history.append(orientation[2])
        
        # Update grid and frontiers
        if current_time - self.last_frontier_update > self.FRONTIER_UPDATE_INTERVAL:
            self.update_frontiers()
            self.last_frontier_update = current_time
        
        # Update visited cells
        grid_x, grid_y = self.grid.world_to_grid(self.state.x, self.state.y)
        self.visited_cells.add((grid_x, grid_y))
        
        # Check for state timeout
        if current_time - self.last_state_change_time > self.STATE_TIMEOUT:
            print(f"State timeout in {self.current_state}, forcing state change")
            self.force_state_change()

    def update_frontiers(self):
        """Update frontier cells with improved detection."""
        self.frontier_cells.clear()
        current_grid_x, current_grid_y = self.grid.world_to_grid(self.state.x, self.state.y)
        
        # Use BFS to find reachable unexplored areas
        queue = deque([(current_grid_x, current_grid_y)])
        visited = set([(current_grid_x, current_grid_y)])
        reachable_frontiers = set()
        
        while queue and len(reachable_frontiers) < 50:  # Limit search to prevent infinite loops
            grid_x, grid_y = queue.popleft()
            
            # Check neighbors
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                new_x, new_y = grid_x + dx, grid_y + dy
                
                if (new_x, new_y) in visited:
                    continue
                    
                if not (0 <= new_x < self.grid.width and 0 <= new_y < self.grid.height):
                    continue
                
                # Check if cell is reachable
                cell = self.grid.grid.get((new_x, new_y))
                if cell is None or not cell.is_obstacle:
                    visited.add((new_x, new_y))
                    
                    # If cell is unvisited and has a visited neighbor, it's a frontier
                    if cell is None or not cell.is_visited:
                        has_visited_neighbor = False
                        for nx, ny in [(new_x+1, new_y), (new_x-1, new_y), 
                                     (new_x, new_y+1), (new_x, new_y-1)]:
                            if (nx, ny) in self.visited_cells:
                                has_visited_neighbor = True
                                break
                        
                        if has_visited_neighbor:
                            reachable_frontiers.add((new_x, new_y))
                    
                    queue.append((new_x, new_y))
        
        self.frontier_cells = reachable_frontiers
        print(f"Found {len(self.frontier_cells)} reachable frontier cells")

    def perform_initial_scan(self):
        """Perform initial 360-degree scan of the environment."""
        current_time = self.robot.getTime()
        
        if not self.initial_scan_done:
            print("Performing initial 360-degree scan...")
            # Rotate slowly while scanning
            self.set_motor_speeds(-self.recovery_rotation_speed, self.recovery_rotation_speed)
            
            # Update grid during rotation
            self.update_robot_state()
            
            # Check if we've completed a full rotation (2.5 seconds)
            if current_time > 2.5:
                print("Initial scan complete")
                self.initial_scan_done = True
                self.set_motor_speeds(0, 0)  # Stop rotation
                return True
            return False
        return True

    def print_debug_info(self):
        """Print detailed debug information."""
        current_time = self.robot.getTime() * 1000
        
        if current_time - self.last_debug_print_time < self.DEBUG_PRINT_INTERVAL:
            return
            
        self.last_debug_print_time = current_time
        
        # Get current LIDAR stats
        valid_ranges = self.get_valid_ranges(self.state.lidar_scan)
        valid_count = len(valid_ranges)
        total_points = len(self.state.lidar_scan) if self.state.lidar_scan is not None else 0
        
        print("\n=== Debug Information ===")
        print(f"Time: {current_time/1000:.1f}s")
        print(f"State: {self.current_state}")
        print(f"Position: ({self.state.x:.2f}, {self.state.y:.2f})")
        print(f"Orientation: {math.degrees(self.state.theta):.1f}°")
        print(f"LIDAR: {valid_count}/{total_points} valid points ({valid_count/total_points*100:.1f}%)")
        print(f"Grid cells: {len(self.grid.grid)}")
        print(f"Frontier cells: {len(self.frontier_cells)}")
        print(f"Mapping failures: {self.mapping_failures}")
        print(f"Recovery attempts: {self.recovery_attempts}")
        print("=======================\n")

    def detect_loop(self) -> bool:
        """Detect if robot is stuck in a loop."""
        if len(self.position_history) < 20:
            return False
            
        # Check position history for loops
        recent_positions = list(self.position_history)[-20:]
        center_x = sum(p[0] for p in recent_positions) / len(recent_positions)
        center_y = sum(p[1] for p in recent_positions) / len(recent_positions)
        
        # Calculate average distance from center
        avg_distance = sum(math.hypot(p[0] - center_x, p[1] - center_y) 
                          for p in recent_positions) / len(recent_positions)
        
        # Check orientation history for spinning
        recent_orientations = list(self.orientation_history)[-20:]
        orientation_changes = [abs(recent_orientations[i+1] - recent_orientations[i])
                             for i in range(len(recent_orientations)-1)]
        avg_orientation_change = sum(orientation_changes) / len(orientation_changes)
        
        return (avg_distance < 0.1 and  # Small movement radius
                avg_orientation_change > 0.1)  # Significant rotation

    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """Set motor speeds with smooth transitions."""
        # Clip speeds to maximum allowed
        left_speed = np.clip(left_speed, -self.MAX_SPEED, self.MAX_SPEED)
        right_speed = np.clip(right_speed, -self.MAX_SPEED, self.MAX_SPEED)
        
        # Smooth transitions
        max_change = 0.5  # Maximum speed change per timestep
        left_diff = left_speed - self.current_left_speed
        right_diff = right_speed - self.current_right_speed
        
        # Clip the speed changes
        left_diff = np.clip(left_diff, -max_change, max_change)
        right_diff = np.clip(right_diff, -max_change, max_change)
        
        # Update current speeds
        self.current_left_speed += left_diff
        self.current_right_speed += right_diff
        
        # Ensure minimum speed when moving
        if abs(left_speed) > 0.1 or abs(right_speed) > 0.1:
            self.current_left_speed = max(abs(self.current_left_speed), self.MIN_SPEED) * np.sign(self.current_left_speed)
            self.current_right_speed = max(abs(self.current_right_speed), self.MIN_SPEED) * np.sign(self.current_right_speed)
        
        # Set motor velocities
        self.left_motor.setVelocity(self.current_left_speed)
        self.right_motor.setVelocity(self.current_right_speed)
        
        # Debug output
        print(f"Motor speeds: L={self.current_left_speed:.2f} R={self.current_right_speed:.2f}")

    def force_state_change(self):
        """Force a state change when stuck."""
        self.last_state_change_time = self.robot.getTime() * 1000
        
        if self.current_state == 'MAPPING':
            self.mapping_failures += 1
            if self.mapping_failures >= self.MAX_MAPPING_FAILURES:
                print("Too many mapping failures, attempting escape")
                self.current_state = 'RECOVERY'
                self.escape_rotation_direction = 1
                self.escape_rotation_time = self.robot.getTime() * 1000
            else:
                self.current_state = 'PLANNING'
        elif self.current_state == 'PLANNING':
            self.current_state = 'RECOVERY'
        elif self.current_state == 'MOVING':
            self.current_state = 'RECOVERY'
        elif self.current_state == 'RECOVERY':
            self.current_state = 'MAPPING'
            self.mapping_failures = 0
            self.recovery_attempts = 0

    def is_stuck(self) -> bool:
        """Check if robot is stuck or spinning in place."""
        if len(self.position_history) < 5:
            return False
            
        # Calculate movement in last few steps
        recent_positions = list(self.position_history)[-5:]
        total_movement = 0
        for i in range(len(recent_positions)-1):
            dx = recent_positions[i+1][0] - recent_positions[i][0]
            dy = recent_positions[i+1][1] - recent_positions[i][1]
            total_movement += math.hypot(dx, dy)
            
        # Check if movement is too small
        if total_movement < self.MIN_MOVEMENT_THRESHOLD:
            self.stuck_counter += 1
            if self.stuck_counter >= self.MAX_STUCK_COUNT:
                print("Robot appears to be stuck (minimal movement detected)")
                return True
        else:
            self.stuck_counter = 0
            
        return False

    def handle_mapping(self):
        """Handle mapping state with forced exploration."""
        current_time = self.robot.getTime() * 1000
        
        # Check for loop or stuck condition
        if self.detect_loop() or self.is_stuck():
            print("Loop or stuck condition detected in mapping")
            if self.mapping_failures > 5 and len(self.frontier_cells) == 0:
                print("Forcing exploratory rotation due to mapping failures and no frontiers")
                self.exploratory_rotation_time = current_time
                self.current_state = 'EXPLORATORY_ROTATION'
                return
            self.force_state_change()
            return
        
        # Get valid LIDAR readings
        front_dist = self.get_sector_distances(*self.FRONT_SECTOR)
        front_min = min(front_dist) if front_dist else float('inf')
        
        # Base speed calculation with minimum distance enforcement
        base_speed = self.BASE_SPEED * 0.4
        if front_min < 0.15:  # If too close to obstacle
            base_speed *= 0.5  # Reduce speed
            turn_speed = base_speed * 0.5  # Add turning to move away
            self.set_motor_speeds(base_speed - turn_speed, base_speed + turn_speed)
            return
        
        # Normal mapping behavior
        if len(self.grid.grid) > 1000 and len(self.frontier_cells) == 0:
            print("Mapping complete, switching to planning")
            self.current_state = 'PLANNING'
            return
        
        # Move forward with exploration behavior
        turn_speed = base_speed * 0.1
        self.set_motor_speeds(base_speed - turn_speed, base_speed + turn_speed)

    def handle_planning(self):
        """Handle planning state with empty map check."""
        # Skip planning if map is too empty
        if len(self.grid.grid) < 10:  # Arbitrary small threshold
            print("Map too empty, returning to mapping")
            self.current_state = 'MAPPING'
            return
            
        if not self.frontier_cells:
            print("No frontier cells found, returning to mapping")
            self.current_state = 'MAPPING'
            return
        
        # Rest of the existing planning logic...
        start = (self.state.x, self.state.y)
        best_path = None
        best_distance = float('inf')
        
        for frontier in self.frontier_cells:
            world_x, world_y = self.grid.grid_to_world(*frontier)
            path = self.planner.find_path(start, (world_x, world_y))
            if path:
                distance = len(path)
                if distance < best_distance:
                    best_distance = distance
                    best_path = path
        
        if best_path:
            print(f"Found path to frontier with {len(best_path)} points")
            self.current_path = best_path
            self.path_index = 0
            self.current_state = 'MOVING'
        else:
            print("No path found to frontiers, attempting recovery")
            self.current_state = 'RECOVERY'

    def handle_moving(self):
        """Handle the moving state with improved obstacle avoidance."""
        if not self.current_path or self.path_index >= len(self.current_path):
            print("Path complete, switching to planning")
            self.current_state = 'PLANNING'
            return
        
        # Get current target
        target = self.current_path[self.path_index]
        
        # Get distances for safety check
        front_dist = self.get_sector_distances(*self.FRONT_SECTOR)
        front_min = min(front_dist) if front_dist else float('inf')
        
        # Calculate distance and angle to target
        dx = target[0] - self.state.x
        dy = target[1] - self.state.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.state.theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Base speed calculation
        base_speed = self.BASE_SPEED
        
        # Obstacle avoidance
        if front_min < self.OBSTACLE_AVOIDANCE_DISTANCE:
            # Get obstacle avoidance direction
            turn_dir, turn_strength = self.get_obstacle_avoidance_direction()
            
            # Adjust speed based on obstacle distance
            speed_factor = (front_min - self.EMERGENCY_DISTANCE) / (self.OBSTACLE_AVOIDANCE_DISTANCE - self.EMERGENCY_DISTANCE)
            speed_factor = max(0.3, min(1.0, speed_factor))  # Keep minimum speed
            base_speed *= speed_factor
            
            # Apply obstacle avoidance turning
            turn_correction = turn_dir * turn_strength
            self.set_motor_speeds(
                base_speed * (1 - turn_correction),
                base_speed * (1 + turn_correction)
            )
            print(f"Obstacle avoidance: turning {'right' if turn_dir > 0 else 'left'} with strength {turn_strength:.2f}")
            return
        
        # Normal path following
        if abs(angle_diff) > 0.1:
            # Smooth turning with proportional control
            turn_speed = np.clip(angle_diff * 0.5, -0.5, 0.5)
            self.set_motor_speeds(-turn_speed, turn_speed)
            return
        
        # Move towards target with slight turning correction
        turn_correction = np.clip(angle_diff * 0.3, -0.2, 0.2)
        self.set_motor_speeds(
            base_speed * (1 - turn_correction),
            base_speed * (1 + turn_correction)
        )
        
        # Check if reached target
        if distance < self.grid.resolution * 0.5:
            self.grid.mark_visited(target[0], target[1])
            self.path_index += 1
    
    def handle_recovery(self):
        """Handle recovery state with improved reset logic."""
        current_time = self.robot.getTime() * 1000
        
        # Check if we're in escape rotation
        if current_time - self.escape_rotation_time < self.ESCAPE_ROTATION_DURATION:
            rotation_speed = self.recovery_rotation_speed * (1 + self.recovery_attempts * 0.2)
            self.set_motor_speeds(-rotation_speed, rotation_speed)
            return
        
        # Check if we have valid LIDAR data
        valid_ranges = self.get_valid_ranges(self.state.lidar_scan)
        if len(valid_ranges) >= self.MIN_VALID_POINTS:
            print("Recovery successful, valid LIDAR data restored")
            self.current_state = 'PLANNING'
            self.recovery_attempts = 0
            return
        
        # Try different recovery strategies
        self.recovery_attempts += 1
        if self.recovery_attempts >= self.MAX_RECOVERY_ATTEMPTS:
            print("Max recovery attempts reached, performing full reset")
            # Full reset of exploration data
            self.frontier_cells.clear()
            self.current_path = []
            self.mapping_failures = 0
            self.recovery_attempts = 0
            self.visited_cells.clear()
            self.position_history.clear()
            self.orientation_history.clear()
            self.initial_scan_done = False
            self.initial_cells_marked = False  # Reset initial cell marking
            self.current_state = 'MAPPING'
            # Mark initial cells after reset
            self.mark_initial_cells()
            return
        
        # Rest of the existing recovery logic...
        if self.recovery_attempts % 2 == 0:
            backup_speed = self.BASE_SPEED * 0.3
            turn_speed = backup_speed * 0.5 * self.escape_rotation_direction
            self.set_motor_speeds(-backup_speed - turn_speed, -backup_speed + turn_speed)
        else:
            self.escape_rotation_direction *= -1
            self.escape_rotation_time = current_time
            rotation_speed = self.recovery_rotation_speed * (1 + self.recovery_attempts * 0.2)
            self.set_motor_speeds(-rotation_speed, rotation_speed)

    def mark_initial_cells(self):
        """Mark initial cells around robot as visited to seed the map."""
        if self.initial_cells_marked:
            return
            
        print("Marking initial cells around robot position...")
        current_x, current_y = self.grid.world_to_grid(self.state.x, self.state.y)
        
        # Mark a 3x3 grid around the robot
        for dx in range(-self.INITIAL_GRID_SIZE, self.INITIAL_GRID_SIZE + 1):
            for dy in range(-self.INITIAL_GRID_SIZE, self.INITIAL_GRID_SIZE + 1):
                grid_x, grid_y = current_x + dx, current_y + dy
                
                # Create cell if it doesn't exist
                if (grid_x, grid_y) not in self.grid.grid:
                    self.grid.grid[(grid_x, grid_y)] = GridCell(grid_x, grid_y)
                
                # Mark as visited and not obstacle
                cell = self.grid.grid[(grid_x, grid_y)]
                cell.is_visited = True
                cell.is_obstacle = False
                cell.confidence = 0.0
                self.visited_cells.add((grid_x, grid_y))
        
        self.initial_cells_marked = True
        print(f"Marked {self.INITIAL_GRID_SIZE*2+1}x{self.INITIAL_GRID_SIZE*2+1} initial cells")
        self.update_frontiers()  # Update frontiers after marking initial cells

    def handle_exploratory_rotation(self):
        """Handle forced exploratory rotation state."""
        current_time = self.robot.getTime() * 1000
        
        if current_time - self.exploratory_rotation_time < self.EXPLORATORY_ROTATION_DURATION:
            # Rotate in place while scanning
            self.set_motor_speeds(-self.exploratory_rotation_speed, self.exploratory_rotation_speed)
            return
            
        # After rotation, update frontiers and return to mapping
        self.update_frontiers()
        print(f"Exploratory rotation complete, found {len(self.frontier_cells)} frontiers")
        self.current_state = 'MAPPING'
        self.mapping_failures = 0  # Reset mapping failures after exploration

    def run(self):
        """Main control loop with exploratory rotation state."""
        while self.robot.step(self.timestep) != -1:
            # Perform initial scan if needed
            if not self.perform_initial_scan():
                continue
            
            # Mark initial cells if not done
            if not self.initial_cells_marked:
                self.mark_initial_cells()
            
            # Update robot state
            self.update_robot_state()
            
            # Print debug information
            self.print_debug_info()
            
            # State machine
            if self.current_state == 'MAPPING':
                self.handle_mapping()
            elif self.current_state == 'PLANNING':
                self.handle_planning()
            elif self.current_state == 'MOVING':
                self.handle_moving()
            elif self.current_state == 'RECOVERY':
                self.handle_recovery()
            elif self.current_state == 'EXPLORATORY_ROTATION':
                self.handle_exploratory_rotation()

def main():
    controller = IntelligentMazeSolver()
    controller.run()

if __name__ == "__main__":
    main()