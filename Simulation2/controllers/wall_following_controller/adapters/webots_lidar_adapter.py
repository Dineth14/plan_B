import numpy as np
from controller import Lidar

class WebotsLidarAdapter:
    def __init__(self, lidar_device, config):
        if not isinstance(lidar_device, Lidar):
            raise TypeError("lidar_device must be a Webots Lidar device")
            
        self.lidar_device = lidar_device
        self.config = config
        
        # Configure LIDAR properties
        self.lidar_device.enable(config.LIDAR_ENABLE_TIME)
        self.lidar_device.enablePointCloud()  # Enable point cloud for better data
        
        # Get and verify sensor properties
        self.fov = self.lidar_device.getFov()
        self.resolution = self.lidar_device.getHorizontalResolution()
        self.max_range = self.lidar_device.getMaxRange()
        self.min_range = self.lidar_device.getMinRange()
        
        # Verify LIDAR is working
        if self.fov <= 0 or self.resolution <= 0 or self.max_range <= 0:
            raise ValueError("Invalid LIDAR properties detected")
            
        print(f"LIDAR initialized successfully:")
        print(f"  - FOV: {self.fov:.2f} rad")
        print(f"  - Resolution: {self.resolution} points")
        print(f"  - Range: [{self.min_range:.2f}, {self.max_range:.2f}] m")
        print(f"  - Update time: {config.LIDAR_ENABLE_TIME} ms")
        
        # Initialize last valid scan
        self.last_valid_scan = None
        self.scan_count = 0

    def get_scan(self):
        """Get the raw LIDAR scan data."""
        try:
            # Try to get range image
            ranges = self.lidar_device.getRangeImage()
            self.scan_count += 1
            
            # Verify we got valid data
            if ranges is None:
                print(f"Warning: LIDAR returned None (scan {self.scan_count})")
                return self.last_valid_scan if self.last_valid_scan is not None else np.zeros(self.resolution)
                
            if len(ranges) != self.resolution:
                print(f"Warning: LIDAR returned {len(ranges)} points, expected {self.resolution} (scan {self.scan_count})")
                return self.last_valid_scan if self.last_valid_scan is not None else np.zeros(self.resolution)
                
            # Convert to numpy array and check for invalid values
            ranges = np.array(ranges, dtype=np.float32)
            
            # Check for invalid values
            invalid_mask = np.isnan(ranges) | np.isinf(ranges) | (ranges < self.min_range) | (ranges > self.max_range)
            if np.any(invalid_mask):
                invalid_count = np.sum(invalid_mask)
                print(f"Warning: LIDAR scan {self.scan_count} contains {invalid_count} invalid values")
                # Replace invalid values with max range
                ranges[invalid_mask] = self.max_range
            
            # Check if we have any valid readings
            valid_count = np.sum(~invalid_mask)
            if valid_count > 0:
                self.last_valid_scan = ranges.copy()
                if self.scan_count % 10 == 0:  # Print stats every 10 scans
                    print(f"LIDAR scan {self.scan_count} - Valid points: {valid_count}/{self.resolution}")
                    print(f"  Min: {np.min(ranges[~invalid_mask]):.2f}m, Max: {np.max(ranges[~invalid_mask]):.2f}m")
            else:
                print(f"Warning: No valid readings in scan {self.scan_count}")
                return self.last_valid_scan if self.last_valid_scan is not None else np.zeros(self.resolution)
            
            return ranges
            
        except Exception as e:
            print(f"Error getting LIDAR scan {self.scan_count}: {e}")
            return self.last_valid_scan if self.last_valid_scan is not None else np.zeros(self.resolution)

    def get_distances(self):
        """Get processed distance measurements."""
        ranges = self.get_scan()
        
        # Filter out invalid readings
        valid_mask = (ranges > self.min_range) & (ranges < self.max_range)
        
        # Get point cloud data as backup for invalid readings
        try:
            point_cloud = self.lidar_device.getPointCloud()
            if point_cloud and len(point_cloud) > 0:
                # Convert point cloud to distances
                cloud_distances = np.array([np.sqrt(p.x**2 + p.y**2) for p in point_cloud])
                # Use point cloud data where range image is invalid
                ranges[~valid_mask] = np.minimum(ranges[~valid_mask], cloud_distances[~valid_mask])
        except Exception as e:
            print(f"Warning: Could not get point cloud data: {e}")
        
        return ranges

    def get_angle(self):
        """Get the angle corresponding to each measurement."""
        return np.linspace(-self.fov/2, self.fov/2, self.resolution)

    def get_resolution(self):
        """Get the number of beams in the LIDAR sensor."""
        return self.resolution

    def get_sector_distances(self, start_angle, end_angle):
        """Get distances in a specific sector of the LIDAR scan."""
        angles = self.get_angle()
        distances = self.get_distances()
        
        # Convert angles to indices
        start_idx = int((start_angle + self.fov/2) * self.resolution / self.fov)
        end_idx = int((end_angle + self.fov/2) * self.resolution / self.fov)
        
        # Ensure indices are within bounds
        start_idx = max(0, min(start_idx, self.resolution-1))
        end_idx = max(0, min(end_idx, self.resolution-1))
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
            
        sector = distances[start_idx:end_idx+1]
        valid_mask = (sector > self.min_range) & (sector < self.max_range)
        
        # Print sector info
        if np.any(valid_mask):
            valid_count = np.sum(valid_mask)
            min_dist = np.min(sector[valid_mask])
            max_dist = np.max(sector[valid_mask])
            print(f"Sector [{start_angle:.2f}, {end_angle:.2f}] rad:")
            print(f"  Valid points: {valid_count}/{len(sector)}")
            print(f"  Min: {min_dist:.2f}m, Max: {max_dist:.2f}m")
        else:
            print(f"Sector [{start_angle:.2f}, {end_angle:.2f}] rad: No valid readings")
        
        return sector