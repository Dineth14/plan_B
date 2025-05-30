class WebotsLidarAdapter:
    def __init__(self, lidar_device, config):
        self.lidar_device = lidar_device
        self.config = config
        self.lidar_device.enable(config.LIDAR_ENABLE_TIME)

    def get_scan(self):
        return self.lidar_device.getRangeImage()

    def get_distances(self):
        return self.get_scan()

    def get_angle(self):
        return self.lidar_device.getFov() / 2  # Field of view divided by 2 for angle calculation

    def get_resolution(self):
        return self.lidar_device.getHorizontalResolution()  # Returns the number of beams in the LIDAR sensor