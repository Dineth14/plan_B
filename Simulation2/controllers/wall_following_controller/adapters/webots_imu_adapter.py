class WebotsIMUAdapter:
    def __init__(self, inertial_unit, accelerometer, gyro, config):
        self.inertial_unit = inertial_unit
        self.accelerometer = accelerometer
        self.gyro = gyro
        self.config = config
        
        self.inertial_unit.enable(config.TIMESTEP)
        self.accelerometer.enable(config.TIMESTEP)
        self.gyro.enable(config.TIMESTEP)

    def get_data(self):
        accel = self.accelerometer.getValues()
        gyro = self.gyro.getValues()
        orientation = self.inertial_unit.getRollPitchYaw()
        
        return {
            'accel': accel,
            'gyro': gyro,
            'orientation': orientation
        }