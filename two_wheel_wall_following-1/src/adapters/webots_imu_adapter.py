class WebotsIMUAdapter:
    def __init__(self, inertial_unit, accelerometer, gyro, config):
        self.inertial_unit = inertial_unit
        self.accelerometer = accelerometer
        self.gyro = gyro
        self.config = config
        
        self.inertial_unit.enable(config.TIME_STEP)
        self.accelerometer.enable(config.TIME_STEP)
        self.gyro.enable(config.TIME_STEP)

    def get_data(self):
        accel = self.accelerometer.getValues()
        gyro = self.gyro.getValues()
        orientation = self.inertial_unit.getRollPitchYaw()
        
        return {
            'accel': accel,
            'gyro': gyro,
            'orientation': orientation
        }