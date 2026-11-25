class LidarSensor:
    def __init__(self,robot,step_ms):
        self.lidar = robot.getDevice("front_lidar")
        self.lidar.enable(step_ms)

    def get_scan(self):
        return self.lidar.getRangeImage()