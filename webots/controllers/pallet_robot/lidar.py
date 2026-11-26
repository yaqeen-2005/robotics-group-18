import math
class LidarSensor:
    def __init__(self,robot,step_ms):
        self.lidar = robot.getDevice("front_lidar")
        self.lidar.enable(step_ms)
        self.resolution = self.lidar.getHorizontalResolution()
        self.fov = self.lidar.getFov()
        self.angle_increment = self.fov / self.resolution


    def get_scan(self):
        return self.lidar.getRangeImage()
    
    def get_points(self, scan):
        points = []
        for i, distance in enumerate(scan):
            if not math.isfinite(distance):
                continue

            angle = self.fov / 2 - (i + 0.5) * self.angle_increment

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            points.append((x, y))
        
        return points