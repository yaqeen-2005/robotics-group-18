"""Visual servo for paired Webots station markers, in camera x-forward / y-left axes."""
import math


def dock_command(camera, station):
    marks = {}
    for obj in camera.getRecognitionObjects():
        label = obj.getModel()
        if isinstance(label, bytes):
            label = label.decode()
        if label in (station+"_L", station+"_R"):
            marks[label[-1]] = obj.getPosition()
    if len(marks) != 2:
        return 0., .35, False
    left, right = marks["L"], marks["R"]
    dx, dy = left[0]-right[0], left[1]-right[1]
    yaw = math.atan2(-dx, dy)
    x = (left[0]+right[0])/2 - .70*math.cos(yaw)
    y = (left[1]+right[1])/2 - .70*math.sin(yaw)
    dist = math.hypot(x,y)
    if dist < .022:
        if abs(yaw)<.028:
            return 0.,0.,True
        return 0.,max(-.28,min(.28,1.8*yaw)),False
    # Follow the centreline ahead of the chassis before entering the narrow rails.
    along = x*math.cos(yaw) + y*math.sin(yaw)
    lead = max(0., along-.65)
    bearing = math.atan2(y-lead*math.sin(yaw),x-lead*math.cos(yaw))
    speed = min(.30,.8*dist)*max(0.,math.cos(bearing)) if abs(bearing)<.45 else 0.
    spin = max(-.38,min(.38,2.8*bearing-.7*yaw))
    return speed,spin,False
