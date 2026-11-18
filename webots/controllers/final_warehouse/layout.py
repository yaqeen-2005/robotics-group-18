"""Shared geometry for the hall, traffic, world builder and offline evaluation."""
import json
import math
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CFG = json.loads((ROOT / 'config/final_warehouse.json').read_text())
PROFILES = {load['id']: load for load in json.loads((ROOT / 'config/integration_scenario.json').read_text())['loads']}
EMPTY_RADIUS = math.hypot(1.08, .92) / 2
LOAD_RADIUS = max(math.hypot(*load['deck_size'][:2]) / 2 for load in PROFILES.values()) + .05
TRUCK_RADIUS = 2.15
PERSON_RADIUS = .36
FORK_OFFSET = 1.40
SEAT_Z = .52
SLATS, SLAT_HEIGHT, GATE_COIL = 6, .64, 4.04
GATE_HEAD_HEIGHT = .67
WALL_HEIGHT = GATE_COIL + GATE_HEAD_HEIGHT/2
WALL_WIDTH = .32


def wet_tyres(position, orientation, sites):
    """Evaluate each wheel contact in the puddle ellipse, independently of reports."""
    result=[]
    for lateral in (.41,-.41):
        x=position[0]+lateral*orientation[1]; y=position[1]+lateral*orientation[4]
        result.append(any(((x-s['center'][0])/(s['size'][0]*.43))**2+
                          ((y-s['center'][1])/(s['size'][1]*.43))**2<=1 for s in sites))
    return tuple(result)


def view_axis(eye, target):
    dx,dy,dz=(b-a for a,b in zip(eye,target))
    heading=math.atan2(dy,dx)/2; pitch=math.atan2(-dz,math.hypot(dx,dy))/2
    half_cos=math.cos(heading)*math.cos(pitch); norm=math.sqrt(max(1e-15,1-half_cos*half_cos))
    return (-math.sin(heading)*math.sin(pitch)/norm,math.cos(heading)*math.sin(pitch)/norm,
            math.sin(heading)*math.cos(pitch)/norm,2*math.acos(half_cos))


def camera_view(mode, robot=None, dock=None):
    if mode==0: eye,target=(0,-50,105),(0,0,0)
    elif mode==1: eye,target=(-30.2,-14.8,15.4),(-41.9,-13,1.0)
    elif mode==2: eye,target=(robot[0],robot[1]-7,10),(*robot[:2],.3)
    else:
        x,y=dock['at']; eye,target=(x+dock['entry']*9.5,y-5.5,6.6),(x,y,.65)
    return eye,view_axis(eye,target)


@dataclass(frozen=True)
class Fixture:
    name: str
    kind: str
    x: float
    y: float
    length: float
    width: float
    height: float
    angle: float = 0.0

    def local(self, point):
        dx, dy = point[0] - self.x, point[1] - self.y
        cosine, sine = math.cos(self.angle), math.sin(self.angle)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def distance(self, point):
        dx, dy = self.local(point)
        return math.hypot(max(0, abs(dx) - self.length / 2), max(0, abs(dy) - self.width / 2))

    def intersects(self, start, end, margin):
        """Slab test, with square (conservative) corners on the expanded box."""
        if (max(start[0], end[0]) < self.x - self.bound - margin or min(start[0], end[0]) > self.x + self.bound + margin or
                max(start[1], end[1]) < self.y - self.bound - margin or min(start[1], end[1]) > self.y + self.bound + margin):
            return False
        near, far = 0., 1.
        for origin, target, half in zip(self.local(start), self.local(end), (self.length / 2 + margin, self.width / 2 + margin)):
            delta = target - origin
            if abs(delta) < 1e-10:
                if abs(origin) > half:
                    return False
            else:
                first, last = sorted(((-half - origin) / delta, (half - origin) / delta))
                near, far = max(first, near), min(last, far)
                if near > far:
                    return False
        return True

    @property
    def bound(self):
        return (self.length + self.width) / 2


def stations():
    feed = CFG['dispatch']
    return [dict(id=f'P{index:02}', name='AS/RS outfeed', at=[feed['x'], y], entry=1)
            for index, y in enumerate(feed['lanes_y'], 1)] + CFG['docks']


def approach(station):
    return station['at'][0] + station['entry'] * 3.1, station['at'][1]


def gates():
    return [dict(y=0, **item) for item in CFG['crossings'] if item['mode'] != 'open'] + [CFG['stock_gate']]


def gate_rect(gate):
    return Fixture(gate['id'], 'shutter', gate['x'], gate['y'], gate['w'], .24, 3.5)


def wall_panels(spec):
    doors = spec.get('doors', [])
    if 'gate' in spec:
        gate = next(g for g in gates() if g['id'] == spec['gate'])
        doors = [(gate['x'] - gate['w']/2, gate['x'] + gate['w']/2)]
    start, finish = spec['span']
    for index, (end, after) in enumerate([*doors, (finish, finish)]):
        if end > start:
            middle, length = (start + end)/2, end - start
            dims = (middle, spec['at'], length, WALL_WIDTH) if spec['axis']=='x' else (spec['at'], middle, WALL_WIDTH, length)
            yield Fixture(f"{spec['id']}_{index}", 'partition', *dims, WALL_HEIGHT)
        start = after


def perimeter():
    hx,hy=(v/2 for v in CFG['hall'])
    for side in ('west','east','north','south'):
        vertical=side in ('west','east')
        extent=hy if vertical else hx
        outside=(-1 if side in ('west','south') else 1)*((hx if vertical else hy)+.12)
        points=[(-extent-.12,outside)]
        for _,a,b,depth in sorted((bay for bay in CFG['edge_bays'] if bay[0]==side),key=lambda bay:bay[1]):
            inside=outside-math.copysign(depth,outside)
            points.extend([(a,outside),(a,inside),(b,inside),(b,outside)])
        points.append((extent+.12,outside))
        for index,((a,b),(c,d)) in enumerate(zip(points,points[1:])):
            x,y=(a+c)/2,(b+d)/2
            dx,dy=abs(c-a)+.24,abs(d-b)+.24
            yield Fixture(f'outer_{side}_{index}','wall',y if vertical else x,x if vertical else y,
                          dy if vertical else dx,dx if vertical else dy,4.4)


def fixtures():
    items = []

    def add(name, kind, x, y, length, width, height, degrees=0):
        items.append(Fixture(name, kind, x, y, length, width, height, math.radians(degrees)))

    half_x, half_y = [value / 2 for value in CFG['hall']]
    items.extend(perimeter())
    left = -half_x
    for aperture in CFG['crossings'] + [dict(id='end', x=half_x, w=0)]:
        right = aperture['x'] - aperture['w'] / 2
        add('bulkhead_' + aperture['id'], 'barrier', (left + right) / 2, 0, right - left, WALL_WIDTH, WALL_HEIGHT)
        left = aperture['x'] + aperture['w'] / 2
    for spec in CFG['partitions']:
        items.extend(wall_panels(spec))
    for item in gates():
        for side in (-1, 1):
            add(f"{item['id']}_post{side}", 'gate_post', item['x'] + side * (item['w'] / 2 + .17), item['y'], .32, .54, WALL_HEIGHT)
    feed = CFG['dispatch']
    left = min(feed['lanes_y']) - 2.5
    upper = max(feed['lanes_y']) + 2.5
    for index, (end, start) in enumerate([(y - 1.32, y + 1.32) for y in feed['lanes_y']] + [(upper, upper)]):
        add(f'outfeed_panel_{index}', 'facade', feed['wall_x'], (left + end) / 2, .30, end - left, 4.1)
        left = start
    for side in (min(feed['lanes_y']) - 2.5, upper):
        add(f'machine_end_{side}', 'facade', (-half_x + feed['wall_x']) / 2, side, half_x + feed['wall_x'], .26, 4.1)
    for station in stations():
        x, y = station['at']
        for sign in (-1, 1):
            add(f"{station['id']}_rail{sign}", 'rail', x, y + sign * .58, 2.3, .12, .48)
        if station['id'].startswith('P'):
            end = x - 1.15
            start = feed['source_x'] - .95
            add(station['id'] + '_belt', 'conveyor', (start + end) / 2, y, end - start, 1.62, .48)
        else:
            for sign in (-1,1):
                add(f"{station['id']}_jamb{sign}", 'dock_post', x-station['entry']*.86, y+sign*1.25, .15,.15,2.90)
    for row, y in enumerate((-23, -16, -9, 9, 16, 23)):
        for col, x in enumerate((-27, -16, -5, 6, 17)):
            if (row, col) in {(1, 0), (2, 3), (3, 4), (4, 1), (5, 2),
                              (0,0), (0,3), (1,4), (2,1), (3,1), (4,2), (5,1)}:
                continue
            add(f'R{row + 1}{col + 1}', 'rack', x, y, 6.6, 2, 3.8)
    for ident, args in enumerate([(-28.3,-15.6,3.9,1.55,-17),(7.3,-9.8,3.6,1.65,23),(-16.8,16.8,3.8,1.55,14),
                                  (-4.9,23.5,4,1.55,-21),(-39,-25.5,4.2,1.6,90),(17,9.5,3,1.7,-18)]):
        x, y, length, width, angle = args
        add(f'S{ident + 1:02}', 'island', x,y,length,width,2.65,angle)
    for index, (x,y) in enumerate([(-32,-23),(-10,-16),(12,-9),(-32,23),(-10,9),(12,16),(33,30),(10,-30)]):
        add(f'C{index + 1:02}', 'column', x,y,.66,.66,4.85)
    for index, (x,y,angle) in enumerate([(-43.5,30.8,0),(-39.7,30.8,0),(-46.7,20.0,90),
                                          (17.5,31.0,0),(-9,-30.9,180),(46.8,-28,-90),(46.8,23.5,-90)]):
        add(f'WS{index + 1:02}', 'bench', x,y,2.75,1.1,1.52,angle)
        rad=math.radians(angle)
        add(f'WS{index + 1:02}_seat','chair',x+.53*math.cos(rad)+1.12*math.sin(rad),y+.53*math.sin(rad)-1.12*math.cos(rad),.57,.62,.91,angle)
    for index,(x,y,angle) in enumerate([(-42.7,7,8),(-32,3,-12),(12,-24,18),(-3,-29,4),(41,-24,-8),(-37.2,20.1,12)]):
        add(f'CR{index + 1:02}','crate',x,y,1.5,1.1,1.22,angle)
    for index,(x,y,length,width,high,angle) in enumerate([
        (-30.3,30,2.9,1.35,1.5,0),(-26.7,30.2,2.5,1.2,.96,4),(-16.6,22.9,2.4,1.4,1.55,5),
        (-14,23.1,1.7,1.15,1.18,0),(-5.8,28.8,2.4,1.3,1.35,0),(-3,29.2,1.65,1.15,.96,-7),
        (3.9,22.1,1.35,1.05,1.25,-4),(15.1,22.1,2.15,1.25,1.35,5),(9.5,30.9,1.5,1.1,1.06,0)]):
        add(f'BUFFER_{index+1:02}','buffer',x,y,length,width,high,angle)
    for fleet in CFG['trucks']:
        for index, y in enumerate(fleet['ends']):
            x=fleet['dock_x'] + FORK_OFFSET
            for side in (-1,1):
                add(f"{fleet['id']}_stock_{index}_{side}", 'stock', x, y + side * .56, 1.32, .1, .48)
                for end in (-1,1): add(f"{fleet['id']}_stock_{index}_post{side}_{end}", 'stock_post',x+end*.72,y+side*.88,.09,.09,2.5)
            add(f"{fleet['id']}_stock_{index}_back", 'stock_back',x+.87,y,.12,1.86,2.48)
    return items


def segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    along = min(1., max(0., ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / max(1.e-12, dx * dx + dy * dy)))
    return math.hypot(point[0] - start[0] - along * dx, point[1] - start[1] - along * dy)
