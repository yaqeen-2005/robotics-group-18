"""Seeded warehouse traffic with shelf shuttles and reserved floor transfers."""
import math
import random
from staging import FloorTraffic, paths_conflict

from layout import CFG, FORK_OFFSET, PERSON_RADIUS, SEAT_Z, TRUCK_RADIUS, fixtures, gate_rect, segment_distance


def advance(start, end, length):
    distance = math.dist(start, end)
    if distance <= length:
        return tuple(end)
    return tuple(a + (b - a) * length / distance for a, b in zip(start, end))


def blocked_by_circle(start, end, position, clearance):
    return (math.dist(end, position) <= math.dist(start, position) + 1e-7 and
            segment_distance(position, start, end) < clearance)


def gate_blocks(start, end, radius, gates):
    return any(gate.travel < .995 and gate_rect(gate.spec).intersects(start, end, radius + .12) for gate in gates)


class Truck:
    def __init__(self, spec, seed, dispatcher=None):
        self.spec, self.rng = spec, random.Random(seed)
        self.end = 0
        self.pos = (spec['dock_x'], spec['ends'][0])
        self.yaw, self.forks = 0., 0.
        self.phase, self.timer = 'loading', 0.
        self.floor_path=[]
        self.cargo, self.cargo_yaw = self.stock_pose(), 0.
        self.loaded = False
        self.route = []
        self.aisle, self.trips, self.waiting = None, 0, ''
        self.dispatcher=dispatcher
        self.fleet=[]; self.reported=[]; self.next_plan=0.; self.clock=0.
        self.floor_visits=0
        self.foot_backoff=0.

    def stock_pose(self):
        if self.floor_path and self.phase in ('floor_lower','floor_withdraw','floor_wait','floor_approach','floor_raise'):
            x,y=self.floor_path[-1]; return (x+FORK_OFFSET,y,.14)
        return (self.spec['dock_x'] + FORK_OFFSET, self.spec['ends'][self.end], SEAT_Z)

    def leg(self, x, y, heading):
        return (x, y), heading

    def choose_route(self):
        self.aisle = self.rng.choice(self.spec['aisles'])
        home_y, away_y = self.spec['ends'][self.end], self.spec['ends'][1 - self.end]
        return [self.leg(self.aisle, home_y, 0.), self.leg(self.aisle, away_y, math.copysign(math.pi / 2, away_y - home_y)),
                self.leg(self.spec['dock_x'], away_y, 0.)]

    def prediction(self, seconds=4.):
        if not self.route:
            return self.pos, self.pos
        return self.pos, advance(self.pos, self.route[0][0], seconds * self.spec['speed'])

    def step(self, dt, robot, people, gates, freight=()):
        self.clock+=dt
        paused_for_person=self.waiting=='pedestrian'
        self.waiting = ''
        if self.phase in ('floor_lower','floor_wait','floor_raise'):
            self.timer+=dt
            if self.phase=='floor_lower':
                self.forks=max(-.38,self.forks-.23*dt)
                if self.forks<=-.38:
                    self.cargo=self.stock_pose(); self.loaded=False; self.floor_visits+=1
                    self.phase='floor_withdraw'; self.route=[(self.retreat,0.)]
            elif self.phase=='floor_wait' and self.timer>=self.floor_dwell:
                self.phase='floor_approach'; self.route=[(self.floor_path[-1],0.)]
            elif self.phase=='floor_raise':
                self.loaded=True; self.forks=min(.32,self.forks+.23*dt)
                if self.forks>=.32:
                    self.phase='floor_return'; self.route=self.floor_legs(list(reversed(self.floor_path)),False)
                    self.dispatcher.parked.pop(self.spec['id'],None)
        elif self.phase in ('loading', 'unloading', 'shelved'):
            self.timer += dt
            if self.phase == 'loading':
                self.loaded = True
                self.forks = min(.32, max(0., self.timer - 1.) * .28)
                if self.timer >= 2.8 and self.clock>=self.next_plan:
                    self.next_plan=self.clock+3.
                    others=[(t.pos,TRUCK_RADIUS) for t in self.fleet if t is not self]
                    if self.dispatcher and self.trips%2==1 and self.floor_visits<(self.trips+1)//2:
                        self.dispatcher.reservations.pop(self.spec['id'],None)
                        plan=self.dispatcher.plan(self,others+[robot]+list(freight),self.reported)
                        if plan:
                            self.floor_path,self.retreat=plan
                            self.floor_dwell=self.rng.uniform(12,24)
                            self.phase='floor_haul'; self.route=self.floor_legs(self.floor_path,True)
                    else:
                        route=self.choose_route()
                        full=[self.pos,*[p for p,_ in route]]
                        full.append((full[-1][0]-3.1,full[-1][1]))
                        if not self.dispatcher or not any(paths_conflict(full,path) for who,path in self.dispatcher.routes() if who!=self.spec['id']):
                            self.phase,self.route='haul',route
                            if self.dispatcher:self.dispatcher.reservations[self.spec['id']]=full
            elif self.phase == 'unloading':
                self.forks = max(0., .32 - self.timer * .22)
                if self.timer >= 2.5:
                    self.loaded, self.trips, self.cargo, self.cargo_yaw = False, self.trips + 1, self.stock_pose(), 0.
                    self.phase = 'withdraw'
                    self.route = [self.leg(self.spec['dock_x'] - 3.1, self.pos[1], 0.)]
            elif self.timer >= 6.:
                self.phase, self.route = 'approach', [self.leg(self.spec['dock_x'], self.pos[1], 0.)]
        elif self.route:
            target, heading = self.route[0]
            turn = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
            moving = abs(turn) < .002
            candidate = advance(self.pos, target, dt * self.spec['speed']) if moving else self.pos
            if paused_for_person and self.phase in ('floor_haul','floor_return') and self.route and self.foot_backoff>=3 and self.dispatcher:
                self.foot_backoff=0.
                self.detour(people)
                self.waiting='rerouting'
                return
            if blocked_by_circle(self.pos, candidate, robot[0], TRUCK_RADIUS + robot[1] + .20):
                self.waiting = 'robot'
            elif any(blocked_by_circle(self.pos, candidate, pos, TRUCK_RADIUS + PERSON_RADIUS + .08) for pos in people):
                self.waiting = 'pedestrian'
                self.foot_backoff+=dt
            elif any(blocked_by_circle(self.pos, candidate, pos, TRUCK_RADIUS + radius + .12) for pos, radius in freight):
                self.waiting = 'freight'
            elif any(blocked_by_circle(self.pos,candidate,t.pos,2*TRUCK_RADIUS+.15) for t in self.fleet if t is not self):
                self.waiting='forklift'
            elif gate_blocks(self.pos, candidate, TRUCK_RADIUS, gates):
                self.waiting = 'gate'
            elif not moving:
                self.yaw += max(-.95 * dt, min(.95 * dt, turn))
            else:
                self.foot_backoff=0.
                self.pos, self.yaw = candidate, heading
                if candidate == target:
                    self.route.pop(0)
                    if not self.route:
                        self.timer = 0.
                        if self.phase=='floor_haul':
                            self.phase='floor_lower'
                        elif self.phase=='floor_withdraw':
                            self.phase='floor_wait'
                        elif self.phase=='floor_approach':
                            self.phase='floor_raise'
                        elif self.phase=='floor_return':
                            self.dispatcher.clear(self.spec['id']); self.floor_path=[]
                            self.phase='unloading'
                        elif self.phase == 'haul':
                            self.end, self.phase = 1 - self.end, 'unloading'
                        else:
                            self.phase = 'shelved' if self.phase == 'withdraw' else 'loading'
        if self.loaded:
            self.cargo = (self.pos[0] + FORK_OFFSET * math.cos(self.yaw), self.pos[1] + FORK_OFFSET * math.sin(self.yaw), SEAT_Z + self.forks)
            self.cargo_yaw = self.yaw

    def floor_legs(self,points,outward):
        legs=[]
        for index,(a,b) in enumerate(zip(points,points[1:])):
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])
            legs.append((b,0. if (outward and index==0) or (not outward and index==len(points)-2) else yaw))
        if outward: legs.append((points[-1],0.))
        return legs

    def detour(self,people):
        # Rejoin the reserved path ahead of the obstruction; retain the return reservation.
        mgr=self.dispatcher; ident=self.spec['id']
        grid=mgr.grid.copy()
        for p in people: grid.difference_update(mgr.disk(p,TRUCK_RADIUS+PERSON_RADIUS+.22))
        for t in self.fleet:
            if t is not self: grid.difference_update(mgr.disk(t.pos,2*TRUCK_RADIUS+.28))
        if not grid: return
        start=mgr.nearest(self.pos,grid)
        if math.dist(mgr.xy(start),self.pos)>1.4:return
        parents=mgr.reachable(start,grid)
        for index,(goal,heading) in enumerate(self.route[:-1]):
            if math.dist(goal,self.pos)<3: continue
            dest=tuple(round(v/mgr.pitch) for v in goal)
            if dest not in parents: continue
            path=[]
            while dest is not None: path.append(mgr.xy(dest));dest=parents[dest]
            line=[self.pos]
            for pos in reversed(path):
                if len(line)>2 and (line[-2][0]==line[-1][0]==pos[0] or line[-2][1]==line[-1][1]==pos[1]): line.pop()
                line.append(pos)
            line.append(goal)
            if any(not mgr.clean_line(a,b) for a,b in zip(line,line[1:])):continue
            if any(paths_conflict(line,path) for who,path in mgr.routes() if who!=ident):continue
            self.route=[(b,math.atan2(b[1]-a[1],b[0]-a[0])) for a,b in zip(line,line[1:]) if a!=b]+self.route[index+1:]
            mgr.diversions.setdefault(ident,[]).append(line)
            return


class Walker:
    def __init__(self, spec, seed, obstacles):
        self.spec, self.rng = spec, random.Random(seed)
        self.pos, self.yaw = tuple(spec['start']), 0.
        self.target, self.dwell, self.stride, self.waiting = None, self.rng.uniform(.2, 2.), 0., ''
        self.leg_angle = 0.
        xmin, xmax, ymin, ymax = spec['zone']
        self.obstacles = [item for item in obstacles if xmin - item.bound - 1 <= item.x <= xmax + item.bound + 1 and
                          ymin - item.bound - 1 <= item.y <= ymax + item.bound + 1]

    def clear(self, start, end):
        return not any(item.intersects(start, end, PERSON_RADIUS + .12) for item in self.obstacles)

    def new_target(self):
        xmin, xmax, ymin, ymax = self.spec['zone']
        for _ in range(45):
            dest = self.rng.uniform(xmin, xmax), self.rng.uniform(ymin, ymax)
            if math.dist(self.pos, dest) > 1.3 and self.clear(self.pos, dest):
                return dest
        return None

    def step(self, dt, robot, trucks, gates, other_people, freight=(), enabled=True):
        previous = self.pos
        if enabled:
            self.move(dt, robot, trucks, gates, other_people, freight)
        distance = math.dist(previous, self.pos)
        if distance > 1e-6:
            self.stride += distance * 5
            target = .38 * math.sin(self.stride)
        else:
            target = 0.
        rate = 2.4 if distance > 1e-6 else 1.65
        self.leg_angle += max(-rate*dt, min(rate*dt, target-self.leg_angle))
        if distance <= 1e-6 and self.leg_angle == 0.: self.stride = 0.

    def move(self, dt, robot, trucks, gates, other_people, freight):
        self.waiting = ''
        if self.dwell > 0:
            self.dwell -= dt
            return
        if self.target is None:
            self.target = self.new_target()
            if self.target is None:
                self.dwell = .7
                return
        candidate = advance(self.pos, self.target, dt * self.spec['speed'])
        for truck in trucks:
            start, end = truck.prediction()
            clearance = TRUCK_RADIUS + PERSON_RADIUS + .45
            if segment_distance(candidate, start, end) >= clearance:
                continue
            self.waiting = 'forklift'
            if segment_distance(self.pos, start, end) < clearance - .015 and start != end:
                dx, dy = end[0] - start[0], end[1] - start[1]
                length = math.hypot(dx, dy)
                escapes = [(self.pos[0] - sign * dy / length * dt, self.pos[1] + sign * dx / length * dt) for sign in (-1, 1)]
                if truck.waiting=='pedestrian':
                    escapes.extend((self.pos[0]+dt*math.cos(i*math.pi/8),self.pos[1]+dt*math.sin(i*math.pi/8)) for i in range(16))
                xmin, xmax, ymin, ymax = self.spec['zone']
                valid = [pos for pos in escapes if xmin <= pos[0] <= xmax and ymin <= pos[1] <= ymax and self.clear(self.pos, pos)
                         and (segment_distance(pos, start, end) > segment_distance(self.pos, start, end) + .001
                              or (truck.waiting=='pedestrian' and math.dist(pos,start)>math.dist(self.pos,start)+dt*.45))
                         and math.dist(pos, start) >= math.dist(self.pos, start) - 1e-5]
                if valid:
                    candidate = max(valid, key=lambda pos: (segment_distance(pos, start, end),math.dist(pos,start)))
                    break
            return
        if blocked_by_circle(self.pos, candidate, robot[0], robot[1] + PERSON_RADIUS + .20):
            self.waiting = 'robot'
            return
        if any(blocked_by_circle(self.pos, candidate, pos, 2 * PERSON_RADIUS + .14) for pos in other_people):
            self.target = None
            return
        if any(blocked_by_circle(self.pos, candidate, pos, radius + PERSON_RADIUS + .14) for pos, radius in freight):
            self.target = None
            return
        if gate_blocks(self.pos, candidate, PERSON_RADIUS, gates):
            self.waiting = 'gate'
            return
        if not self.clear(self.pos,candidate):
            self.target = None
            return
        self.yaw = math.atan2(candidate[1] - self.pos[1], candidate[0] - self.pos[0])
        self.pos = candidate
        if candidate == self.target:
            self.target, self.dwell = None, self.rng.uniform(.8, 4.5)


def make_traffic(seed):
    obstacles = fixtures()
    manager=FloorTraffic()
    trucks=[Truck(spec,seed+200+index,manager) for index,spec in enumerate(CFG['trucks'])]
    for truck in trucks: truck.fleet=trucks
    return trucks,[Walker(spec, seed + 500 + index, obstacles) for index, spec in enumerate(CFG['people'])]
