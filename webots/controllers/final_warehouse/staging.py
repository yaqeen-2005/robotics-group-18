"""Select temporary floor berths and reserve the loaded truck's swept route."""
from collections import deque
import math

from layout import CFG, FORK_OFFSET, LOAD_RADIUS, TRUCK_RADIUS, Fixture, approach, fixtures, gate_rect, gates, segment_distance, stations


def near_path(point,path,radius):
    return any(segment_distance(point,a,b)<radius for a,b in zip(path,path[1:])) or bool(path and math.dist(point,path[0])<radius)


class FloorTraffic:
    pitch=.4

    def __init__(self):
        self.obstacles=fixtures()
        self.grid=self.raster(self.obstacles+[gate_rect(g) for g in gates() if g['mode']!='presence'],TRUCK_RADIUS+.06)
        worst=self.obstacles+[gate_rect(g) for g in gates()]+[Fixture(s['id'],'spill',*s['center'],*s['size'],0) for s in CFG['spills']]
        self.walk_grid=self.raster(worst,LOAD_RADIUS)
        self.services=[(round(x/self.pitch),round(y/self.pitch)) for x,y in [CFG['parking'],*[approach(s) for s in stations()]]]
        self.reservations={}
        self.diversions={}
        self.parked={}
        self.aisles=[(s['dock_x'],y) for s in CFG['trucks'] for y in s['ends']]

    def raster(self, obstacles, radius):
        pitch=self.pitch; hx,hy=CFG['hall'][0]/2,CFG['hall'][1]/2
        free={(i,j) for i in range(math.ceil(-hx/pitch),math.ceil(hx/pitch)) for j in range(math.ceil(-hy/pitch),math.ceil(hy/pitch))}
        margin=radius+pitch/math.sqrt(2)
        for item in obstacles:
            c,s=abs(math.cos(item.angle)),abs(math.sin(item.angle))
            w=(item.length*c+item.width*s)/2+margin; h=(item.length*s+item.width*c)/2+margin
            for i in range(math.floor((item.x-w)/pitch),math.ceil((item.x+w)/pitch)+1):
                for j in range(math.floor((item.y-h)/pitch),math.ceil((item.y+h)/pitch)+1):
                    if item.distance((i*pitch,j*pitch))<=margin: free.discard((i,j))
        # Exterior recesses are separated by the physical perimeter.
        return free

    def xy(self,cell): return tuple(v*self.pitch for v in cell)

    def nearest(self,pos,grid): return min(grid,key=lambda c:math.dist(self.xy(c),pos))

    def reachable(self,start,grid):
        parent={start:None}; queue=deque([start])
        while queue:
            i,j=cell=queue.popleft()
            for nxt in ((i-1,j),(i+1,j),(i,j-1),(i,j+1)):
                if nxt in grid and nxt not in parent: parent[nxt]=cell; queue.append(nxt)
        return parent

    def service_routes(self,loads):
        free=self.walk_grid.copy()
        for load in loads:
            free.difference_update(self.disk(load,.85+LOAD_RADIUS+self.pitch/math.sqrt(2)))
        if self.services[0] not in free: return False
        seen=self.reachable(self.services[0],free)
        return all(s in seen for s in self.services)

    def disk(self,point,r):
        x,y=point
        return {(i,j) for i in range(math.floor((x-r)/self.pitch),math.ceil((x+r)/self.pitch)+1)
                for j in range(math.floor((y-r)/self.pitch),math.ceil((y+r)/self.pitch)+1)
                if math.dist(point,self.xy((i,j)))<=r}

    def clean_line(self,a,b):
        return not any(f.intersects(a,b,TRUCK_RADIUS+.055) for f in self.obstacles)

    def plan(self,truck,others,spills):
        ident=truck.spec['id']; home=truck.pos
        exit_at=(home[0]-3.6,home[1])
        # A truck backs out straight before the first turn near its shelf.
        free=self.grid.copy()
        for who,line in self.routes():
            if who==ident: continue
            free={cell for cell in free if not near_path(self.xy(cell),line,2*TRUCK_RADIUS+.55)}
        for p,r in others: free.difference_update(self.disk(p,r+TRUCK_RADIUS+.50))
        # Reported spills can close a cross-aisle for stock traffic too.
        for site in spills: free.difference_update(self.disk(site['center'],max(site['size'])/2+TRUCK_RADIUS+.45))
        if not free:return None
        start=self.nearest(exit_at,free)
        if math.dist(self.xy(start),exit_at)>1.35 or not self.clean_line(exit_at,self.xy(start)):return None
        parents=self.reachable(start,free); choices=list(parents); truck.rng.shuffle(choices)
        jobs=[s['at'] for s in stations()]+[approach(s) for s in stations()]+[CFG['parking']]
        for cell in sorted(choices[:100],key=lambda c: abs(self.xy(c)[0]-home[0])<12):
            x,y=self.xy(cell); target=(x+truck.rng.uniform(-.18,.18),y+truck.rng.uniform(-.18,.18))
            # Offset samples within the grid cell: the delivery coordinates aren't waypoints.
            if math.dist(target,exit_at)<6 or not self.clean_line((x,y),target):continue
            drop=(target[0]+FORK_OFFSET,target[1])
            if any(math.dist(drop,p)<4.6 for p in jobs+self.aisles):continue
            if any(abs(drop[0]-g['x'])<g['w']/2+2 and abs(drop[1]-g['y'])<3.4 for g in gates()+[dict(y=0,**g) for g in CFG['crossings'] if g['mode']=='open']):continue
            retreat=(target[0]-2.75,target[1])
            if not self.clean_line(target,retreat) or any(gate_rect(g).intersects(target,retreat,TRUCK_RADIUS) for g in gates()):continue
            seq=[]; cur=cell
            while cur is not None: seq.append(self.xy(cur));cur=parents[cur]
            points=[home,exit_at]
            for pos in reversed(seq):
                if len(points)>3 and ((points[-2][0]==points[-1][0]==pos[0]) or (points[-2][1]==points[-1][1]==pos[1])): points.pop()
                points.append(pos)
            points.append(target)
            # Reserve approach, load, withdrawal, and the identical reverse journey.
            sweep=points+[retreat]
            if any(near_path(p,sweep,TRUCK_RADIUS+r+.18) for p,r in others):continue
            if any(paths_conflict(sweep,line) for who,line in self.routes() if who!=ident):continue
            if not self.service_routes([*self.parked.values(),drop]):continue
            self.reservations[ident]=sweep; self.parked[ident]=drop
            return points,retreat
        return None

    def clear(self,ident):
        self.reservations.pop(ident,None); self.diversions.pop(ident,None); self.parked.pop(ident,None)

    def routes(self):
        yield from self.reservations.items()
        for ident,lines in self.diversions.items():
            for line in lines: yield ident,line


def paths_conflict(first,second):
    for a,b in zip(first,first[1:]):
        for c,d in zip(second,second[1:]):
            def cross(p,q,r): return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
            if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0: return True
            if min(segment_distance(a,c,d),segment_distance(b,c,d),segment_distance(c,a,b),segment_distance(d,a,b))<2*TRUCK_RADIUS+.25:return True
    return False
