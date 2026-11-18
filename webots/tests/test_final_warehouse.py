"""Geometry, traffic and facility invariants for the production map."""
from collections import deque
import math
import re
import struct
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controllers/final_warehouse')); sys.path.insert(0,str(ROOT/'tools'))
from layout import CFG, EMPTY_RADIUS, FORK_OFFSET, LOAD_RADIUS, PERSON_RADIUS, PROFILES, SEAT_Z, TRUCK_RADIUS, WALL_HEIGHT, Fixture, approach, fixtures, gate_rect, gates, stations
from operations import Dispatch, Gate, Spills
from layout import wet_tyres
from traffic import Truck, make_traffic
from build_final_world import build


class HallTests(unittest.TestCase):
    def test_worst_case_loaded_access(self):
        step=.4; margin=LOAD_RADIUS+step/math.sqrt(2)
        blocked=fixtures()+[gate_rect(gate) for gate in gates()]+[Fixture(s['id'],'spill',*s['center'],*s['size'],0) for s in CFG['spills']]
        occupied=set(); half_x,half_y=(round(size/(2*step)) for size in CFG['hall'])
        for item in blocked:
            co,si=abs(math.cos(item.angle)),abs(math.sin(item.angle))
            padx=(item.length*co+item.width*si)/2+margin; pady=(item.length*si+item.width*co)/2+margin
            for ix in range(max(-half_x,math.floor((item.x-padx)/step)),min(half_x,math.ceil((item.x+padx)/step))+1):
                for iy in range(max(-half_y,math.floor((item.y-pady)/step)),min(half_y,math.ceil((item.y+pady)/step))+1):
                    if item.distance((ix*step,iy*step))<=margin: occupied.add((ix,iy))
        center=lambda point: tuple(round(v/step) for v in point)
        origin=center(CFG['parking']); self.assertNotIn(origin,occupied)
        seen={origin}; queue=deque([origin])
        while queue:
            ix,iy=queue.popleft()
            for cell in ((ix+1,iy),(ix-1,iy),(ix,iy+1),(ix,iy-1)):
                if abs(cell[0])<=half_x and abs(cell[1])<=half_y and cell not in occupied and cell not in seen:
                    seen.add(cell); queue.append(cell)
        for station in stations(): self.assertTrue(center(approach(station)) in seen,station['id'])
        for point in [(-37,0),(42,0),(-42,24),(4,20),(-19,-19),(24,13),(30,-18),(4,5),(18,-4)]:
            self.assertTrue(center(point) in seen,point)
        stock=CFG['stock_gate']; low=(stock['x'],stock['y']-4); high=(stock['x'],stock['y']+4)
        source,target=center(low),center(high); distances={source:0}; queue=deque([source])
        while queue and target not in distances:
            ix,iy=cell=queue.popleft()
            for neighbour in ((ix+1,iy),(ix-1,iy),(ix,iy+1),(ix,iy-1)):
                if neighbour in seen and neighbour not in distances:
                    distances[neighbour]=distances[cell]+1; queue.append(neighbour)
        self.assertGreater(distances[target]*step,4*math.dist(low,high))
        self.assertFalse(any(f.intersects(low,high,LOAD_RADIUS) for f in fixtures()))

    def test_partition_heights_and_spill_sites(self):
        static=fixtures()
        for gate in gates():
            for side in (-1,1):
                jamb=(gate['x']+side*(gate['w']/2+.42),gate['y'])
                self.assertTrue(any(f.kind in ('partition','barrier') and f.distance(jamb)<.11 and f.height==WALL_HEIGHT for f in static),gate['id'])
        for site in CFG['spills']:
            x,y=site['center']; sx,sy=site['size']
            corners=[(x-sx/2,y-sy/2),(x+sx/2,y-sy/2),(x+sx/2,y+sy/2),(x-sx/2,y+sy/2)]
            for item in static:
                within=abs(item.x-x)<=sx/2 and abs(item.y-y)<=sy/2
                self.assertFalse(within or any(item.intersects(a,b,0) for a,b in zip(corners,corners[1:]+corners[:1])),(site['id'],item.name))

    def test_shortcut_and_universal_berths(self):
        narrow=next(g for g in CFG['crossings'] if g['id']=='SHORT'); x=narrow['x']
        jambs=[r for r in fixtures() if r.kind=='barrier']
        self.assertFalse(any(r.intersects((x,-2),(x,2),EMPTY_RADIUS+.05) for r in jambs))
        self.assertTrue(any(r.intersects((x,-2),(x,2),LOAD_RADIUS) for r in jambs))
        for radius,expected in [(EMPTY_RADIUS+.05,['SHORT','CART','BULK']),(.85722,['CART','BULK']),(LOAD_RADIUS,['BULK'])]:
            usable=[g['id'] for g in CFG['crossings'] if g['id'] in ('SHORT','CART','BULK') and
                    not any(r.intersects((g['x'],-2),(g['x'],2),radius) for r in fixtures())]
            self.assertEqual(usable,expected)
        for station in stations():
            rails=[item for item in fixtures() if item.name.startswith(station['id']+'_rail')]
            inner=abs(rails[0].y-rails[1].y)-rails[0].width
            self.assertGreater(inner,.92); self.assertEqual({r.height for r in rails},{.48})
            for load in PROFILES.values():
                length,width,_=load['deck_size']; self.assertGreater(width,inner+.08); self.assertLess(length,rails[0].length)
                self.assertLess(width,2.35)

    def test_all_truck_routes_swept_and_disjoint(self):
        sectors=[]
        for spec in CFG['trucks']:
            xmin,xmax,ymin,ymax=spec['sector']; sectors.append((xmin,xmax,ymin,ymax))
            obstacles=[item for item in fixtures() if not item.name.startswith(spec['id']+'_stock')]
            for aisle in spec['aisles']:
                for go,back in (spec['ends'],list(reversed(spec['ends']))):
                    route=[(spec['dock_x'],go),(aisle,go),(aisle,back),(spec['dock_x'],back)]
                    for point in route:
                        self.assertTrue(xmin+TRUCK_RADIUS<=point[0]<=xmax-TRUCK_RADIUS and ymin+TRUCK_RADIUS<=point[1]<=ymax-TRUCK_RADIUS)
                    for start,end in zip(route,route[1:]):
                        self.assertFalse(any(item.intersects(start,end,TRUCK_RADIUS) for item in obstacles),(spec['id'],aisle,start))
        for index,first in enumerate(sectors):
            for second in sectors[index+1:]:
                self.assertTrue(first[1]<second[0] or first[0]>second[1] or first[3]<second[2] or first[2]>second[3])

    def test_traffic_yields_and_stock_continues(self):
        for seed in (18000,54,913):
            trucks,people=make_traffic(seed); gateset=[Gate(s,seed+i) for i,s in enumerate(gates())]; stop=(CFG['parking'],EMPTY_RADIUS)
            aisles={t.spec['id']:set() for t in trucks}; yields=0; travels={p.spec['id']:0. for p in people}
            ground_drops=set()
            for tick in range(4550):
                dt=.16; occupants=[stop]+[(p.pos,PERSON_RADIUS) for p in people]+[(t.pos,TRUCK_RADIUS) for t in trucks]
                for gate in gateset: gate.step((tick+1)*dt,dt,occupants)
                for truck in trucks:
                    truck.step(dt,stop,[p.pos for p in people],gateset)
                    if truck.phase=='floor_wait':
                        ground_drops.add(tuple(truck.cargo));self.assertAlmostEqual(truck.cargo[2],.14)
                self.assertGreater(math.dist(trucks[0].pos,trucks[1].pos),2*TRUCK_RADIUS)
                for truck in trucks:
                    if truck.aisle: aisles[truck.spec['id']].add(truck.aisle)
                    if truck.loaded: self.assertAlmostEqual(math.dist(truck.pos,truck.cargo[:2]),FORK_OFFSET)
                    else: self.assertEqual(truck.cargo,truck.stock_pose())
                for p in people:
                    old=p.pos; p.step(dt,stop,trucks,gateset,[q.pos for q in people if q!=p],[(t.cargo[:2],.85) for t in trucks]); travels[p.spec['id']]+=math.dist(old,p.pos)
                    yields+=int(p.waiting=='forklift'); xmin,xmax,ymin,ymax=p.spec['zone']
                    self.assertTrue(xmin<=p.pos[0]<=xmax and ymin<=p.pos[1]<=ymax)
                    for truck in trucks: self.assertGreater(math.dist(p.pos,truck.pos),TRUCK_RADIUS+PERSON_RADIUS-.001)
                if tick%20==0:
                    for p in people: self.assertTrue(p.clear(p.pos,p.pos),p.spec['id'])
            self.assertGreater(yields,0); self.assertTrue(all(dist>24 for dist in travels.values()),travels)
            for t in trucks:
                self.assertGreaterEqual(t.trips,3,(seed,t.spec['id'],t.pos,t.waiting)); self.assertGreaterEqual(t.floor_visits,1)
            self.assertGreaterEqual(len(ground_drops),3)
            if seed==18000:self.assertTrue(any(x<20 for x,y,z in ground_drops))

    def test_slip_uses_wheel_contacts_not_body_centre(self):
        site={'center':[0,0],'size':[2.4,1.6]}; identity=(1,0,0,0,1,0,0,0,1)
        self.assertEqual(wet_tyres((0,.5),identity,[site]),(False,True))
        self.assertEqual(wet_tyres((0,-.5),identity,[site]),(True,False))
        self.assertEqual(wet_tyres((0,0),identity,[]),(False,False))

    def test_gate_hold_and_obstruction_reversal(self):
        gate=Gate(next(spec for spec in gates() if spec['mode']=='presence'),11); pos=(gate.spec['x'],gate.spec['y']-2.9)
        for tick in range(1,801):
            now=tick*.1; gate.step(now,.1,[(pos,.72)] if now<4 or 40<now<48 else [])
            if 3<=now<=30: self.assertEqual(gate.phase,'open')
            if 36<=now<=40: self.assertEqual(gate.phase,'closed')
            if 43<=now<=70: self.assertEqual(gate.phase,'open')
        gate=Gate(next(spec for spec in gates() if spec['mode']=='timed'),91)
        closures=0; opened=0.; close_since=None
        for tick in range(1,8000):
            old=gate.phase; now=tick*.1; gate.step(now,.1,[])
            if gate.phase=='closing' and old=='open': self.assertGreaterEqual(now-opened,CFG['gate_policy']['min_open_s']); closures+=1
            if gate.phase=='closed' and old=='closing': close_since=now
            if gate.phase=='opening' and old=='closed': self.assertGreaterEqual(now-close_since,CFG['gate_policy']['min_closed_s'])
            if gate.phase=='open' and old=='opening': opened=now
        self.assertGreater(closures,3)
        gate.phase,gate.travel='closing',.55
        gate.step(801,.1,[((gate.spec['x'],gate.spec['y']),1.1)])
        self.assertEqual(gate.phase,'opening'); self.assertGreater(gate.travel,.55)

    def test_walker_stands_neutral_when_yielding_or_paused(self):
        _,people=make_traffic(29); p=people[0]; p.dwell=0
        p.target=(p.pos[0]+2,p.pos[1]); away=((100,100),EMPTY_RADIUS)
        for _ in range(5): p.step(.064,away,[],[],[])
        self.assertGreater(abs(p.leg_angle),.16); start=p.pos
        for _ in range(5): p.step(.064,((p.pos[0]+.7,p.pos[1]),EMPTY_RADIUS),[],[],[])
        self.assertEqual((p.pos,p.leg_angle),(start,0.))
        for _ in range(6): p.step(.064,away,[],[],[])
        self.assertGreater(abs(p.leg_angle),.16); start=p.pos
        for _ in range(5): p.step(.064,away,[],[],[],enabled=False)
        self.assertEqual((p.pos,p.leg_angle),(start,0.))

    def test_spills_and_manual_reporting(self):
        policy=CFG['spill_policy']; a,b=Spills(18),Spills(18); sizes=set(); changed_reports=False
        for tick in range(1,501):
            now=tick*policy['tick_s']; a.step(now); b.step(now); sizes.add(len(a.present)); changed_reports|=bool(a.reported)
            self.assertEqual((a.present,a.reported),(b.present,b.reported)); self.assertTrue(a.reported<=a.present)
            self.assertEqual({row['id'] for row in a.public()},a.reported)
        self.assertTrue(changed_reports); self.assertEqual((min(sizes),max(sizes)),(policy['min'],policy['max']))
        active=next(iter(a.present-a.cleaning.keys())); a.report(active,True); self.assertIn(active,a.reported)
        a.report(active,False); self.assertIn(active,a.present); self.assertNotIn(active,a.reported)
        with self.assertRaises(ValueError): a.report(next(iter(a.sites.keys()-a.present)),True)

    def test_only_reported_spills_fade_and_remain_forbidden_until_clean(self):
        state=Spills(17); state.policy=dict(state.policy,cleanup_chance=1.,report_chance=0.,add_chance=1.)
        for n in range(1,6): state.step(n*12)
        self.assertEqual(len(state.present),state.policy['max']); self.assertFalse(state.cleaning)
        site=sorted(state.present)[0]; state.report(site,True)
        self.assertIn(('cleaning',site),state.step(72))
        self.assertEqual(state.transparency(site,72),0.)
        self.assertAlmostEqual(state.transparency(site,73.5),.5)
        self.assertIn(site,{row['id'] for row in state.public()})
        self.assertEqual(len(state.present),state.policy['max'])
        with self.assertRaises(ValueError): state.report(site,False)
        self.assertIn(('cleaned',site),state.step(75,False))
        self.assertNotIn(site,state.present); self.assertFalse(state.public())
        state.present=set(sorted(state.sites)[:state.policy['min']]); state.reported=set(state.present)
        state.policy['add_chance']=0.
        for n in range(8,15): state.step(n*12)
        self.assertFalse(state.cleaning); self.assertEqual(len(state.present),state.policy['min'])

    def test_dispatch_checks_identity_height_pose_and_release(self):
        seen=set()
        for seed in range(40):
            state=Dispatch(seed); batch=[state.allocate(i,20,True) for i in range(3)]
            self.assertEqual(len({j.dock['id'] for j in batch}),3); seen.update(j.profile for j in batch)
        self.assertEqual(seen,set(PROFILES)); job=batch[0]
        self.assertIsNone(state.allocate(0,100,True)); x=job.x; state.advance_belt(job,2,False); self.assertEqual(x,job.x)
        state.advance_belt(job,30,True); self.assertEqual(job.phase,'ready'); self.assertEqual(job.x,job.pickup[0])
        pickup=job.pickup; state.observe(job,30,[pickup[0]+1,pickup[1],SEAT_Z],pickup,True,0,0); self.assertEqual(job.phase,'ready')
        state.observe(job,31,[*pickup,SEAT_Z+.2],pickup,True,0,0); self.assertEqual(job.phase,'in_transit')
        good=[*job.dock['at'],SEAT_Z]; wrong=[*batch[1].dock['at'],SEAT_Z]
        for pos,robot,flat,yaw,speed in [(wrong,(0,0),True,0,0),(good,good,True,0,0),(good,(0,0),False,0,0),
                                           (good,(0,0),True,.4,0),(good,(0,0),True,0,.2)]:
            state.observe(job,32,pos,robot,flat,yaw,speed); state.observe(job,35,pos,robot,flat,yaw,speed); self.assertEqual(job.phase,'in_transit')
        state.observe(job,36,good,(0,0),True,0,0); state.observe(job,38,good,(0,0),True,0,0); self.assertEqual(job.phase,'delivered')
        self.assertFalse(state.release(job,40,True)); self.assertEqual(job.transparency,0.)
        self.assertFalse(state.release(job,43.5,True)); self.assertAlmostEqual(job.transparency,.5)
        self.assertFalse(state.release(job,46,False)); self.assertAlmostEqual(job.transparency,.5)
        self.assertTrue(state.release(job,47.5,True))
        self.assertIsNone(state.allocate(0,52,False)); self.assertGreater(state.allocate(0,52,True).serial,job.serial)

    def test_checked_in_world_rebuilds_and_uses_local_assets(self):
        world=build(); self.assertEqual(world,(ROOT/'worlds/final_warehouse.wbt').read_text(encoding='utf-8'))
        for url in set(re.findall(r'"(\.\./[^"\n]+\.(?:proto|png))"',world)):
            path=ROOT/'worlds'/url; self.assertTrue(path.is_file(),url)
            if path.suffix=='.png':
                with path.open('rb') as data: data.seek(16); sizes=struct.unpack('>II',data.read(8))
                self.assertTrue(all(n>0 and n&(n-1)==0 for n in sizes),(url,sizes))
        self.assertNotIn('https://',world)


if __name__=='__main__': unittest.main()
