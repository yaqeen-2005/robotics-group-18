"""Run with: python -m unittest discover -s webots/tests -v"""
import collections
import json
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controllers/warehouse_manager'))
sys.path.insert(0,str(ROOT/'tools'))
from scenario import Scenario, actor_pose, near_robot
from build_integration_world import build


class ScenarioTests(unittest.TestCase):
    def test_proximity_stop_and_restart_clearance(self):
        # Person approaching the enlarged empty robot.
        self.assertFalse(near_robot([1.3,0],[1.3,0],.32,[0,0],.71,False))
        self.assertTrue(near_robot([1.3,0],[1.2,0],.32,[0,0],.71,False))
        self.assertTrue(near_robot([1.3,0],[1.3,0],.32,[0,0],.71,True))
        self.assertFalse(near_robot([1.4,0],[1.4,0],.32,[0,0],.71,True))
        # Forklift and nearby cargo require more clearance.
        self.assertTrue(near_robot([2.9,0],[2.8,0],1.55,[0,0],1.08,False))

    def setUp(self):
        self.config=json.loads((ROOT/'config/integration_scenario.json').read_text())

    def test_reports_do_not_disclose_unreported_spills(self):
        state=Scenario(self.config)
        self.assertEqual([s['id'] for s in state.reports()],['spill_2'])
        state.set_reported(0,True)
        self.assertEqual(len(state.reports()),2)
        state.set_present(0,False)
        self.assertEqual(len(state.reports()),1)
        with self.assertRaises(ValueError): state.set_reported(0,True)
        state.set_present(0,True)
        self.assertFalse(state.spills[0]['reported'])

    def test_random_ticks_repeat_across_step_sizes(self):
        a,b=Scenario(self.config),Scenario(self.config)
        a.random_enabled=b.random_enabled=True
        for _ in range(10000): a.advance(.016)
        for _ in range(160): b.advance(1)
        self.assertEqual(a.spills,b.spills)
        self.assertEqual(a.rng.getstate(),b.rng.getstate())
        c=Scenario(self.config)
        c.advance(100)
        self.assertEqual(c.spills,self.config['spills'])

    def test_traffic_motion_and_pause(self):
        state=Scenario(self.config)
        state.traffic_enabled=False
        state.advance(10)
        self.assertEqual(state.traffic_time,0)
        for actor in self.config['actors']:
            previous,_=actor_pose(actor,0)
            for i in range(1,3000):
                xy,_=actor_pose(actor,i*.016)
                self.assertLessEqual(math.dist(previous,xy),actor['speed']*.016+1e-8)
                previous=xy

    def test_clearance_routes_and_no_route(self):
        # Flood-fill free grid centres with an extra half-cell diagonal margin.
        # Checking full 4-connected edges via this margin prevents tunnelling
        # through thin walls. This is layout validation, not the team planner.
        _,obstacles=build(self.config)
        def reachable(radius, closed):
            rects=[(o['center'],o['size']) for o in obstacles]
            for x,width in closed: rects.append(([x,0],[width,.25]))
            spacing=.05; inflated=radius+spacing*math.sqrt(2)/2
            def free(ix,iy):
                x,y=ix*spacing,iy*spacing
                if abs(x)>12-inflated or abs(y)>9-inflated: return False
                for (cx,cy),(sx,sy) in rects:
                    dx=max(abs(x-cx)-sx/2,0); dy=max(abs(y-cy)-sy/2,0)
                    if dx*dx+dy*dy < inflated*inflated: return False
                return True
            start=(-120,-32); end=(-120,32)
            todo=collections.deque([start]); seen={start}
            while todo:
                cell=todo.popleft()
                if cell==end: return True
                for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nxt=(cell[0]+dx,cell[1]+dy)
                    if nxt not in seen and free(*nxt): seen.add(nxt); todo.append(nxt)
            return False
        # Radius includes the shared 5 cm planner margin.
        empty=math.hypot(1.08,.92)/2+.05
        wide=math.hypot(1.2,1.8)/2+.05
        self.assertTrue(reachable(wide,[]))
        self.assertTrue(reachable(empty,[(6,3.2)]))
        self.assertFalse(reachable(wide,[(6,3.2)]))
        self.assertFalse(reachable(empty,[(-6,1.6),(6,3.2)]))
        self.assertLess(2*empty,1.6)
        self.assertGreater(2*wide,1.6)
        self.assertLess(2*wide,3.2)


if __name__=='__main__': unittest.main()
