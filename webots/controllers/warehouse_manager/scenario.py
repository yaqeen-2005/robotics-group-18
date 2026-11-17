"""Deterministic scenario state; independent of Webots for testing."""
import copy
import math
import random


def actor_pose(actor, time):
    """Straight shuttle with dwell at each end; returns XY and fixed heading.

    Forklifts reverse on the return leg, avoiding instantaneous 180-degree turns.
    """
    a, b = actor['start'], actor['end']
    length = math.dist(a, b)
    duration = length / actor['speed']
    dwell = actor['dwell_s']
    t = time % (2 * (duration + dwell))
    if t < dwell:
        u = 0
    elif t < dwell + duration:
        u = (t - dwell) / duration
    elif t < 2 * dwell + duration:
        u = 1
    else:
        u = 1 - (t - 2 * dwell - duration) / duration
    return [a[i] + u * (b[i] - a[i]) for i in range(2)], math.atan2(b[1]-a[1], b[0]-a[0])


class Scenario:
    def __init__(self, config):
        self.config = config
        self.spills = copy.deepcopy(config['spills'])
        self.rng = random.Random(config['seed'])
        self.random_enabled = config['random_spills_enabled']
        self.traffic_enabled = config['traffic_enabled']
        self.traffic_time = 0.0
        self.tick_elapsed = 0.0

    def set_present(self, index, present):
        spill = self.spills[index]
        spill['present'] = present
        # A newly appearing spill always needs a fresh report.
        spill['reported'] = False

    def set_reported(self, index, reported):
        spill = self.spills[index]
        if reported and not spill['present']:
            raise ValueError('Cannot report an absent spill; make it present first.')
        spill['reported'] = reported

    def advance(self, dt):
        changes = []
        if self.traffic_enabled:
            self.traffic_time += dt
        if self.random_enabled:
            self.tick_elapsed += dt
            while self.tick_elapsed + 1e-9 >= self.config['spill_tick_s']:
                self.tick_elapsed -= self.config['spill_tick_s']
                for i, spill in enumerate(self.spills):
                    if self.rng.random() < self.config['spill_change_probability']:
                        self.set_present(i, not spill['present'])
                        changes.append(spill['id'])
        return changes

    def reports(self):
        # Only explicitly reported hazards cross into the navigation interface.
        return [dict(id=s['id'], center=s['center'], size=s['size'])
                for s in self.spills if s['present'] and s['reported']]


def near_robot(current, proposed, actor_radius, robot_xy, robot_radius, stopped):
    """Stop before contact; extra restart clearance prevents stop/start chatter."""
    clearance = .35 if stopped else .20
    distance = min(math.dist(current, robot_xy), math.dist(proposed, robot_xy))
    return distance < actor_radius + robot_radius + clearance
