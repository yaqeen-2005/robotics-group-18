"""Time-based facilities and order rules, independent of the simulator."""
import math
import random
from dataclasses import dataclass

from layout import CFG, PROFILES, SEAT_Z


class Gate:
    def __init__(self, spec, seed):
        self.spec, self.policy, self.rng = spec, CFG['gate_policy'], random.Random(seed)
        self.phase = 'open' if spec['mode'] == 'timed' else 'closed'
        self.travel = float(self.phase == 'open')
        self.since = self.last_seen = 0.
        self.tick = self.rng.uniform(1, self.policy['tick_s'])

    def step(self, now, dt, occupants):
        spec, policy = self.spec, self.policy
        def near(depth):
            return any(abs(pos[0] - spec['x']) <= spec['w'] / 2 + radius and
                       abs(pos[1] - spec['y']) <= depth + radius for pos, radius in occupants)
        seen = near(policy['sensor_m'] if spec['mode'] == 'presence' else .95)
        if seen:
            self.last_seen = now
        tick = now >= self.tick
        if tick:
            self.tick += policy['tick_s']
        if self.phase == 'closing' and seen:
            self.phase = 'opening'
        elif self.phase == 'closed':
            if ((spec['mode'] == 'presence' and seen) or (spec['mode'] == 'timed' and tick and
                    now - self.since >= policy['min_closed_s'] and self.rng.random() < policy['open_chance'])):
                self.phase = 'opening'
        elif self.phase == 'open' and not seen and now - self.last_seen >= policy['clear_hold_s'] and now - self.since >= policy['min_open_s']:
            if spec['mode'] == 'presence' or (tick and self.rng.random() < policy['close_chance']):
                self.phase = 'closing'
        if self.phase in ('opening', 'closing'):
            self.travel = min(1., max(0., self.travel + (1 if self.phase == 'opening' else -1) * dt / policy['travel_s']))
            if self.travel in (0., 1.):
                self.phase = 'open' if self.travel else 'closed'
                self.since = now


class Spills:
    def __init__(self, seed):
        self.rng, self.policy = random.Random(seed), CFG['spill_policy']
        self.sites = {site['id']: site for site in CFG['spills']}
        self.present = set(self.rng.sample(list(self.sites), self.policy['min']))
        self.reported = set()
        self.cleaning = {}
        self.next_tick = self.policy['tick_s']

    def report(self, ident, value):
        if type(value) is not bool or ident not in self.sites:
            raise ValueError('Supply a spill id and a boolean report value.')
        if value and ident not in self.present:
            raise ValueError('There is no active spill at ' + ident)
        if ident in self.cleaning and not value:
            raise ValueError('Cleanup is underway; the area stays reported until it is clear.')
        (self.reported.add if value else self.reported.discard)(ident)

    def transparency(self, ident, now):
        if ident not in self.present: return 1.
        if ident not in self.cleaning: return 0.
        return min(1., max(0., (now-self.cleaning[ident])/self.policy['fade_s']))

    def step(self, now, random_ticks=True):
        changes = []
        for ident in list(self.cleaning):
            if self.transparency(ident, now) < 1.: continue
            del self.cleaning[ident]
            self.present.discard(ident)
            self.reported.discard(ident)
            changes.append(('cleaned', ident))
        if not random_ticks:
            self.next_tick = now + self.policy['tick_s']
        if now < self.next_tick: return changes
        self.next_tick = now + self.policy['tick_s']
        candidates = sorted(self.reported - self.cleaning.keys())
        if candidates and len(self.present)-len(self.cleaning) > self.policy['min'] and self.rng.random() < self.policy['cleanup_chance']:
            ident = self.rng.choice(candidates)
            self.cleaning[ident] = now
            changes.append(('cleaning', ident))
        vacant = sorted(self.sites.keys() - self.present - {i for event,i in changes if event=='cleaned'})
        if vacant and len(self.present) < self.policy['max'] and self.rng.random() < self.policy['add_chance']:
            ident = self.rng.choice(vacant)
            self.present.add(ident)
            changes.append(('appeared', ident))
        for ident in sorted(self.present - self.reported):
            if self.rng.random() < self.policy['report_chance']:
                self.reported.add(ident)
        return changes

    def public(self):
        return [dict(self.sites[ident]) for ident in sorted(self.reported)]


@dataclass
class Job:
    serial: int
    lane: int
    profile: str
    dock: dict
    x: float
    phase: str = 'feeding'
    seated_since: float = -1.
    completed_at: float = -1.
    fade_elapsed: float = 0.
    receipt_checked_at: float = -1.

    @property
    def ident(self):
        return f'ORDER_{self.serial:04}'

    @property
    def pickup(self):
        return CFG['dispatch']['x'], CFG['dispatch']['lanes_y'][self.lane]

    def public(self):
        cargo = PROFILES[self.profile]
        return dict(id=self.ident, profile=self.profile, status=self.phase, station=f'P{self.lane + 1:02}',
                    pickup=list(self.pickup), destination=self.dock['id'], dropoff=list(self.dock['at']),
                    deck_size_m=cargo['deck_size'], mass_kg=cargo['mass_kg'], handling=cargo['handling'])

    @property
    def transparency(self):
        return min(1., self.fade_elapsed/CFG['dispatch']['fade_s'])


class Dispatch:
    def __init__(self, seed):
        self.rng, self.serial = random.Random(seed), 0
        self.jobs = [None] * len(CFG['dispatch']['lanes_y'])
        self.due = [1 + CONFIG_STAGGER * index for index in range(len(self.jobs))]

    def allocate(self, lane, now, clear):
        if self.jobs[lane] or now < self.due[lane] or not clear:
            return None
        reserved = {job.dock['id'] for job in self.jobs if job}
        choices = [dock for dock in CFG['docks'] if dock['id'] not in reserved]
        if not choices:
            return None
        self.serial += 1
        job = Job(self.serial, lane, self.rng.choice(list(PROFILES)), self.rng.choice(choices), CFG['dispatch']['source_x'])
        self.jobs[lane] = job
        return job

    def advance_belt(self, job, dt, clear):
        if job.phase != 'feeding' or not clear:
            return False
        job.x = min(job.pickup[0], job.x + CFG['dispatch']['belt_m_s'] * dt)
        if job.x >= job.pickup[0]:
            job.phase = 'ready'
            return True
        return False

    def observe(self, job, now, pos, robot, flat, yaw, speed):
        separation = math.dist(pos[:2], robot[:2])
        if job.phase == 'ready' and flat and separation < .78 and pos[2] > SEAT_Z + .065:
            job.phase = 'in_transit'
        if job.phase == 'in_transit':
            dx, dy = pos[0] - job.dock['at'][0], pos[1] - job.dock['at'][1]
            deck = PROFILES[job.profile]['deck_size']
            supported = abs(dy) + .55 <= deck[1] * .5 * abs(math.cos(yaw)) and abs(math.sin(yaw)) < .23
            seated = (abs(dx) <= .36 and supported and flat and abs(pos[2] - SEAT_Z) < .026 and
                      speed < .055 and separation > 1.8)
            if not seated:
                job.seated_since = -1.
            elif job.seated_since < 0:
                job.seated_since = now
            elif now - job.seated_since >= 1.2:
                job.phase, job.completed_at = 'delivered', now

    def release(self, job, now, clear):
        if job.phase != 'delivered':
            return False
        fade_start = job.completed_at + CFG['dispatch']['outtake_s'] - CFG['dispatch']['fade_s']
        elapsed = max(0., now - max(fade_start, job.receipt_checked_at))
        job.receipt_checked_at = now
        if not clear: return False
        job.fade_elapsed += elapsed
        if job.transparency < 1.:
            return False
        self.jobs[job.lane] = None
        self.due[job.lane] = now + CFG['dispatch']['feed_delay_s']
        return True


CONFIG_STAGGER = 2.4
