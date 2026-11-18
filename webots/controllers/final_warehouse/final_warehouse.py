"""Meridian facility supervisor. The transport robot keeps its own controller."""
import csv
from datetime import datetime, timezone
import json
import math
import os
import secrets
import sys

from controller import Supervisor
from layout import CFG, EMPTY_RADIUS, GATE_COIL, PERSON_RADIUS, PROFILES, ROOT, SEAT_Z, SLATS, SLAT_HEIGHT, TRUCK_RADIUS, camera_view, gates
from operations import Dispatch, Gate, Spills
from traffic import make_traffic
from layout import wet_tyres

sys.path.insert(0, str(ROOT/'tools'))
from build_integration_world import load_node

LANE_COLORS = ((.13,.91,.78), (1.,.65,.17), (.83,.51,.98))
LANE_INK = (0x65FFE3, 0xFFD073, 0xE0ADFF)


class Facility:
    def __init__(self, sim):
        self.sim, self.now = sim, 0.
        self.step_ms = int(sim.getBasicTimeStep()) * 4
        self.dt = self.step_ms / 1000
        choice = os.environ.get('G18_WAREHOUSE_SEED') or CFG['seed']
        self.seed = secrets.randbits(32) if choice is None or str(choice).lower()=='random' else int(choice)
        self.spills, self.dispatch = Spills(self.seed + 11), Dispatch(self.seed + 19)
        self.gates = [Gate(spec, self.seed + 100 + index) for index,spec in enumerate(gates())]
        self.trucks, self.people = make_traffic(self.seed)
        self.nodes, self.fields, self.last = {}, {}, {}
        self.body = self.node('TRANSPORT_ROBOT')
        self.tyres=[self.body.getField(side+'TyreMaterial') for side in ('left','right')]
        self.wet=(False,False)
        self.root = sim.getRoot().getField('children')
        self.items = {}
        self.emitter = sim.getDevice('events'); self.receiver = sim.getDevice('commands'); self.receiver.enable(self.step_ms)
        self.keyboard = sim.getKeyboard(); self.keyboard.enable(self.step_ms)
        self.held, self.seq = set(), 0
        self.camera, self.focus, self.inspector = 0, 0, None
        self.banner = None
        self.next_publish, self.next_trace, self.next_camera = 0., 0., 0.
        self.traffic_enabled, self.random_spills = True, True
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.out = ROOT/'results/final'/stamp; self.out.mkdir(parents=True)
        self.events = (self.out/'events.jsonl').open('w',encoding='utf-8',buffering=1)
        self.truth_file = (self.out/'ground_truth.csv').open('w',newline='',encoding='utf-8')
        self.truth = csv.writer(self.truth_file); self.truth.writerow(['time_s','id','x_m','y_m','z_m','yaw_rad'])
        (self.out/'run.json').write_text(json.dumps({'seed':self.seed,'scenario':CFG,'profiles':PROFILES},indent=2),encoding='utf-8')
        self.log('start', seed=self.seed)
        print(f'[Meridian] Seed {self.seed}. 1/2/3: focus lane, V: camera, I: inspect next spill, H: toggle its report. Logs: {self.out}',flush=True)

    def node(self, ident):
        if ident not in self.nodes:
            node = self.sim.getFromDef(ident)
            if node is None: raise RuntimeError('World missing DEF '+ident)
            self.nodes[ident] = node
        return self.nodes[ident]

    def put(self, ident, key, value, kind='Vec3f'):
        pair = ident,key
        data = float(value) if kind=='Float' else tuple(value)
        if self.last.get(pair) == data: return
        if pair not in self.fields: self.fields[pair] = self.node(ident).getField(key)
        getattr(self.fields[pair],'setSF'+kind)(data if kind=='Float' else list(data))
        self.last[pair] = data

    def log(self, event, **details):
        self.events.write(json.dumps(dict(time=round(self.now,3),event=event,**details))+'\n')

    def radio(self, data):
        self.emitter.send(json.dumps(data,separators=(',',':')).encode('utf-8'))

    def command(self, data):
        if not isinstance(data, dict): raise ValueError('Expected a JSON object.')
        op, value = data.get('op'), data.get('value')
        if op == 'report': self.spills.report(data.get('id'),value)
        elif op in ('traffic','random_spills') and type(value) is bool:
            if op == 'traffic': self.traffic_enabled = value
            else: self.random_spills = value
        else: raise ValueError('Expected report, traffic or random_spills with a boolean value.')
        self.log('command', command=data)

    def inputs(self):
        keys=set(); key=self.keyboard.getKey()
        while key != -1:
            keys.add(key & 0xffff); key=self.keyboard.getKey()
        pressed=keys-self.held; self.held=keys
        for index in range(3):
            if ord('1')+index in pressed: self.focus=index; self.next_camera=0
        if ord('V') in pressed: self.camera=(self.camera+1)%4; self.next_camera=0; self.set_camera()
        if ord('I') in pressed:
            active=sorted(self.spills.present)
            self.inspector=active[(active.index(self.inspector)+1)%len(active)] if self.inspector in active else active[0]
        if ord('H') in pressed and self.inspector in self.spills.present and self.inspector not in self.spills.cleaning:
            self.command(dict(op='report',id=self.inspector,value=self.inspector not in self.spills.reported))
        while self.receiver.getQueueLength():
            data={}
            try:
                data=json.loads(self.receiver.getString()); self.command(data)
                result={'ok':True}
            except (ValueError,TypeError) as error:
                result={'ok':False,'reason':str(error)}
            self.radio(dict(type='command_result',request_id=data.get('request_id') if isinstance(data,dict) else None,
                            op=data.get('op') if isinstance(data,dict) else None,simulation_time_s=self.now,**result))
            self.receiver.nextPacket()

    def insert(self, job, physical):
        name=job.ident if physical else 'BELT_'+job.ident
        profile=PROFILES[job.profile]; position=[job.x,job.pickup[1],SEAT_Z+(.002 if physical else .003)]
        if physical: text=load_node(profile,name,position)
        else:
            size=profile['deck_size']; h=profile['cargo_height']; dx,dy=size[:2]
            text=f'''DEF {name} Solid {{ name "{name.lower()}" translation {' '.join(map(str,position))}
 children [ Shape {{ appearance USE WOOD geometry Box {{ size {dx} {dy} .08 }} }}
 Pose {{ translation 0 0 {.04+h/2} children [ Shape {{ appearance PBRAppearance {{ baseColor {' '.join(map(str,profile['color']))} metalness 0 roughness .8 }} geometry Box {{ size {dx-.08} {dy-.08} {h} }} }} ] }} ]
 boundingObject Pose {{ translation 0 0 {h/2} children [ Box {{ size {dx} {dy} {h+.08} }} ] }} }}'''
        self.root.importMFNodeFromString(-1,text)
        node=self.sim.getFromDef(name)
        if node is None: raise RuntimeError('Could not create pallet '+name)
        self.items[job.ident]=node

    def freight(self):
        return [(job,self.items[job.ident].getPosition(),math.hypot(*PROFILES[job.profile]['deck_size'][:2])/2)
                for job in self.dispatch.jobs if job]

    def run_dispatch(self, robot, actors, freight):
        feed=CFG['dispatch']
        def clear(centre, half_x, half_y, ignore=None):
            external=[robot]+actors+[(pos,radius) for job,pos,radius in freight if job is not ignore]
            return all(math.hypot(max(0,abs(pos[0]-centre[0])-half_x),max(0,abs(pos[1]-centre[1])-half_y)) > radius+.10 for pos,radius in external)
        for lane,job in enumerate(self.dispatch.jobs):
            y=feed['lanes_y'][lane]
            if job is None:
                job=self.dispatch.allocate(lane,self.now,clear((feed['source_x'],y),.95,.92))
                if job: self.insert(job,False); self.log('order_spawned',**job.public())
                continue
            node=self.items[job.ident]; old=job.phase
            if old=='feeding':
                dx,dy=PROFILES[job.profile]['deck_size'][:2]
                if self.dispatch.advance_belt(job,self.dt,clear((min(feed['x'],job.x+self.dt*feed['belt_m_s']),y),dx/2,dy/2,job)):
                    node.remove(); self.insert(job,True)
                else: node.getField('translation').setSFVec3f([job.x,y,SEAT_Z+.003])
            elif old in ('ready','in_transit'):
                orient=node.getOrientation(); yaw=math.atan2(orient[3],orient[0]); speed=math.hypot(*node.getVelocity()[:3])
                self.dispatch.observe(job,self.now,node.getPosition(),robot[0],orient[8]>.993,yaw,speed)
            elif old=='delivered':
                if self.dispatch.release(job,self.now,clear(job.dock['at'],1.15,.65,job)):
                    node.remove(); del self.items[job.ident]; self.log('order_cleared',id=job.ident)
                elif job.fade_elapsed>0:
                    node.getField('transparency').setSFFloat(job.transparency)
            if old!=job.phase:
                self.log('order_'+job.phase,**job.public())
                if job.phase in ('in_transit','delivered'):
                    self.focus=job.lane; self.next_camera=0
                    pickup=job.phase=='in_transit'
                    title=f'PARCEL #{job.serial:03} '+('PICKED UP' if pickup else 'DELIVERED')
                    detail=('Take to ' if pickup else 'Received at ')+job.dock['id']+' / '+job.dock['name']
                    self.banner=(title,detail,LANE_INK[job.lane],self.now+4.)
                    self.next_publish=0

    def gate_and_traffic(self, robot, freight):
        actors=[(p.pos,PERSON_RADIUS) for p in self.people]+[(t.pos,TRUCK_RADIUS) for t in self.trucks]
        obstacles=[(pos,radius) for _,pos,radius in freight]
        stock=[(t.cargo[:2],.85) for t in self.trucks]
        occupants=[robot]+actors+obstacles+stock
        for gate in self.gates:
            old=gate.phase; gate.step(self.now,self.dt,occupants); spec=gate.spec
            for index in range(SLATS):
                height=min(GATE_COIL,(index+.5)*SLAT_HEIGHT+gate.travel*GATE_COIL)
                self.put(spec['id'] if index==0 else spec['id']+f'_SLAT_{index}','translation',(spec['x'],spec['y'],height))
            lamp=(.18,.85,.43) if gate.phase=='open' else ((.96,.40,.10) if gate.phase=='closed' else (.99,.73,.16))
            self.put(spec['id']+'_LIGHT','emissiveColor',lamp,'Color')
            if old!=gate.phase: self.log('gate',id=spec['id'],state=gate.phase)
        horizontal=[(pos[:2],radius) for pos,radius in obstacles]
        if self.traffic_enabled:
            for truck in self.trucks:
                truck.reported=self.spills.public()
                old=truck.phase; truck.step(self.dt,robot,[walker.pos for walker in self.people],self.gates,horizontal)
                if old!=truck.phase: self.log('stock_'+truck.phase,id=truck.spec['id'],aisle=truck.aisle,trips=truck.trips,
                                               stock_position=truck.cargo)
        for person in self.people:
            person.step(self.dt,robot,self.trucks,self.gates,[other.pos for other in self.people if other is not person],horizontal+stock,enabled=self.traffic_enabled)
        for truck in self.trucks:
            ident=truck.spec['id']; self.put(ident,'translation',(*truck.pos,0)); self.put(ident,'rotation',(0,0,1,truck.yaw),'Rotation')
            self.put(ident+'_FORKS','translation',(0,0,truck.forks)); self.put(ident+'_STOCK','translation',truck.cargo)
            for suffix,side in [('L',1),('R',-1)]: self.put(ident+'_TINE_'+suffix,'translation',(1.39,side*.30,.44+truck.forks))
            self.put(ident+'_STOCK','rotation',(0,0,1,truck.cargo_yaw),'Rotation')
        for person in self.people:
            ident=person.spec['id']; self.put(ident,'translation',(*person.pos,0)); self.put(ident,'rotation',(0,0,1,person.yaw),'Rotation')
            for suffix,side in [('L',1),('R',-1)]: self.put(ident+'_LEG_'+suffix,'rotation',(0,1,0,side*person.leg_angle),'Rotation')
        return [(p.pos,PERSON_RADIUS) for p in self.people]+[(t.pos,TRUCK_RADIUS) for t in self.trucks]+[(t.cargo[:2],.85) for t in self.trucks]

    def set_camera(self):
        job=self.dispatch.jobs[self.focus]; dock=job.dock if job else {'at':CFG['parking'],'entry':1}
        eye,axis=camera_view(self.camera,self.body.getPosition(),dock)
        self.put('HALL_CAMERA','position',eye); self.put('HALL_CAMERA','orientation',axis,'Rotation')

    def label(self, index, text, y, color=0xFFFFFF, size=.052, x=.012):
        style=(text,x,y,color,size)
        if self.last.get(('hud',index))==style: return
        self.sim.setLabel(20+index,text,x+.0015,y+.002,size,0x081820,0,'Arial')
        self.sim.setLabel(index,text,x,y,size,color,0,'Arial'); self.last['hud',index]=style

    def beacon(self, ident, color, strength):
        self.put(ident+'_LAMP','baseColor',color,'Color')
        self.put(ident+'_LAMP','emissiveColor',tuple(v*strength for v in color),'Color')

    def display(self):
        carried=next((j for j in self.dispatch.jobs if j and j.phase=='in_transit'),None)
        header=(f'ON BOARD   /   PARCEL #{carried.serial:03}    >    {carried.dock["id"]}  {carried.dock["name"].upper()}'
                if carried else 'MERIDIAN  /  DISPATCH')
        self.label(0,header,.015,0xFFFFFF,.066)
        for lane,job in enumerate(self.dispatch.jobs):
            on_board=job is not None and job is carried
            mark='[CARRYING]' if on_board else ('>' if lane==self.focus else ' ')
            phrase={'feeding':'on rollers','ready':'ready to lift','in_transit':'deliver here','delivered':'received'}
            text=(f'{mark} P{lane+1:02}   #{job.serial:03}  {job.profile.upper()}  >  {job.dock["id"]} {job.dock["name"]}   |   {phrase[job.phase]}'
                  if job else f'{mark} P{lane+1:02}   Awaiting next pallet')
            self.label(lane+1,text,.083+lane*.050,0xFFFFFF if on_board else LANE_INK[lane],.059 if on_board else .051)
            self.beacon(f'P{lane+1:02}',LANE_COLORS[lane] if job else (.18,.31,.34),
                        (1. if job and job.phase in ('ready','feeding') and self.focus==lane else .66) if job else 0.)
            signal=(.13,.87,.45) if job and job.phase=='ready' else ((.98,.52,.12) if job and job.phase=='feeding' else (.12,.28,.30))
            self.put(f'P{lane+1:02}_SIGNAL','emissiveColor',tuple(v*.74 for v in signal),'Color')
        if self.inspector in self.spills.present:
            note=(f'{self.inspector}: cleaning in progress' if self.inspector in self.spills.cleaning else
                  f'{self.inspector}: '+('reported' if self.inspector in self.spills.reported else 'not reported')+'   [H] toggle')
        else: note='[1-3] lane    [V] camera    [I] inspect spill'
        self.label(4,note,.242,0xEEF0CC,.039)
        if self.banner and self.now >= self.banner[3]: self.banner=None
        title,detail,ink=self.banner[:3] if self.banner else ('','',0xFFFFFF)
        self.label(5,title,.442,0xFFFFFF,.109,.365)
        self.label(6,detail,.534,ink,.073,.378)
        for dock in CFG['docks']:
            job=next((j for j in self.dispatch.jobs if j and j.dock['id']==dock['id']),None)
            color=LANE_COLORS[job.lane] if job else (.18,.31,.34)
            factor=(1. if job is carried or (not carried and job.lane==self.focus) else .46) if job else 0.
            self.beacon(dock['id'],color,factor)

    def step(self):
        self.now=self.sim.getTime(); before=(set(self.spills.present),set(self.spills.reported))
        self.inputs()
        for event,ident in self.spills.step(self.now,self.random_spills):
            self.log('spill_'+event,id=ident)
        if self.now<=self.dt+.001 or before!=(self.spills.present,self.spills.reported):
            self.log('spill_state',present=sorted(self.spills.present),reported=sorted(self.spills.reported))
        for ident,site in self.spills.sites.items():
            alpha=self.spills.transparency(ident,self.now)
            self.put(ident.upper()+'_FILM','transparency',alpha,'Float')
            self.put(ident.upper()+'_MARK','transparency',alpha,'Float')
            self.put(ident.upper(),'translation',(*site['center'],.018 if ident in self.spills.present else -2))
            self.put(ident.upper()+'_REPORT','translation',(*site['center'],.023 if ident in self.spills.reported else -2))
        pos=self.body.getPosition(); freight=self.freight(); radius=EMPTY_RADIUS
        wet=wet_tyres(pos,self.body.getOrientation(),[self.spills.sites[s] for s in self.spills.present])
        if wet!=self.wet:
            for field,old,new in zip(self.tyres,self.wet,wet):
                if old!=new: field.setSFString('wet_tyre' if new else 'tyre')
            self.wet=wet; self.log('wheel_surface',left_wet=wet[0],right_wet=wet[1])
        for job,xyz,size in freight:
            separation=math.dist(pos[:2],xyz[:2])
            if separation<1.0 and job.phase!='feeding': radius=max(radius,separation+size)
        robot=(pos[:2],radius)
        actors=self.gate_and_traffic(robot,freight); self.run_dispatch(robot,actors,freight)
        if self.now>=self.next_publish:
            self.display()
            self.radio(dict(type='warehouse_status',schema_version=1,seq=self.seq,simulation_time_s=self.now,frame='warehouse',
                 reported_spills=self.spills.public(),output_pallets=[job.public() for job in self.dispatch.jobs if job],
                 gates=[dict(id=g.spec['id'],mode=g.spec['mode'],state=g.phase,open=g.phase=='open') for g in self.gates]))
            self.seq+=1; self.next_publish=self.now+.48
        if self.camera in (2,3) and self.now>=self.next_camera: self.set_camera(); self.next_camera=self.now+.15
        if self.now>=self.next_trace:
            ori=self.body.getOrientation(); poses=[('TRANSPORT_ROBOT',pos,math.atan2(ori[3],ori[0]))]
            poses.extend((p.spec['id'],(*p.pos,0),p.yaw) for p in self.people)
            for truck in self.trucks:
                poses.extend([(truck.spec['id'],(*truck.pos,0),truck.yaw),(truck.spec['id']+'_STOCK',truck.cargo,truck.cargo_yaw)])
            for ident,node in self.items.items():
                ori=node.getOrientation(); poses.append((ident,node.getPosition(),math.atan2(ori[3],ori[0])))
            for ident,xyz,yaw in poses:
                self.truth.writerow([round(self.now,3),ident,*[round(v,4) for v in xyz],round(yaw,4)])
            self.truth_file.flush(); self.next_trace=self.now+.98


def main():
    sim=Supervisor(); facility=Facility(sim)
    try:
        while sim.step(facility.step_ms)!=-1: facility.step()
    finally:
        facility.events.close(); facility.truth_file.close()


if __name__=='__main__': main()
