"""Scenario-only Supervisor. Never drives the transport robot or supplies its pose to navigation."""
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

from controller import Supervisor
from scenario import Scenario, actor_pose

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from build_integration_world import load_node


def main():
    config_bytes = (ROOT/'config/integration_scenario.json').read_bytes()
    config = json.loads(config_bytes)
    state = Scenario(config)
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    keyboard = robot.getKeyboard()
    keyboard.enable(step)
    emitter = robot.getDevice('events')
    receiver = robot.getDevice('commands')
    receiver.enable(step)
    nodes = {name: robot.getFromDef(name) for name in (
        ['TRANSPORT_ROBOT','NARROW_GATE','WIDE_GATE']
        + [a['id'] for a in config['actors']]
        + ['SPILL_'+str(i) for i in range(1,4)]
        + ['SPILL_COLOR_'+str(i) for i in range(1,4)]
        + ['LOAD_'+p['id'].upper() for p in config['loads']])}
    if any(n is None for n in nodes.values()):
        raise RuntimeError('World/config mismatch: rebuild integration_test.wbt.')
    loads = [(nodes['LOAD_'+p['id'].upper()],p) for p in config['loads']]
    results = ROOT/'results/integration'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    results.mkdir(parents=True)
    (results/'scenario.json').write_bytes(config_bytes)
    (results.parent/'latest.json').write_text(json.dumps({'directory':str(results)}))
    events_file = (results/'events.jsonl').open('w',encoding='utf-8')
    truth_file = (results/'ground_truth.csv').open('w',newline='',encoding='utf-8')
    writer = csv.writer(truth_file)
    writer.writerow(['simulation_time_s','object_id','x_m','y_m','z_m'])
    gates = {'narrow':False,'wide':False}
    temporary_until = None
    selected = 0
    output_count = 0
    seq = 0
    now = 0.0
    held = set()
    last_publish = -1.0
    smoke = '--smoke' in sys.argv
    smoke_done = set()
    actor_times = {a['id']:0.0 for a in config['actors']}

    def log(kind, **data):
        events_file.write(json.dumps(dict(time=now, event=kind, **data))+'\n')
        events_file.flush()

    def status(message):
        print('[warehouse] '+message, flush=True)
        log('message', message=message)

    def occupants():
        # Conservative circles are used ONLY to prevent spawning into objects.
        yield nodes['TRANSPORT_ROBOT'].getPosition(), .70
        for node,p in loads:
            yield node.getPosition(), math.hypot(*p['deck_size'][:2])/2
        for actor in config['actors']:
            yield nodes[actor['id']].getPosition(), (1.55 if actor['kind']=='forklift' else .32)

    def clear_rectangle(center, half_size):
        for pos,radius in occupants():
            if (abs(pos[0]-center[0]) < half_size[0]+radius
                    and abs(pos[1]-center[1]) < half_size[1]+radius):
                return False
        return True

    def gate(name, closed):
        x,width=(-6,1.6) if name=='narrow' else (6,3.2)
        if closed and not gates[name] and not clear_rectangle([x,0],[width/2,.125]):
            raise ValueError('Gate area occupied. Move traffic/robot/load clear and retry.')
        nodes[name.upper()+'_GATE'].getField('translation').setSFVec3f([x,0,.55 if closed else 4])
        gates[name]=closed
        log('gate', id=name, closed=closed)

    def spawn():
        nonlocal output_count
        profile=config['loads'][selected]
        xy=config['output_station']
        if not clear_rectangle(xy,[1.2,1.05]):
            raise ValueError('Storage output occupied. Clear the previous pallet and robot first.')
        output_count += 1
        name='OUTPUT_'+str(output_count)
        robot.getRoot().getField('children').importMFNodeFromString(-1,load_node(profile,name,[*xy,.52]))
        node=robot.getFromDef(name)
        if node is None:
            raise RuntimeError('Output pallet import failed.')
        loads.append((node,profile))
        log('pallet_ready', id=name, profile=profile['id'], station='storage_output')
        status(f'{name}: {profile["id"]} ready at storage output.')

    def command(data):
        nonlocal selected, temporary_until
        op=data.get('op')
        if op in ('report','presence'):
            index=next((i for i,s in enumerate(state.spills) if s['id']==data.get('id')),None)
            if index is None or type(data.get('value')) is not bool:
                raise ValueError('Use a valid spill id and boolean value.')
            if op=='report': state.set_reported(index,data['value'])
            else: state.set_present(index,data['value'])
        elif op in ('traffic','random_spills'):
            if type(data.get('value')) is not bool: raise ValueError('value must be boolean')
            setattr(state,'traffic_enabled' if op=='traffic' else 'random_enabled',data['value'])
        elif op=='gate':
            if data.get('id') not in gates or type(data.get('value')) is not bool:
                raise ValueError('Gate id must be narrow/wide; value must be boolean.')
            gate(data['id'],data['value'])
            if data['id']=='wide': temporary_until=None
        elif op=='temporary_blockage':
            gate('wide',True)
            temporary_until=now+config['temporary_blockage_s']
        elif op=='select_load':
            ids=[p['id'] for p in config['loads']]
            if data.get('id') not in ids: raise ValueError('Unknown load profile.')
            selected=ids.index(data['id'])
        elif op=='spawn': spawn()
        else: raise ValueError('Unknown operation.')
        log('command', command=data)

    def public_state():
        return dict(schema_version=1,type='warehouse_status',seq=seq,simulation_time_s=now,
                    frame='warehouse', reported_spills=state.reports(),
                    output_profile=config['loads'][selected]['id'],
                    output_pallets=[dict(id=n.getDef(),profile=p['id']) for n,p in loads if n.getDef().startswith('OUTPUT_')])

    def publish():
        nonlocal seq
        seq += 1
        public=public_state()
        emitter.send(json.dumps(public).encode('utf-8'))
        (results/'reported_state.json').write_text(json.dumps(public,indent=2))
        (results/'evaluation_state.json').write_text(json.dumps(dict(
            simulation_time_s=now,spills=state.spills,gates=gates,
            traffic_enabled=state.traffic_enabled,random_spills_enabled=state.random_enabled),indent=2))
        for name,node in nodes.items():
            if name.startswith(('LOAD_','PERSON_','FORKLIFT_','TRANSPORT_')):
                writer.writerow([now,name,*node.getPosition()])
        for node,_ in loads[len(config['loads']):]:
            writer.writerow([now,node.getDef(),*node.getPosition()])
        truth_file.flush()
        spill_text=' | '.join(f'{i+1}: '+('REPORTED' if s['reported'] else 'unreported') if s['present'] else f'{i+1}: absent'
                              for i,s in enumerate(state.spills))
        robot.setLabel(0,'INTEGRATION TEST | 1/2/3 report | 4/5/6 presence | T traffic | N random',.015,.02,.027,0x17212B,0,'Arial')
        robot.setLabel(1,spill_text,.015,.055,.027,0x17212B,0,'Arial')
        robot.setLabel(2,f'G narrow gate | H wide gate | B block 20s | F profile: {config["loads"][selected]["id"]} | O output',.015,.09,.025,0x17212B,0,'Arial')
        robot.setLabel(3,f'Traffic: {state.traffic_enabled} | random spills: {state.random_enabled} | gates: {gates}',.015,.12,.025,0x17212B,0,'Arial')

    status('1/2/3 toggle spill reports; 4/5/6 toggle presence; T traffic; N random spills.')
    status('G/H toggle narrow/wide gates; B blocks wide route for 20s; F selects output profile; O requests pallet.')
    status('Reload the world for a fresh seeded scenario. Results: '+str(results))
    log('start',seed=config['seed'],config_sha256=hashlib.sha256(config_bytes).hexdigest())
    try:
        while robot.step(step) != -1:
            now=robot.getTime()
            keys=set()
            key=keyboard.getKey()
            while key!=-1:
                keys.add(key & 0xFFFF); key=keyboard.getKey()
            pressed=keys-held; held=keys
            pending=[]
            for i in range(3):
                if ord(str(i+1)) in pressed:
                    pending.append(dict(op='report',id=state.spills[i]['id'],value=not state.spills[i]['reported']))
                if ord(str(i+4)) in pressed:
                    pending.append(dict(op='presence',id=state.spills[i]['id'],value=not state.spills[i]['present']))
            for key,op,value in [('T','traffic',not state.traffic_enabled),('N','random_spills',not state.random_enabled)]:
                if ord(key) in pressed: pending.append(dict(op=op,value=value))
            for key,name in [('G','narrow'),('H','wide')]:
                if ord(key) in pressed: pending.append(dict(op='gate',id=name,value=not gates[name]))
            if ord('B') in pressed: pending.append(dict(op='temporary_blockage'))
            if ord('F') in pressed: pending.append(dict(op='select_load',id=config['loads'][(selected+1)%len(config['loads'])]['id']))
            if ord('O') in pressed: pending.append(dict(op='spawn'))
            while receiver.getQueueLength():
                try:
                    data=json.loads(receiver.getString())
                    if not isinstance(data,dict): raise ValueError('Command must be a JSON object.')
                    pending.append(data)
                except (ValueError,TypeError) as exc: status('Rejected packet: '+str(exc))
                receiver.nextPacket()
            if smoke:
                schedule=[(.5,dict(op='temporary_blockage')),
                          (1,dict(op='report',id='spill_1',value=True)),
                          (2,dict(op='presence',id='spill_3',value=True)),
                          (3,dict(op='report',id='spill_3',value=True)),
                          (4,dict(op='gate',id='narrow',value=True)),
                          (5,dict(op='spawn')),
                          (6,dict(op='spawn')),
                          (7,dict(op='gate',id='narrow',value=False))]
                for i,(at,data) in enumerate(schedule):
                    if now>=at and i not in smoke_done: pending.append(data); smoke_done.add(i)
            for data in pending:
                result=dict(type='command_result',request_id=data.get('request_id'),op=data.get('op'),simulation_time_s=now)
                try:
                    command(data)
                    result['ok']=True
                except ValueError as exc:
                    status('Rejected: '+str(exc))
                    result.update(ok=False,reason=str(exc))
                emitter.send(json.dumps(result).encode('utf-8'))
            for changed in state.advance(step/1000): log('random_spill_change',id=changed)
            for actor in config['actors']:
                actor_time=actor_times[actor['id']] + (step/1000 if state.traffic_enabled else 0)
                xy,heading=actor_pose(actor,actor_time)
                radius=1.55 if actor['kind']=='forklift' else .32
                obstructed=any(closed and abs(xy[0]-x)<width/2+radius and abs(xy[1])<.125+radius
                    for name,x,width in [('narrow',-6,1.6),('wide',6,3.2)] for closed in [gates[name]])
                if not obstructed:
                    actor_times[actor['id']]=actor_time
                else:
                    xy,heading=actor_pose(actor,actor_times[actor['id']])
                nodes[actor['id']].getField('translation').setSFVec3f([*xy,0])
                nodes[actor['id']].getField('rotation').setSFRotation([0,0,1,heading])
            for i,spill in enumerate(state.spills,1):
                nodes['SPILL_'+str(i)].getField('translation').setSFVec3f([*spill['center'],.007 if spill['present'] else -1])
                color=[.85,.15,.08] if spill['reported'] else [.8,.55,.05]
                nodes['SPILL_COLOR_'+str(i)].getField('baseColor').setSFColor(color)
            if temporary_until is not None and now>=temporary_until:
                gate('wide',False); temporary_until=None
            if now-last_publish>=.5 or pending:
                publish(); last_publish=now
            if smoke and '--preview' in sys.argv and now>=2 and 'preview' not in smoke_done:
                robot.exportImage(str(results/'preview.png'),100)
                smoke_done.add('preview')
            if smoke and now>=25:
                assert len(loads)==len(config['loads'])+1, 'Occupied output must reject second spawn.'
                assert len(state.reports())==3 and not any(gates.values())
                for node,_ in loads:
                    assert .48<node.getPosition()[2]<.56, 'Pallet did not settle on rails.'
                assert math.dist(nodes['PERSON_1'].getPosition()[:2],config['actors'][0]['start'])>.2
                status('SMOKE PASS: spill reporting/presence, gates, timed reopening, moving traffic, output occupancy, load settling.')
                robot.simulationQuit(0)
                break
    except Exception:
        if smoke:
            log('smoke_failed')
            robot.simulationQuit(1)
        raise
    finally:
        events_file.close(); truth_file.close()


if __name__=='__main__':
    main()
