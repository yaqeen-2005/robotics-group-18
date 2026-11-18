"""Isolated simulator acceptance probe; uses world pose solely to steer a scripted QA trip."""
import json
import math
import os
from pathlib import Path
import sys
from controller import Supervisor

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'controllers/final_warehouse')); sys.path.insert(0,str(ROOT/'tools'))
from layout import CFG, PROFILES, SEAT_Z, camera_view
from build_integration_world import load_node


def traction_trials(robot,step,body,motors,tx):
    """After the delivery, start matched acceleration tests from rest on dry and wet floor."""
    for op in ('traffic','random_spills'):
        tx.send(json.dumps({'op':op,'value':False}).encode())
    sensors=[robot.getDevice(side+'_wheel_sensor') for side in ('left','right')]
    for sensor in sensors:sensor.enable(step)
    def wait(seconds):
        for _ in range(math.ceil(seconds*1000/step)): robot.step(step)
    wait(3.5)
    sites={False:[],True:[]}
    actors=[robot.getFromDef(spec['id']).getPosition()[:2] for spec in CFG['people']+CFG['trucks']]
    for s in CFG['spills']:
        if any(math.dist(s['center'],a)<3 for a in actors):continue
        present=robot.getFromDef(s['id'].upper()).getField('translation').getSFVec3f()[2]>0
        sites[present].append(s)
    results={}
    for name,wet in [('dry',False),('wet',True),('recovery',False)]:
        s=max(sites[wet],key=lambda item:item['size'][0]*item['size'][1]); x,y=s['center']
        for m in motors:m.setVelocity(0)
        body.getField('translation').setSFVec3f([x-.12,y,.16])
        body.getField('rotation').setSFRotation([0,0,1,0]);body.resetPhysics();wait(.8)
        start=body.getPosition()[:2]; before=[v.getValue() for v in sensors]
        material=[body.getField(side+'TyreMaterial').getSFString() for side in ('left','right')]
        for m in motors:m.setVelocity(4.)
        wait(.8)
        travel=math.dist(start,body.getPosition()[:2]); odo=.075*sum(v.getValue()-a for v,a in zip(sensors,before))
        results[name]=dict(site=s['id'],encoder_m=round(odo,4),actual_m=round(travel,4),materials=material)
    for m in motors:m.setVelocity(0)
    return results


def main():
    robot=Supervisor(); step=int(robot.getBasicTimeStep())*2; body=robot.getSelf()
    output=Path(__file__).resolve().parents[2]/'probe.json'
    rx=robot.getDevice('warehouse_events'); rx.enable(step); tx=robot.getDevice('warehouse_commands')
    lidar=robot.getDevice('front_lidar'); lidar.enable(step*8)
    lift=robot.getDevice('lift_motor'); lift.setVelocity(.25); lift.setPosition(0)
    stages=[robot.getDevice('lift_stage1_motor'),robot.getDevice('lift_stage2_motor')]
    sensor=robot.getDevice('lift_sensor'); sensor.enable(step)
    motors=[robot.getDevice(name) for name in ('left_wheel_motor','right_wheel_motor')]
    for motor in motors: motor.setPosition(float('inf')); motor.setVelocity(0)
    states=set(); reports=set(); readings=0; ack=False; seed_check=False; captured=set(); invalid=False; arrived=False; replenished=False
    round_index=0; placed_at=0.; berth_samples=[]; berth_checks=[]; deck_ids=list(PROFILES)
    fades={'parcel':False,'spill':False}; painted=False; movements=set()
    lane=None; current_job=None; pose=None; phase=0; since=0.; samples=[]; traction={}; staged={}
    route=[('drive',(-38,-8),1),('drive',(-40,-8),1),('lift',.27,0),('drive',(-36.7,-8),-1),
           ('drive',(-36.7,16),1),('drive',(-40,16),1),('lift',0,0),('drive',(-36.7,16),-1)]

    def finish(ok,reason):
        output.write_text(json.dumps(dict(passed=ok,reason=reason,phase=phase,robot=pose,statuses=sorted(states),packets=readings,
            wrong_command_rejected=ack,lidar=seed_check,berth_checks=len(berth_checks),fades=fades,colour_match=painted,
            reports=[list(row) for row in sorted(reports)],samples=samples,traction=traction,floor_drops=staged),indent=2))
        robot.simulationQuit(0 if ok else 1)

    while robot.step(step)!=-1:
        now=robot.getTime(); pos=body.getPosition(); ori=body.getOrientation(); yaw=math.atan2(ori[3],ori[0]); pose=[round(pos[0],2),round(pos[1],2),round(yaw,3)]
        for spec in CFG['trucks']:
            ident=spec['id']
            if ident in staged:continue
            cargo=robot.getFromDef(ident+'_STOCK').getPosition()
            if abs(cargo[2]-.14)<.003 and math.dist(cargo[:2],robot.getFromDef(ident).getPosition()[:2])>2.6:
                staged[ident]=cargo
        if berth_samples and now-placed_at>=.96:
            for dock,profile,node in berth_samples:
                xyz=node.getPosition()
                assert abs(xyz[2]-SEAT_Z)<.012 and math.dist(xyz[:2],dock['at'])<.025 and node.getOrientation()[8]>.997,(dock['id'],profile,xyz)
                berth_checks.append((dock['id'],profile)); node.remove()
            berth_samples=[]; round_index+=1
        if round_index<5 and not berth_samples and now>.18:
            for index,dock in enumerate(CFG['docks']):
                profile=deck_ids[(index+round_index)%len(deck_ids)]; ident=f'QA_BERTH_{round_index}_{index}'
                robot.getRoot().getField('children').importMFNodeFromString(-1,load_node(PROFILES[profile],ident,[*dock['at'],SEAT_Z+.05]))
                berth_samples.append((dock,profile,robot.getFromDef(ident)))
            placed_at=now
        if not invalid and now>1:
            tx.send(json.dumps({'op':'report','id':'nonexistent','value':True,'request_id':'qa'}).encode()); invalid=True
        while rx.getQueueLength():
            packet=json.loads(rx.getString()); rx.nextPacket()
            if packet['type']=='command_result' and packet['request_id']=='qa': ack=not packet['ok']; continue
            if packet['type']!='warehouse_status': continue
            assert not {'robot_pose','actors','present_spills','layout','ground_truth'} & packet.keys()
            readings+=1; reports.add(tuple(site['id'] for site in packet['reported_spills']))
            orders=packet['output_pallets']; assert len({job['destination'] for job in orders})==len(orders)
            lane=next((job for job in orders if job['station']=='P03'),None)
            if lane and current_job is None and lane['status']=='ready':
                assert lane['destination']=='D01' and lane['profile']=='standard'; current_job=lane['id']; since=now
            if lane and lane['id']==current_job:
                states.add(lane['status'])
                if lane['status'] in ('ready','in_transit','delivered'):
                    p=robot.getFromDef('P03_LAMP').getField('baseColor').getSFColor()
                    d=robot.getFromDef('D01_LAMP').getField('baseColor').getSFColor()
                    painted |= math.dist(p,d)<.01 and p[2]>.8
                if lane['status']=='delivered':
                    pallet=robot.getFromDef(current_job)
                    if pallet: fades['parcel'] |= .05<pallet.getField('transparency').getSFFloat()<.95
            for site in packet['reported_spills']:
                film=robot.getFromDef(site['id'].upper()+'_FILM').getField('transparency').getSFFloat()
                fades['spill'] |= .05<film<.95
            if lane and current_job and lane['id']!=current_job: replenished=True
        if now>17 and not seed_check: seed_check=any(math.isfinite(x) for x in lidar.getRangeImage())
        if os.environ.get('G18_CAPTURE'):
            for at,action in ((7,'detail'),(8.5,'dispatch'),(10,'reset'),(18,'overview')):
                if now<at or action in captured: continue
                captured.add(action)
                if action in ('detail','reset'):
                    eye,axis=camera_view(1 if action=='detail' else 0); view=robot.getFromDef('HALL_CAMERA')
                    view.getField('position').setSFVec3f(list(eye)); view.getField('orientation').setSFRotation(list(axis))
                else: robot.exportImage(str(output.parent/(action+'.jpg')),91)
            if current_job and lane and lane['id']==current_job:
                status=lane['status']
                if status=='in_transit' and status not in movements:
                    robot.exportImage(str(output.parent/'carrying.jpg'),91); movements.add(status)
                if status=='delivered' and 'dock_cam' not in movements:
                    eye,axis=camera_view(3,None,CFG['docks'][0]); view=robot.getFromDef('HALL_CAMERA')
                    view.getField('position').setSFVec3f(list(eye)); view.getField('orientation').setSFRotation(list(axis))
                    movements.add('dock_cam'); dock_at=now
                if status=='delivered' and 'delivery' not in movements and now-dock_at>.3:
                    robot.exportImage(str(output.parent/'delivery.jpg'),91); movements.add('delivery')
        speed=spin=0.
        if current_job and phase<len(route):
            op,target,sign=route[phase]; done=False
            if op=='lift':
                if now-since>.55: lift.setPosition(target)
                done=now-since>2.7 and abs(sensor.getValue()-target)<.017
            else:
                dx,dy=target[0]-pos[0],target[1]-pos[1]; distance=math.hypot(dx,dy)
                desired=math.atan2(sign*dy,sign*dx); error=math.atan2(math.sin(desired-yaw),math.cos(desired-yaw))
                done=distance<.029
                if not done:
                    spin=max(-.43,min(.43,2.0*error))
                    if abs(error)<.24: speed=sign*min(.55,max(.034,.58*distance))*max(0.,math.cos(error))
            if done:
                phase+=1; since=now; speed=spin=0.; samples.append([round(now,2),phase,pose]); print('[QA] phase',phase,pose,flush=True)
        for index,stage in enumerate(stages,1): stage.setPosition(max(0.,sensor.getValue())*index/3)
        motors[0].setVelocity((speed-spin*.41)/.15); motors[1].setVelocity((speed+spin*.41)/.15)
        if replenished and len(staged)==len(CFG['trucks']):
            arrived=phase==len(route) and {'ready','in_transit','delivered'}<=states
            traction=traction_trials(robot,step,body,motors,tx)
            slip=traction['wet']['encoder_m']-traction['wet']['actual_m']
            dry=all(abs(traction[key]['encoder_m']-traction[key]['actual_m'])<.085 for key in ('dry','recovery'))
            finish(arrived and dry and slip>.13 and painted and all(fades.values()) and len(set(berth_checks))==30 and ack and seed_check and readings>100,'delivery, replenishment and traction trials'); return
        if now>620:
            diag={}
            if current_job and robot.getFromDef(current_job):
                node=robot.getFromDef(current_job); diag={'load':node.getPosition(),'orientation':node.getOrientation(),'lift':sensor.getValue()}
            finish(False,'timeout '+str(diag)); return


if __name__=='__main__': main()
