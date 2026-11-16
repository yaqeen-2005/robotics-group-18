"""Test-only controller, installed into an isolated copy by smoke_integration.py."""
import json
import math
from pathlib import Path
from controller import Robot

robot=Robot()
step=int(robot.getBasicTimeStep())
rx=robot.getDevice('warehouse_events'); rx.enable(step)
tx=robot.getDevice('warehouse_commands')
lidar=robot.getDevice('front_lidar'); lidar.enable(step)
for name in ('left_wheel_motor','right_wheel_motor'):
    motor=robot.getDevice(name)
    motor.setPosition(float('inf')); motor.setVelocity(0)
robot.getDevice('lift_motor').setPosition(0)
received=0
ack=False
sent=False
report_sets=set()
while robot.step(step)!=-1:
    if not sent:
        tx.send(json.dumps(dict(op='report',id='spill_1',value=False,request_id='probe')).encode())
        sent=True
    while rx.getQueueLength():
        message=json.loads(rx.getString())
        if message['type']=='warehouse_status':
            received+=1
            assert 'robot_pose' not in message and 'actors' not in message
            report_sets.add(tuple(sorted(s['id'] for s in message['reported_spills'])))
        elif message.get('request_id')=='probe':
            assert message['ok']
            ack=True
        rx.nextPacket()
    if robot.getTime()>=24:
        assert ack and received>20
        assert ('spill_2',) in report_sets
        assert ('spill_1','spill_2','spill_3') in report_sets
        assert any(math.isfinite(x) for x in lidar.getRangeImage())
        Path('interface_result.json').write_text(json.dumps(dict(passed=True,received=received,ack=ack)))
