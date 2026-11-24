"""Run the production facility and a wheel-and-lift acceptance drive in an isolated Webots copy."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--webots',default=shutil.which('webots') or r'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe')
    parser.add_argument('--capture',action='store_true'); parser.add_argument('--timeout',type=int,default=210)
    parser.add_argument('--localization', action='store_true', help='Record a paired P03 to D01 localization trial.')
    parser.add_argument('--mapping', action='store_true', help='Map the P03 to D01 trial with onboard sensors.')
    parser.add_argument('--docking', action='store_true', help='Verify camera-guided entry at P03 and D01.')
    parser.add_argument('--approach-offset',type=float,default=0,choices=(-.15,0.,.15),
                        help='Lateral handoff variation for visual docking trials (metres).')
    args=parser.parse_args()
    if args.docking:
        args.localization = True
    if args.mapping:
        args.localization = True
    if not Path(args.webots).is_file(): parser.error('Pass --webots with the executable path.')
    dest=ROOT.parent/'.runtime'/('final-smoke-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    for folder in ('protos','config','controllers/final_warehouse','textures/final_hall'):
        shutil.copytree(ROOT/folder,dest/folder,ignore=shutil.ignore_patterns('__pycache__'))
    if args.localization:
        for module in ('odometry.py', 'live_pose.py', 'mapping.py', 'live_mapping.py') if args.mapping else ('odometry.py', 'live_pose.py'):
            (dest/'localization').mkdir(exist_ok=True)
            shutil.copyfile(ROOT/'localization'/module, dest/'localization'/module)
    (dest/'worlds').mkdir(); (dest/'tools').mkdir(); probe=dest/'controllers/final_robot_probe'; probe.mkdir()
    shutil.copyfile(ROOT/'tools/build_integration_world.py',dest/'tools/build_integration_world.py')
    shutil.copyfile(ROOT/'tests/final_robot_probe.py',probe/'final_robot_probe.py')
    if args.docking:
        shutil.copyfile(ROOT/'controllers/pallet_robot/docking.py',probe/'docking.py')
    world=dest/'worlds/final_warehouse.wbt'; text=(ROOT/'worlds'/world.name).read_text(encoding='utf-8')
    world.write_text(text.replace('DEF TRANSPORT_ROBOT PalletRobot {','DEF TRANSPORT_ROBOT PalletRobot { controller "final_robot_probe"'),encoding='utf-8')
    robot=dest/'protos/PalletRobot.proto'; text=robot.read_text(); robot.write_text(text.replace('\nRobot {','\nRobot {\n  supervisor TRUE'),encoding='utf-8')
    startup=None
    if os.name=='nt':
        startup=subprocess.STARTUPINFO(); startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW; startup.wShowWindow=0
    demo_seed=json.loads((ROOT/'config/final_warehouse.json').read_text())['demo_seed']
    env=dict(os.environ,G18_WAREHOUSE_SEED=str(demo_seed)); options=[] if args.capture else ['--no-rendering']
    if args.capture: env['G18_CAPTURE']='1'
    if args.localization: env['G18_LOCALIZATION_TRIAL']='1'
    if args.mapping: env['G18_MAPPING_TRIAL']='1'
    if args.docking:
        env['G18_DOCKING_TRIAL']='1'
        env['G18_APPROACH_OFFSET']=str(args.approach_offset)
    print('Isolated Webots run:',dest,flush=True)
    with (dest/'webots.log').open('w',encoding='utf-8') as log:
        result=subprocess.run([args.webots,'--batch','--minimize','--mode=fast','--stdout','--stderr','--port=1283',*options,str(world)],
            env=env,stdout=log,stderr=subprocess.STDOUT,startupinfo=startup,timeout=args.timeout)
    result_file=dest/'probe.json'
    if not result_file.exists(): raise RuntimeError('Controllers did not finish. Inspect '+str(dest/'webots.log'))
    report=json.loads(result_file.read_text()); print(f"Robot: {report['reason']}; {report['packets']} status packets, {report['berth_checks']} physical berth checks. Full result: {result_file}")
    if args.localization:
        if result.returncode != 0 or not report['passed']:
            raise RuntimeError(f'Localization trial failed; see {dest}')
        print(f'Paired poses: {dest / "paired.csv"}')
        return
    event_file=next((dest/'results/final').glob('*/events.jsonl')); events=[json.loads(row) for row in event_file.read_text().splitlines()]
    assert result.returncode==0 and report['passed'],dest
    assert {r['id'] for r in events if r['event']=='stock_withdraw'}=={'FLT_N','FLT_S'},'Both stock shuttles must deliver.'
    assert any(r['event']=='gate' and r['id']=='G05' and r['state']=='open' for r in events)
    assert any(r['event']=='gate' and r['id'] in ('G02','G03') and r['state']=='closed' for r in events)
    assert any(r['event']=='spill_state' and r['reported'] for r in events)
    print('PASS: physical pickup, loaded drive, delivery/replenishment, radio, LiDAR, stock transfers, gates and reports.')


if __name__=='__main__': main()
