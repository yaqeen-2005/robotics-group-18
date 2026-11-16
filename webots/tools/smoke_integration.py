"""Run a 25-s scenario smoke test in an isolated copy; never edits source worlds."""
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
    parser.add_argument('--webots',default=shutil.which('webots') or
                        r'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe')
    args=parser.parse_args()
    if not Path(args.webots).is_file():
        parser.error('Webots executable not found; pass --webots PATH.')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    dest=ROOT.parent/'.runtime'/('integration-smoke-'+stamp)
    for folder in ('protos','worlds','controllers','tools','config','textures'):
        shutil.copytree(ROOT/folder,dest/folder,ignore=shutil.ignore_patterns('__pycache__','.*.wbproj'))
    probe=dest/'controllers/warehouse_interface_probe'
    probe.mkdir()
    shutil.copyfile(ROOT/'tests/warehouse_interface_probe.py',probe/'warehouse_interface_probe.py')
    world=dest/'worlds/integration_test.wbt'
    text=world.read_text(encoding='utf-8')
    text=text.replace('controller "warehouse_manager"','controller "warehouse_manager" controllerArgs [ "--smoke" ]')
    text=text.replace('DEF TRANSPORT_ROBOT PalletRobot {','DEF TRANSPORT_ROBOT PalletRobot { controller "warehouse_interface_probe"')
    world.write_text(text,encoding='utf-8')
    startup=None
    if os.name=='nt':
        startup=subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow=0
    print('Smoke test workspace:',dest,flush=True)
    with (dest/'webots.log').open('w',encoding='utf-8') as log:
        completed=subprocess.run([args.webots,'--batch','--minimize','--mode=fast',
            '--no-rendering','--stdout','--stderr','--port=1248',str(world)],
            stdout=log,stderr=subprocess.STDOUT,startupinfo=startup,timeout=60)
    # Windows launcher stdout can truncate at shutdown; use flushed result files.
    latest=json.loads((dest/'results/integration/latest.json').read_text())
    events=(Path(latest['directory'])/'events.jsonl').read_text()
    interface=json.loads((probe/'interface_result.json').read_text())
    if completed.returncode or 'SMOKE PASS:' not in events or not interface['passed']:
        raise RuntimeError('Smoke test failed; inspect '+str(dest/'webots.log'))
    print('PASS: scenario controls, timed gate, traffic, pallet settling/spawn, radio interface and LiDAR.')


if __name__=='__main__':
    main()
