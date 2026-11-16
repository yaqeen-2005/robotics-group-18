"""Build only integration_test.wbt and its evaluation layout manifest."""
import json
import hashlib
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def label_texture(text):
    folder=ROOT/'textures/integration_labels'
    folder.mkdir(parents=True,exist_ok=True)
    filename=hashlib.sha256(text.encode()).hexdigest()[:12]+'.png'
    path=folder/filename
    if not path.exists():
        from PIL import Image, ImageDraw, ImageFont
        font=ImageFont.load_default(size=48)
        image=Image.new('RGBA',(1024,128),(0,0,0,0))
        draw=ImageDraw.Draw(image)
        draw.text((512,64),text,font=font,fill=(25,36,45,255),anchor='mm')
        image.save(path)
    return '../textures/integration_labels/'+filename


def vec(values):
    return ' '.join(str(v) for v in values)


def shape(size, color):
    return f'Shape {{ appearance PBRAppearance {{ baseColor {vec(color)} roughness 0.8 metalness 0 }} geometry Box {{ size {vec(size)} }} }}'


def build(config):
    parts = ['''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/PalletRobot.proto"
IMPORTABLE EXTERNPROTO "../protos/TestLoad.proto"
WorldInfo {
  title "Group 18 - integration test warehouse"
  basicTimeStep 16
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties { material1 "support" material2 "floor" coulombFriction [ 0.01 ] }
    ContactProperties { material1 "tyre" material2 "floor" coulombFriction [ 1 ] }
  ]
}
Viewpoint { orientation 0 1 0 1.57079632679 position 0 0 34 fieldOfView 0.95 description "Integration warehouse overview" }
Background { skyColor [ 0.72 0.80 0.88 ] }
DirectionalLight { direction -0.4 0.3 -1 intensity 1 castShadows FALSE }
''']
    obstacles = []

    def box(name, xyz, size, color=(0.55, 0.60, 0.65), collision=True, def_name=None, floor=False):
        parts.append(f'''{'DEF '+def_name+' ' if def_name else ''}Solid {{
  translation {vec(xyz)} name "{name}"
  {'contactMaterial "floor"' if floor else ''}
  children [ {shape(size, color)} ]
  {'boundingObject Box { size '+vec(size)+' }' if collision else ''}
}}
''')
        if collision and not floor:
            obstacles.append(dict(name=name, center=xyz[:2], size=size[:2]))

    def sign(text, x, y, size=0.24):
        # Webots has no VRML Text node. Use local PNG floor decals instead.
        parts.append(f'''Pose {{ translation {x} {y} 0.012 children [ Shape {{
 appearance PBRAppearance {{ baseColorMap ImageTexture {{ url [ "{label_texture(text)}" ] }} roughness 1 metalness 0 }}
 geometry Plane {{ size {size*12} {size*1.5} }}
}} ] }}
''')

    def stand(name, x, y, color):
        box(name+'_pad', [x,y,0.002], [2.2,2.1,0.004], color, False)
        for side in (-1,1):
            box(name+str(side), [x,y+side*.54,.24], [2.1,.12,.48], color)
        sign(name.upper().replace('_',' '), x, y-1.35)

    box('floor', [0,0,-.1], [24.3,18.3,.2], (.73,.75,.76), floor=True)
    for name, xyz, size in [
        ('west',[-12.075,0,1.2],[.15,18.3,2.4]),
        ('east',[12.075,0,1.2],[.15,18.3,2.4]),
        ('south',[0,-9.075,1.2],[24,.15,2.4]),
        ('north',[0,9.075,1.2],[24,.15,2.4]),
        ('divider_west',[-9.4,0,.9],[5.2,.2,1.8]),
        ('divider_middle',[-.4,0,.9],[9.6,.2,1.8]),
        ('divider_east',[9.8,0,.9],[4.4,.2,1.8]),
    ]:
        box(name,xyz,size)
    for name,x,width in [('SHORTCUT 1.60 m',-6,1.6),('WIDE ROUTE 3.20 m',6,3.2)]:
        box(name, [x,0,.004],[width,1,.008],(.4,.65,.7),False)
        sign(name,x,.6,.20)

    # Shelves have solid low plinths so they are observable at the robot's lidar height.
    for n,(x,y,l,w) in enumerate([(-9,-3.3,3,.8),(-1,-3.3,3,.8),(-2,3.3,3,.8),(3.5,3.3,2,.8),(10.8,-2.8,1,3)]):
        box(f'rack_{n}_plinth',[x,y,.2],[l,w,.4],(.2,.25,.3))
        for dx in (-l/2+.07,l/2-.07):
            for dy in (-w/2+.07,w/2-.07):
                box(f'rack_{n}_post_{dx}_{dy}',[x+dx,y+dy,1.25],[.14,.14,2.5],(.25,.35,.45))
        for z in (.5,1.35,2.2):
            box(f'rack_{n}_shelf_{z}',[x,y,z],[l,w,.10],(.85,.35,.12))
            box(f'rack_{n}_goods_{z}',[x,y,z+.25],[l-.3,w-.2,.4],(.60,.47,.30))

    # Extra isolated objects and an enclosed dead end for mapping/recovery tests.
    for name,xyz,size,color in [
        ('crate',[-10,6.5,.4],[.8,.8,.8],(.55,.35,.2)),
        ('column',[10.5,4,.9],[.4,.4,1.8],(.4,.45,.5)),
        ('dead_end_left',[.3,-8,.5],[.15,1.8,1],(.5,.55,.6)),
        ('dead_end_right',[2.7,-8,.5],[.15,1.8,1],(.5,.55,.6)),
        ('dead_end_back',[1.5,-8.8,.5],[2.4,.15,1],(.5,.55,.6)),
    ]:
        box(name,xyz,size,color)
    sign('DEAD END',1.5,-8)
    box('waiting_pad',[-10.8,-8,.004],[1.4,1.2,.008],(.25,.55,.65),False)
    sign('WAIT',-10.8,-8)

    for profile in config['loads']:
        x,y = profile['station']
        stand(profile['id'],x,y,profile['color'])
        parts.append(load_node(profile, 'LOAD_'+profile['id'].upper(), [x,y,.52]))
    for n,(x,y) in enumerate(config['delivery_stations']):
        stand('delivery_'+str(n+1),x,y,(.2,.45,.8))
    x,y=config['output_station']
    stand('storage_output',x,y,(.2,.7,.6))
    box('storage_facade',[-11.5,3,1.1],[.4,3.3,2.2],(.25,.3,.35))
    # Conveyor styling behind the pickup point; pickup itself retains open rails.
    box('output_conveyor',[-10.7,3,.38],[1.0,1.3,.16],(.45,.48,.5))
    sign('AUTOMATED STORAGE',-10,5.1,.20)

    for i,spill in enumerate(config['spills'],1):
        x,y=spill['center']; sx,sy=spill['size']
        z=.007 if spill['present'] else -1
        color=(.85,.15,.08) if spill['reported'] else (.8,.55,.05)
        parts.append(f'''DEF SPILL_{i} Pose {{ translation {x} {y} {z} children [ Shape {{
 appearance DEF SPILL_COLOR_{i} PBRAppearance {{ baseColor {vec(color)} roughness 0.25 metalness 0 }}
 geometry Box {{ size {sx} {sy} 0.008 }}
}} ] }}
''')
        sign(f'SPILL {i}',x,y+sy/2+.25,.18)

    for actor in config['actors']:
        x,y=actor['start']
        angle=math.atan2(actor['end'][1]-y,actor['end'][0]-x)
        visuals=[]; bounds=[]
        def piece(xyz,size,color):
            visuals.append(f'Pose {{ translation {vec(xyz)} children [ {shape(size,color)} ] }}')
            bounds.append(f'Pose {{ translation {vec(xyz)} children [ Box {{ size {vec(size)} }} ] }}')
        if actor['kind']=='person':
            for side in (-1,1):
                piece([0,side*.14,.38],[.24,.18,.76],(.12,.18,.25))
            piece([0,0,1.08],[.32,.5,.65],(.95,.65,.04))
            visuals.append('Pose { translation 0 0 1.58 children [ Shape { appearance PBRAppearance { baseColor 0.75 0.55 0.4 roughness 1 metalness 0 } geometry Sphere { radius 0.17 } } ] }')
            bounds.append('Pose { translation 0 0 1.58 children [ Sphere { radius 0.17 } ] }')
        else:
            piece([-.25,0,.55],[1.5,1.05,.9],(.95,.65,.03))
            piece([.55,0,1.1],[.15,1,2.1],(.18,.22,.25))
            for side in (-1,1):
                piece([1,side*.34,.12],[1,.12,.12],(.2,.22,.25))
                piece([-.4,side*.52,.24],[1.1,.16,.48],(.06,.06,.06))
            piece([-.25,0,1.9],[1.3,1.1,.1],(.3,.32,.35))
        parts.append(f'''DEF {actor['id']} Solid {{
 translation {x} {y} 0 rotation 0 0 1 {angle} name "{actor['id']}"
 children [ {' '.join(visuals)} ]
 boundingObject Group {{ children [ {' '.join(bounds)} ] }}
}}
''')

    # Raised gates leave corridors open. Manager lowers only into an empty area.
    for name,x,width in [('NARROW_GATE',-6,1.6),('WIDE_GATE',6,3.2)]:
        box(name,[x,0,4],[width,.25,1.1],(.9,.3,.08),True,name)
        obstacles.pop()  # Dynamic gates excluded from static evaluation layout.
    parts.append(f'''DEF TRANSPORT_ROBOT PalletRobot {{
 translation {vec(config['robot_start'])}
 extensionSlot [
   Receiver {{ name "warehouse_events" channel 18 }}
   Emitter {{ name "warehouse_commands" channel 19 }}
 ]
}}
DEF WAREHOUSE_MANAGER Robot {{
 supervisor TRUE name "warehouse_manager" controller "warehouse_manager"
 children [ Emitter {{ name "events" channel 18 }} Receiver {{ name "commands" channel 19 }} ]
}}
''')
    return ''.join(parts), obstacles


def load_node(profile, def_name, position):
    size = profile['deck_size']; h=profile['cargo_height']
    return f'''DEF {def_name} TestLoad {{
 translation {vec(position)} name "{def_name.lower()}"
 deckSize {vec(size)} cargoSize {size[0]-.08} {size[1]-.08} {h}
 cargoOffset 0 0 {size[2]/2+h/2} mass {profile['mass_kg']} color {vec(profile['color'])}
}}
'''


if __name__ == '__main__':
    config=json.loads((ROOT/'config/integration_scenario.json').read_text())
    world, obstacles=build(config)
    (ROOT/'worlds/integration_test.wbt').write_text(world,encoding='utf-8')
    (ROOT/'config/integration_layout_evaluation.json').write_text(json.dumps({
        'purpose':'Ground truth for scenario validation only. Do not feed to SLAM/navigation.',
        'obstacles':obstacles},indent=2)+'\n',encoding='utf-8')
    catalog={}
    for p in config['loads']:
        length,width,_=p['deck_size']
        combined=[max(.98,length),max(.84,width)]
        catalog[p['id']]=dict(deck_size_m=p['deck_size'],cargo_height_m=p['cargo_height'],
            total_height_m=.08+p['cargo_height'],total_mass_kg=p['mass_kg'],handling=p['handling'],
            centered_combined_rectangle_m=combined,
            centered_clearance_radius_m=math.hypot(*combined)/2+.05)
    (ROOT/'config/load_catalog.json').write_text(json.dumps(dict(
        assumptions='Load centred on robot axle; 0.05 m margin. Recompute for attachment offsets.',
        profiles=catalog),indent=2)+'\n',encoding='utf-8')
    print('Built integration_test.wbt; original warehouse.wbt untouched.')
