"""Rebuild just the final Webots world, local artwork and its operator floor plan."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'controllers/final_warehouse'))
from layout import CFG, FORK_OFFSET, GATE_COIL, GATE_HEAD_HEIGHT, SLATS, SLAT_HEIGHT, camera_view, fixtures, gates, stations
from final_models import bounds, box, cylinder, fixture_model, materials, person, pose, shape, sign, solid, stock_load, surface, truck, vec


def marking(at, size, material):
    return box((*at,.006),(*size,.010),material)


def outline(x,y,length,width,material):
    return ''.join(marking((x,y+side*width/2),(length,.08),material)+marking((x+side*length/2,y),(.08,width),material) for side in (-1,1))


def build():
    parts=['''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/PalletRobot.proto"
IMPORTABLE EXTERNPROTO "../protos/TestLoad.proto"
WorldInfo { title "G18 | Meridian distribution hall" basicTimeStep 16 coordinateSystem "ENU"
  contactProperties [ ContactProperties { material1 "support" material2 "floor" coulombFriction [ 0.01 ] }
                      ContactProperties { material1 "tyre" material2 "floor" coulombFriction [ 1 ] }
                      ContactProperties { material1 "wet_tyre" material2 "floor" coulombFriction [ 0.018 ] forceDependentSlip [ 0.12 ] bounce 0 } ] }
''',f'DEF HALL_CAMERA Viewpoint {{ position {vec(camera_view(0)[0])} orientation {vec(camera_view(0)[1])} fieldOfView 1.08 near .08 far 280 description "Meridian | hall overview" }}',
    'Background { skyColor [ .22 .30 .33 ] }',
    'DirectionalLight { direction -.35 .40 -.91 intensity 1.12 ambientIntensity .65 castShadows TRUE }',
    'DirectionalLight { direction .6 -.5 -.65 intensity .48 color .88 .94 1 castShadows FALSE }',materials()]
    parts.append(solid('HALL_SLAB',(0,0,-.12),shape('Box { size 96.5 64.5 .24 }','FLOOR'), 'Box { size 96.5 64.5 .24 }',material='floor'))
    shared={}; obstacles=fixtures()
    for index,item in enumerate(obstacles):
        key=(item.kind,item.length,item.width,item.height,index%4 if item.kind in ('rack','island') else 0)
        if key in shared:
            model='USE '+shared[key]
        else:
            node=f'ASSEMBLY_{index:03}'; shared[key]=node
            model=f'DEF {node} Group {{ children [ {fixture_model(item,index%4)} ] }}'
        parts.append(solid('FIX_'+item.name.replace('-','M').replace('.','_'),(item.x,item.y,0),model,bounds((0,0,item.height/2),(item.length,item.width,item.height)),item.angle))

    for x in (-36.6,42):
        for y in range(-29,31,3):
            if all(item.distance((x,y))>.94 for item in obstacles): parts.append(marking((x,y),(.10,1.5),'WHITE'))
    for y in (-2.8,2.8):
        for x in range(-30,19,4):
            if all(item.distance((x,y))>.94 for item in obstacles): parts.append(marking((x,y),(1.45,.10),'WHITE'))
    for sector, label, x,y in [('N','stock_n',27.5,28.8),('S','stock_s',27.5,-29.3)]:
        parts.append(sign(label,(x,y,.016),(7.0,1.75)))
    for spec in CFG['trucks']:
        for edge in (22.0,33.0):
            low,high=sorted(spec['ends'])
            for y in range(math.ceil(low),math.floor(high),3): parts.append(marking((edge,y),(.10,1.9),'GOLD'))
        for aisle in spec['aisles']:
            start,end=sorted(spec['ends'])
            for i in range(math.ceil((end-start)/3)):
                parts.append(marking((aisle,start+1.2+i*3),(.085,1.15),'MARK'))
        for index,y in enumerate(spec['ends']):
            x=spec['dock_x']+FORK_OFFSET
            parts.append(outline(x,y,1.68,1.65,'MARK')+sign('stock',(x+1.35,y,.016),(1.52,.38),(0,0,1,math.pi/2)))
            deck=box((0,0,0),(1.64,1.95,.14),'AMBER')+box((.13,0,.40),(1.10,1.39,.66),'BOX1')
            parts.append(solid(spec['id']+'_SHELF_'+str(index),(x,y,2.46),deck,bounds((0,0,.33),(1.64,1.95,.80))))
            parts.append(sign('stock',(x-.826,y,2.44),(1.64,.36),(1,-1,-1,2*math.pi/3)))
        parts.extend((truck(spec),stock_load(spec,FORK_OFFSET)))
    for index,spec in enumerate(CFG['people']): parts.append(person(spec,index))
    for y in (-18,13.5):
        for x in (22.7,26.2,29.6,33):
            for offset in (-.48,0,.48): parts.append(marking((x+offset,y),(.21,1.6),'BONE'))
    for station in stations():
        ident=station['id']; x,y=station['at']; side=station['entry']; pickup=ident.startswith('P')
        parts.append(sign(ident.lower(),(x,y-1.93,.019),(2.95,.7375)))
        parts.append(marking((x+side*2.75,y-.6),(.54,.08),'BONE')+marking((x+side*2.75,y+.6),(.54,.08),'BONE'))
        beacon=ident+'_LAMP'
        app=surface((.18,.31,.34),.12,.42,emission=(0,0,0))
        lamp=f'Shape {{ appearance DEF {beacon} {app} geometry Box {{ size .18 2.72 .04 }} }}'
        parts.append(pose((x+side*1.46 if pickup else x-side*1.46,y,.032),lamp))
        for offset in (-1,1):
            parts.append(pose((x+(.08 if pickup else -.08)*side,y+offset*1.34,.032),shape('Box { size 2.58 .14 .04 }',beacon)))
        if pickup:
            continue
        board=box((0,0,0),(.22,2.64,.62),'NAVY')
        for normal in (side,-side):
            face=(1,normal,normal,2*math.pi/3)
            board+=sign(ident.lower(),(normal*.122,0,0),(2.49,.55),face)
        board+=box((0,0,.35),(.22,2.64,.08),beacon)
        parts.append(solid(ident+'_BOARD',(x-side*.86,y,2.75),board,bounds((0,0,.04),(.22,2.64,.70))))
    feed=CFG['dispatch']; wall=feed['wall_x']
    for index,y in enumerate(feed['lanes_y'],1):
        span=2.64; bottom=2.05; height=4.1-bottom
        lintel=box((0,0,0),(.35,span,height),'NAVY')+box((.18,0,-.80),(.035,span,.16),'GOLD')
        parts.append(solid(f'PORTAL_{index}',(wall,y,bottom+height/2),lintel,'Box { size '+vec((.35,span,height))+' }'))
        for side in (-1,1):
            parts.append(box((wall+.17,y+side*1.36,1.1),(.07,.12,2.2),'GOLD'))
        parts.append(sign(f'p{index:02}',(wall+.19,y,2.99),(2.5,.625),(1,1,1,2*math.pi/3)))
        parts.append(pose((wall+.19,y,2.32),f'Shape {{ appearance DEF P{index:02}_SIGNAL {surface((.18,.35,.31))} geometry Box {{ size .045 1.8 .12 }} }}'))
        cover=box((0,0,0),(4.65,4.83,.12),'TEAL')
        parts.append(pose((wall-2.5,y,4.11),cover))
    parts.append(sign('asrs',(-44.95,-13,4.22),(10,2.5),(0,0,1,math.pi/2)))
    px,py=CFG['parking']; parts.append(outline(px,py,3.8,2.12,'TEAL')+sign('park',(px,py-1.48,.019),(3.8,.95)))
    for item in gates():
        ident=item['id']; width=item['w']; x,y=item['x'],item['y']
        for index in range(SLATS):
            height=GATE_COIL if item['mode']=='timed' else (index+.5)*SLAT_HEIGHT
            slat=box((0,0,0),(width,.24,SLAT_HEIGHT),'PANEL')+box((0,0,-.26),(width,.25,.09),'GOLD' if index in (0,1) else 'STEEL')
            if index==3:
                for side in (-1,1): slat+=box((0,side*.13,0),(width*.84,.025,.33),'WINDOW')
            parts.append(solid(ident if index==0 else ident+f'_SLAT_{index}',(x,y,height),slat,'Box { size '+vec((width,.26,SLAT_HEIGHT))+' }'))
        casing=box((0,0,0),(width+.64,.62,GATE_HEAD_HEIGHT),'INK')
        lamp=f'Shape {{ appearance DEF {ident}_LIGHT {surface((.3,.5,.4))} geometry Box {{ size {width*.88} .026 .10 }} }}'
        parts.append(solid(ident+'_HEAD',(x,y,GATE_COIL),casing+pose((0,-.318,.16),lamp)+pose((0,.318,.16),shape('Box { size '+vec((width*.88,.026,.10))+' }',ident+'_LIGHT')),'Box { size '+vec((width+.64,.62,GATE_HEAD_HEIGHT))+' }'))
        parts.append(sign(ident.lower(),(x,y+.8,.020),(min(3.6,width),.90)))
    for site in CFG['spills']:
        x,y=site['center']; width,height=site['size']; ident=site['id'].upper()
        art=f'Shape {{ castShadows FALSE appearance DEF {ident}_FILM {surface((1,1,1),0,.67,"spill")} geometry Plane {{ size {vec((width,height))} }} }}'
        parts.append(pose((x,y,-2),art,ident=ident))
        marker='Box { size '+vec((width,.045,.008))+' }'
        rim=pose((0,-height*.55,0),f'Shape {{ appearance DEF {ident}_MARK {surface((.97,.42,.11))} geometry {marker} }}')+pose((0,height*.55,0),shape(marker,ident+'_MARK'))
        parts.append(pose((x,y,-2),rim,ident=ident+'_REPORT'))
    for label,x,y in [('aisle_a',-17,-3.9),('aisle_b',-17,4.15),('bypass',42,2.7),('bypass',-37,2.7),('short',-4,2.3),('cart',3,2.3),('bulk',8.5,2.3)]:
        parts.append(sign(label,(x,y,.018),(4.0,1)))
    for station in stations():
        x,y = station['at']
        entry = station['entry']
        for suffix, side in [('L',-1),('R',1)]:
            label = station['id']+'_'+suffix
            parts.append(f'Solid {{ name "{label}" model "{label}" translation {x-.70*entry} {y+.24*side*entry} 2.25 recognitionColors [ 0.1 0.8 0.6 ] children [ Shape {{ appearance USE TEAL geometry Box {{ size 0.05 0.08 0.08 }} }} ] }}')
    parts.extend([f'''DEF TRANSPORT_ROBOT PalletRobot {{ translation {px} {py} .16 rotation 0 0 1 -1.570796327
 extensionSlot [ Receiver {{ name "warehouse_events" channel 18 }} Emitter {{ name "warehouse_commands" channel 19 }} ] }}''',
    'DEF FINAL_MANAGER Robot { name "final_warehouse" supervisor TRUE controller "final_warehouse" children [ Emitter { name "events" channel 18 } Receiver { name "commands" channel 19 } ] }'])
    return '\n'.join(parts)+'\n'


def main():
    from final_art import GOLD, plaque, plan_view, surfaces
    surfaces()
    for station in stations():
        ident=station['id']; title=ident+'  /  '+('OUTFEED' if ident[0]=='P' else station['name'].upper())
        plaque(ident.lower(),title,'MERIDIAN   |   '+('AUTOMATED STORAGE' if ident[0]=='P' else 'UNIVERSAL RECEIVING'),GOLD if ident[0]=='P' else '#62c6f0')
    for gate in gates(): plaque(gate['id'].lower(),gate['id']+'   /   GATE',gate['mode'].upper()+'   |   KEEP CLEAR',GOLD)
    for key,title,sub in [('park','AMR  /  01','RESERVED PARKING'),('asrs','MERIDIAN  /  18','AS/RS   |   AUTOMATED OUTFEED'),
        ('stock','FLT ONLY','STOCK TRANSFER'),('stock_n','STOCK  /  NORTH','LIFT TRUCK   |   N SECTOR'),('stock_s','STOCK  /  SOUTH','LIFT TRUCK   |   S SECTOR'),
        ('aisle_a','HALL  /  A','SOUTH RESERVE STORAGE'),('aisle_b','HALL  /  B','NORTH PICK MODULE'),('bypass','SERVICE ROAD','6.0 m   |   PERIMETER'),('short','LIGHT ACCESS','1.65 m   |   CHECK LOAD'),('cart','CART ACCESS','1.95 m   |   CHECK LOAD'),('bulk','BULK ACCESS','2.65 m   |   CHECK LOAD')]: plaque(key,title,sub)
    target=ROOT/'worlds/final_warehouse.wbt'
    target.write_bytes(build().encode('utf-8'))
    plan_view(ROOT/'docs/final_warehouse_plan.png',CFG,fixtures(),stations())
    print('Wrote',target)


if __name__=='__main__':
    main()
