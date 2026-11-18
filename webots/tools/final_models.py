"""Small industrial models with shared materials and simple collision solids."""
import math
import random


def vec(values):
    return ' '.join(format(value, '.7g') for value in values)


def pose(at, contents, turn=None, ident=''):
    rotation = ' rotation ' + vec(turn) if turn else ''
    return f'{("DEF " + ident + " ") if ident else ""}Pose {{ translation {vec(at)}{rotation} children [ {contents} ] }}'


def shape(geometry, material):
    return f'Shape {{ appearance USE {material} geometry {geometry} }}'


def box(at, size, material, turn=None):
    return pose(at, shape('Box { size ' + vec(size) + ' }', material), turn)


def cylinder(at, radius, height, material, turn=None, sides=12):
    return pose(at, shape(f'Cylinder {{ radius {radius} height {height} subdivision {sides} }}', material), turn)


def ball(at, scale, material):
    return f'Transform {{ translation {vec(at)} scale {vec(scale)} children [ {shape("Sphere { radius 1 subdivision 1 }", material)} ] }}'


def bounds(at, size, turn=None):
    return pose(at, 'Box { size ' + vec(size) + ' }', turn)


def solid(ident, at, visuals, collider='', yaw=0., material=''):
    bound = ' boundingObject ' + collider if collider else ''
    contact = f' contactMaterial "{material}"' if material else ''
    return f'DEF {ident} Solid {{ name "{ident.lower()}" translation {vec(at)} rotation 0 0 1 {yaw:.7g}{contact} children [ {visuals} ]{bound} }}'


def surface(base, metal=0., rough=.73, texture=None, repeat=None, emission=None):
    fields = f'baseColor {vec(base)} metalness {metal} roughness {rough}'
    if texture:
        fields += f' baseColorMap ImageTexture {{ url [ "../textures/final_hall/{texture}.png" ] }}'
    if repeat:
        fields += ' textureTransform TextureTransform { scale ' + vec(repeat) + ' }'
    if emission:
        fields += ' emissiveColor ' + vec(emission)
    return 'PBRAppearance { ' + fields + ' }'


def materials():
    colors={'INK':(.075,.13,.16),'NAVY':(.10,.22,.29),'TEAL':(.05,.50,.48),'GOLD':(.95,.65,.14),
            'AMBER':(.97,.42,.11),'BONE':(.84,.85,.79),'WHITE':(.92,.94,.87),'STEEL':(.55,.61,.61),
            'RUBBER':(.055,.068,.072),'WOOD':(.56,.39,.22),'CYAN':(.12,.66,.73),'CONE':(.99,.34,.06),
            'JEANS':(.12,.19,.25),'VEST':(.86,.92,.19),'SKIN0':(.75,.50,.32),'SKIN1':(.46,.28,.19),
            'WINDOW':(.13,.34,.40),'LIME':(.32,.82,.53),'MARK':(.79,.65,.30),'SILVER':(.68,.75,.76)}
    library={key:surface(color, .40 if key in ('STEEL','SILVER') else .04) for key,color in colors.items()}
    library['FLOOR']=surface((.91,.94,.95),0,.83,'concrete',(16,10.6667))
    library['PANEL']=surface((.75,.84,.85),.22,.58,'cladding',(2,1))
    for index in range(3): library['BOX'+str(index)]=surface((1,1,1),0,.9,'carton'+str(index))
    return pose((0,0,-5),' '.join(f'Shape {{ appearance DEF {key} {value} geometry Box {{ size .01 .01 .01 }} }}' for key,value in library.items()))


def sign(key, at, size, turn=None):
    app=surface((1,1,1),0,.85,key)
    return pose(at,f'Shape {{ castShadows FALSE appearance {app} geometry Plane {{ size {vec(size)} }} }}',turn)


def rack(item, variant):
    length,width,high=item.length,item.width,item.height
    parts=[box((0,0,.20),(length,width,.40),'NAVY')]
    levels=(.49, high*.43, high*.76)
    for x in (-length*.47,0,length*.47):
        for side in (-1,1): parts.append(box((x,side*width*.46,high/2),(.09,.10,high),'TEAL'))
    rng=random.Random(variant+17)
    for z in levels:
        parts.append(box((0,0,z),(length,width,.10),'STEEL'))
        for side in (-1,1): parts.append(box((0,side*width*.49,z+.025),(length,.065,.16),'AMBER'))
        slots=4 if length>6 else 3
        for index in range(slots):
            if rng.random()<.14: continue
            dx=(index+.5-slots/2)*length/slots
            bw=length/slots*rng.uniform(.71,.90); bh=min(high-z-.10, rng.uniform(.54,.96))
            parts.append(box((dx,0,z+.11),(bw+.045,width*.87,.11),'WOOD'))
            parts.append(box((dx,rng.uniform(-.045,.045),z+.165+bh/2),(bw,width*.79,bh),'BOX'+str(rng.randrange(3))))
    for end in (-1,1):
        for direction in (-1,1):
            rise,width_span=1.16,width*.86
            for base in (.85,high-1.10):
                parts.append(box((end*length*.47,0,base),(.055,.045,math.hypot(width_span,rise)),'STEEL',(1,0,0,direction*math.atan2(width_span,rise))))
    return ' '.join(parts)


def fixture_model(item, variant):
    length,width,high=item.length,item.width,item.height
    def block(mat): return box((0,0,high/2),(length,width,high),mat)
    if item.kind in ('rack','island'): return rack(item,variant)
    if item.kind in ('wall','facade','barrier','partition'):
        thick1=(length+.012,width+.012,.19); thick2=(length+.018,width+.018,.08)
        return ' '.join([block('PANEL' if item.kind!='barrier' else 'NAVY'),box((0,0,.26),thick1,'INK'),
                         box((0,0,high*.79),thick2,'TEAL' if item.kind!='barrier' else 'GOLD'),
                         box((0,0,high-.08),(length+.025,width+.025,.12),'STEEL')])
    if item.kind in ('column','gate_post','dock_post'):
        return ' '.join([block('STEEL'),box((0,0,.52),(length+.015,width+.015,1.04),'GOLD')]+[
            box((0,0,z),(length+.022,width+.022,.13),'INK') for z in (.37,.73)])
    if item.kind in ('stock','rail'):
        cap=.042
        return ' '.join([box((0,0,(high-cap)/2),(length,width,high-cap),'AMBER' if item.kind=='stock' else 'TEAL'),
                         box((0,0,high-cap/2),(length+.004,width+.009,cap),'SILVER')])
    if item.kind=='stock_post': return block('TEAL')
    if item.kind=='stock_back':
        return ' '.join([block('NAVY'),box((-.064,0,.65),(.015,width,.10),'GOLD')])
    if item.kind=='conveyor':
        parts=[box((0,0,.23),(length,width-.18,.27),'NAVY')]
        for side in (-1,1): parts.append(box((0,side*(width/2-.04),.39),(length,.08,.13),'GOLD'))
        count=round(length/.23)
        for idx in range(count):
            x=(idx+.5-count/2)*length/count
            parts.append(cylinder((x,0,.408),.072,width-.13,'SILVER' if idx%3 else 'STEEL',(1,0,0,math.pi/2)))
        return ' '.join(parts)
    if item.kind=='bench':
        parts=[box((0,0,.91),(length,width,.13),'WOOD'),box((-.76,0,.48),(.82,.98,.86),'TEAL'),
               box((.54,.21,1.24),(.78,.11,.54),'INK'),box((.54,.148,1.25),(.68,.015,.42),'WINDOW'),
               box((.52,-.13,.994),(.52,.19,.045),'NAVY'),box((-.69,0,1.14),(.54,.45,.32),'BOX1')]
        for side in (-1,1): parts.append(box((1.21,side*.44,.45),(.09,.10,.9),'STEEL'))
        return ' '.join(parts)
    if item.kind=='chair':
        return (box((0,0,.46),(.53,.51,.10),'NAVY')+box((0,-.245,.70),(.52,.08,.42),'TEAL')+
                cylinder((0,0,.24),.047,.38,'STEEL')+box((0,0,.065),(.55,.12,.08),'INK')+
                box((0,0,.065),(.12,.56,.08),'INK'))
    if item.kind=='buffer':
        parts=[box((0,0,.08),(length,width,.16),'WOOD')]
        for sign in (-1,1):
            parts.append(box((sign*length*.244,0,.16+(high-.16)/2),(length*.47,width*.94,high-.16),'BOX'+str((variant+sign)%3)))
        return ' '.join(parts)
    return ' '.join([box((0,0,.11),(length,width,.22),'WOOD'),box((0,0,.71),(length*.96,width*.95,1.0),'BOX1'),
                     box((-.28,0,.71),(.065,width*.96,1.012),'NAVY')])


def person(spec,index):
    ident=spec['id']; parts=[]
    for side,suffix in ((-1,'R'),(1,'L')):
        leg=box((0,0,-.33),(.17,.16,.66),'JEANS')+box((.055,0,-.685),(.28,.16,.11),'RUBBER')
        parts.append(pose((0,side*.13,.77),leg,(0,1,0,0),ident+'_LEG_'+suffix))
        parts.append(box((.025,side*.245,1.06),(.145,.13,.51),'NAVY',(0,1,0,-.12)))
    parts += [box((0,0,1.09),(.31,.43,.64),'VEST' if index%3 else 'AMBER'), box((0,0,1.06),(.319,.442,.085),'BONE'),
              ball((0,0,1.58),(.155,.152,.19),'SKIN'+str(index%2)),ball((0,0,1.72),(.171,.168,.095),'GOLD'),
              cylinder((.018,0,1.69),.181,.032,'GOLD')]
    return solid(ident,(*spec['start'],0),' '.join(parts),pose((0,0,.87),'Capsule { radius .29 height 1.15 subdivision 12 }'))


def truck(spec):
    ident=spec['id']; parts=[]
    for x in (-.72,.51):
        for side in (-1,1):
            parts += [cylinder((x,side*.51,.30),.30,.17,'RUBBER',(1,0,0,math.pi/2)),
                      cylinder((x,side*.60,.30),.16,.026,'STEEL',(1,0,0,math.pi/2))]
    parts += [box((-.18,0,.56),(1.75,.99,.52),'GOLD'),box((-.76,0,.86),(.63,1.03,.45),'GOLD'),
              box((-.28,0,.89),(.62,.53,.16),'INK'),box((-.55,0,1.10),(.12,.55,.49),'INK'),
              box((-.20,0,1.35),(.31,.43,.55),'VEST'),ball((-.17,0,1.76),(.15,.15,.18),'SKIN0'),
              cylinder((-.17,0,1.88),.17,.08,'GOLD'),box((-.19,0,2.14),(1.45,1.16,.13),'INK')]
    for side in (-1,1):
        for x in (-.82,.42): parts.append(box((x,side*.5,1.48),(.065,.065,1.28),'INK'))
        parts.append(box((.67,side*.39,1.09),(.11,.11,2),'STEEL'))
    fork=box((.72,0,.59),(.12,1,.35),'INK')
    for side in (-1,1): fork+=box((1.39,side*.30,.44),(1.4,.13,.08),'STEEL')
    parts.append(pose((0,0,0),fork,ident=ident+'_FORKS'))
    collider='Group { children [ '+bounds((-.18,0,.60),(1.76,1.16,1.20))+bounds((-.2,0,1.69),(1.45,1.12,1.0))+bounds((.67,0,1.05),(.14,.90,2.1))
    for suffix,side in [('L',1),('R',-1)]:
        collider+=pose((1.39,side*.30,.44),'Box { size 1.4 .13 .08 }',ident=ident+'_TINE_'+suffix)
    collider+=' ] }'
    return solid(ident,(spec['dock_x'],spec['ends'][0],0),' '.join(parts),collider)


def stock_load(spec,offset):
    parts=[box((0,0,0),(1.08,1.26,.08),'WOOD'),box((0,0,.39),(1,1.16,.70),'BOX2')]
    for y in (-.45,0,.45): parts.append(box((0,y,-.09),(1.08,.10,.10),'WOOD'))
    for y in (-.33,.33): parts.append(box((0,y,.402),(1.014,.047,.694),'TEAL'))
    collider='Group { children [ '+bounds((0,0,.35),(1.08,1.26,.78))+''.join(bounds((0,y,-.09),(1.08,.10,.10)) for y in (-.45,0,.45))+' ] }'
    return solid(spec['id']+'_STOCK',(spec['dock_x']+offset,spec['ends'][0],.52),' '.join(parts),collider)
