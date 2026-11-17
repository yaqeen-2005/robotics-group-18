"""Regenerate the primitive warehouse. This writes files; it does not run Webots."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
HEADER = '#VRML_SIM R2025a utf8\n'

def box(name, xyz, size, color='0.55 0.60 0.65', collision=True):
    geom = f'Box {{ size {size} }}'
    return f'''Solid {{
  translation {xyz}
  name "{name}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.9 metalness 0 }} geometry {geom} }} ]
  {'boundingObject ' + geom if collision else ''}
}}
'''

world = HEADER + '''EXTERNPROTO "../protos/PalletRobot.proto"
EXTERNPROTO "../protos/Pallet.proto"
WorldInfo {
  basicTimeStep 16
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties { material1 "support" material2 "floor" coulombFriction [ 0.01 ] }
    ContactProperties { material1 "tyre" material2 "floor" coulombFriction [ 1.0 ] }
  ]
}
Viewpoint {
  orientation 0 1 0 1.57079632679
  position 0 0 20
  fieldOfView 1.0
  description "Warehouse overhead"
}
Background { skyColor [ 0.72 0.80 0.88 ] }
DirectionalLight { direction -0.4 0.3 -1 intensity 1 castShadows FALSE }
'''
world += box('floor', '0 0 -0.1', '12.3 10.3 0.2', '0.72 0.72 0.72').replace('name "floor"', 'name "floor"\n  contactMaterial "floor"')
for name, pos, size in [
    ('west','-6.075 0 0.75','0.15 10.3 1.5'),
    ('east','6.075 0 0.75','0.15 10.3 1.5'),
    ('south','0 -5.075 0.75','12 0.15 1.5'),
    ('north','0 5.075 0.75','12 0.15 1.5'),
    ('divider_west','-4.9 0 0.75','2.2 0.2 1.5'),
    ('divider_middle','-0.2 0 0.75','4 0.2 1.5'),
    ('divider_east','5.1 0 0.75','1.8 0.2 1.5'),
    ('shelf_west','-5 3 0.75','1 2 1.5'),
    ('shelf_east','5 3 0.75','1 2 1.5'),
]:
    world += box(name, pos, size)
# Rails support the outer edge of a pallet, leaving a 1.04m entry opening.
# Enter along +x/-x. Robot total width is 0.92m. Rail top = 0.48m.
for name, x, y, color in [('normal_pickup',-3,-3,'0.2 0.65 0.25'), ('wide_pickup',3,-3,'0.8 0.55 0.1'), ('delivery',-3,3,'0.2 0.35 0.8')]:
    world += box(name+'_marker', f'{x} {y} 0.002', '1.8 1.8 0.004', color, False)
    for side in [-1,1]:
        world += box(name+('_left' if side==1 else '_right'), f'{x} {y+side*0.58} 0.24', '1.45 0.12 0.48', color)
world += '''DEF NORMAL_PALLET Pallet { translation -3 -3 0.52 name "normal_pallet" }
DEF WIDE_PALLET Pallet { translation 3 -3 0.52 name "wide_pallet" size 1.2 1.4 0.08 mass 15 color 0.8 0.5 0.15 }
DEF TRANSPORT_ROBOT PalletRobot { translation -4.8 -3 0.16 }
'''
(ROOT/'worlds'/'warehouse.wbt').write_text(world)

if __name__ == '__main__':
    print('Wrote webots/worlds/warehouse.wbt (no simulation launched).')
