"""Original tiled surfaces, industrial graphics and the printable hall key."""
import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ASSETS = Path(__file__).resolve().parents[1] / 'textures/final_hall'
INK, WHITE, TEAL, GOLD = '#122d38', '#e8ede6', '#16b4a4', '#f3b544'


def font(size):
    return ImageFont.load_default(size=size)


def write(name, image):
    ASSETS.mkdir(parents=True, exist_ok=True)
    image.save(ASSETS / (name + '.png'), optimize=True)


def surfaces():
    rng = random.Random(18)
    size = 256
    field = [[rng.randrange(-7,8) for _ in range(8)] for _ in range(8)]
    image = Image.new('RGB',(size,size))
    pixels = []
    for y in range(size):
        for x in range(size):
            ix, iy = x // 32, y // 32
            fx, fy = x % 32 / 32, y % 32 / 32
            tone = sum(field[(iy+dy)%8][(ix+dx)%8] * wx * wy for dx,wx in enumerate((1-fx,fx)) for dy,wy in enumerate((1-fy,fy)))
            base = 155 + round(tone + rng.gauss(0, 3)) - (18 if x < 2 or y < 2 else 0)
            pixels.append((base,base+3,base+2))
    image.putdata(pixels)
    write('concrete',image)
    for index, (base, accent) in enumerate([((175,129,76),'#344a4c'),((198,157,103),'#365469'),((148,107,70),'#634735')]):
        image = Image.new('RGB',(size,size))
        image.putdata([tuple(channel + rng.randint(-4,4) for channel in base) for _ in range(size*size)])
        draw = ImageDraw.Draw(image)
        draw.rectangle((112,0,135,size),fill='#b18e61'); draw.line((125,0,125,size),fill='#876c48',width=3)
        draw.rectangle((16,155,100,209),fill='#e7ddbe'); draw.text((23,160),'G18 / LOT',font=font(13),fill=accent)
        for x in range(23,94,4):
            draw.rectangle((x,185,x+rng.choice((1,2,3)),202),fill=INK)
        for x in (191,211):
            draw.line((x,72,x,40),fill=accent,width=4); draw.polygon([(x,33),(x-7,46),(x+7,46)],fill=accent)
        write('carton'+str(index),image)
    metal = Image.new('RGB',(128,128),'#9ca9a9'); draw = ImageDraw.Draw(metal)
    for x in range(0,128,16):
        for offset,shade in enumerate(('#778687','#889799','#adb9b8','#afbbba')):
            draw.line((x+offset,0,x+offset,128),fill=shade)
    write('cladding',metal)
    puddle = Image.new('RGBA',(256,256),(0,0,0,0)); draw = ImageDraw.Draw(puddle)
    for offset,rad,colour in [(0,106,(35,58,62,195)),(0,98,(69,92,90,210)),(-7,65,(89,115,108,205))]:
        points=[]
        for index in range(100):
            angle=math.tau*index/100
            radius=rad*(.86+.09*math.sin(3*angle+.5)+.08*math.cos(5*angle)+.055*math.sin(7*angle))
            points.append((128+offset+radius*math.cos(angle),132+radius*math.sin(angle)))
        draw.polygon(points,fill=colour)
    draw.arc((65,72,177,156),190,265,fill=(191,202,175,165),width=3)
    for x,y,radius in [(39,58,6),(211,176,5),(204,187,3),(47,166,3)]:
        draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=(56,80,80,193))
    write('spill',puddle)


def plaque(key, title, subtitle, color=TEAL):
    image = Image.new('RGB',(512,128),INK); draw = ImageDraw.Draw(image)
    draw.rectangle((0,0,13,128),fill=color); draw.rectangle((20,9,505,119),outline='#658489',width=2)
    draw.text((36,14),title,font=font(min(57, int(720/max(1,len(title))))),fill=WHITE,stroke_width=1)
    draw.text((38,85),subtitle,font=font(20),fill=color)
    write(key,image)


def plan_view(path, cfg, obstacles, stations):
    image=Image.new('RGB',(1520,1100),'#102630'); draw=ImageDraw.Draw(image)
    draw.text((65,29),'18   /   DISTRIBUTION HALL',font=font(36),fill=WHITE)
    draw.text((65,82),'FACILITY PLAN     /     96 x 64 m     /     NORTH UP',font=font(19),fill='#9ac2c0')
    scale, ox, oy=13,760,565
    def pt(x,y): return (round(ox+x*scale),round(oy-y*scale))
    def rect(x1,y1,x2,y2,**kw): draw.rectangle([pt(min(x1,x2),max(y1,y2)),pt(max(x1,x2),min(y1,y2))],**kw)
    rect(-48,-32,48,32,fill='#d4dcd4')
    for x in range(-45,49,5): draw.line([pt(x,-32),pt(x,32)],fill='#bdc9c2')
    for y in range(-30,33,5): draw.line([pt(-48,y),pt(48,y)],fill='#bdc9c2')
    for side,a,b,depth in cfg['edge_bays']:
        cut={'west':(-48,a,-48+depth,b),'east':(48-depth,a,48,b),
             'north':(a,32-depth,b,32),'south':(a,-32,b,-32+depth)}[side]
        rect(*cut,fill='#102630')
    for truck in cfg['trucks']:
        for aisle in truck['aisles']:
            a,b=truck['ends']; edge=truck['dock_x']; xy=[(edge,a),(aisle,a),(aisle,b),(edge,b)]
            draw.line([pt(*pos) for pos in xy],fill='#bf8d2f',width=3)
    colors={'wall':'#243e47','barrier':'#243e47','partition':'#243e47','rack':'#587a88','island':'#638591','column':'#ce8b42','rail':'#577b78','facade':'#397b7d','stock':'#94713a','bench':'#967c65','crate':'#a68e76','conveyor':'#519d96'}
    for item in obstacles:
        co,si=math.cos(item.angle),math.sin(item.angle)
        points=[pt(item.x+a*co-b*si,item.y+a*si+b*co) for a,b in [(-item.length/2,-item.width/2),(item.length/2,-item.width/2),(item.length/2,item.width/2),(-item.length/2,item.width/2)]]
        draw.polygon(points,fill=colors.get(item.kind,'#52656b'))
    for site in cfg['spills']:
        x,y=pt(*site['center']); draw.ellipse((x-6,y-6,x+6,y+6),outline='#956c27',width=2)
    for station in stations:
        x,y=pt(*station['at']); pick=station['id'][0]=='P'; shade='#087f78' if pick else '#286091'
        draw.rounded_rectangle((x-19,y-12,x+19,y+11),4,fill=shade); draw.text((x,y),station['id'],anchor='mm',font=font(14),fill='white')
    for gate in cfg['crossings']+[cfg['stock_gate']]:
        x,y=gate['x'],gate.get('y',0); half=gate['w']/2; mode=gate['mode']
        rect(x-half,y-.15,x+half,y+.15,fill={'open':'#19816f','presence':'#337ec3','timed':'#b7633d'}[mode]); px,py=pt(x,y)
        draw.text((px,py-7),gate['id'],anchor='ms',font=font(12),fill=INK)
    x,y=pt(*cfg['parking']); draw.rounded_rectangle((x-27,y-10,x+27,y+10),4,fill='#075a58'); draw.text((x,y),'PARK',anchor='mm',font=font(13),fill='white')
    draw.text((135,1016),'TEAL  Pickup / parking      BLUE  Delivery      GOLD  Shelf routes (floor trips vary)      RINGS  Possible spills',font=font(20),fill=WHITE)
    draw.text((135,1055),'ROAD 6 m    |    LIGHT 1.65 m    |    CART 1.95 m    |    BULK 2.65 m    |    G01/G04/G05 presence    |    G02/G03 timed',font=font(16),fill='#a6c7c1')
    path.parent.mkdir(parents=True,exist_ok=True); image.save(path,optimize=True)
