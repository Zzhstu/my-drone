"""Closed version of the existing room. Geometry is exported for OFFLINE audit only."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw
from integration.r3live.scene import make_world as base_world


def make_world(path):
    path = Path(path)
    base_world(path)
    tree = ET.parse(path); world = tree.getroot().find('world')
    for include in list(world.findall('include')):
        if include.findtext('uri') == 'model://ground_plane': world.remove(include)
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '.75 .75 .75 1'
    ET.SubElement(scene, 'shadows').text = 'false'
    def slab(name, xyz, size):
        model=ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,[*xyz,0,0,0]))
        link=ET.SubElement(model,'link',name='body')
        for kind in ('collision','visual'):
            item=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,size))
    slab('floor',[0,0,-.05],[11,12,.1])
    slab('ceiling',[0,0,4.05],[11,12,.1])
    assets=path.parent/'room_materials';assets.mkdir()
    materials=[]
    boxes=[]
    for index,model in enumerate(world.findall('model')):
        name=model.get('name')
        size=np.array(list(map(float,model.findtext('link/collision/geometry/box/size').split())))
        center=np.array(list(map(float,model.findtext('pose').split())))[:3]
        boxes.append(dict(name=name,minimum=(center-size/2).tolist(),maximum=(center+size/2).tolist()))
        if name not in ('front','back','left','right','floor','ceiling'):continue
        rng=np.random.RandomState(1709+index)
        palette=np.array([[185,166,124],[85,124,163],[154,105,89],[120,153,125],[190,190,185],[82,94,101]])
        tiles=palette[rng.randint(0,len(palette),(16,16))].astype(np.uint8)
        canvas=Image.fromarray(tiles).resize((1024,1024),Image.NEAREST)
        draw=ImageDraw.Draw(canvas)
        for k in range(16):
            draw.line((k*64,0,k*64,1024),fill=(48,48,48),width=2)
            draw.line((0,k*64,1024,k*64),fill=(48,48,48),width=2)
        for y in range(16):
            for x in range(16):
                draw.text((x*64+13,y*64+23),'%02d:%02d'%(x,y),fill=(235,235,225))
        canvas.save(str(assets/(name+'.png')))
        materials.append('material RoomGS/%s\n{\n technique\n {\n  pass\n  {\n   lighting off\n   texture_unit\n   {\n    texture %s.png\n   }\n  }\n }\n}\n'%(name,name))
        visual=model.find('link/visual')
        old=visual.find('material')
        if old is not None:visual.remove(old)
        script=ET.SubElement(ET.SubElement(visual,'material'),'script')
        ET.SubElement(script,'uri').text='file://'+str(assets/'room.material')
        ET.SubElement(script,'uri').text='file://'+str(assets)
        ET.SubElement(script,'name').text='RoomGS/'+name
    (assets/'room.material').write_text('\n'.join(materials))
    tree.write(str(path),encoding='utf-8',xml_declaration=True)
    truth=dict(scope='OFFLINE EVALUATION ONLY; never a mapper/planner/trainer input',
               bounds=[[-5.4,-5.9,0],[5.4,5.9,4]],boxes=boxes)
    (path.parent/'audit_scene.json').write_text(json.dumps(truth,indent=2)+'\n')
    return truth
