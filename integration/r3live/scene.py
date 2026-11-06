"""Small static color/geometry-rich room; no geometry is supplied to SLAM."""
import xml.etree.ElementTree as ET


def make_world(path):
    sdf = ET.Element('sdf', version='1.6'); world = ET.SubElement(sdf, 'world', name='default')
    for name in ('ground_plane', 'sun'):
        ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://'+name
    physics = ET.SubElement(world, 'physics', name='default_physics', type='ode')
    for k, v in [('max_step_size', '.001'), ('real_time_update_rate', '1000'), ('real_time_factor', '1')]:
        ET.SubElement(physics, k).text = v
    ET.SubElement(world, 'gravity').text = '0 0 -9.80665'
    def box(name, xyz, size, color):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, [*xyz, 0, 0, 0]))
        link = ET.SubElement(model, 'link', name='body')
        for tag in ('collision', 'visual'):
            item = ET.SubElement(link, tag, name=tag)
            ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = ' '.join(map(str, size))
            if tag == 'visual':
                mat = ET.SubElement(item, 'material')
                for channel in ('ambient', 'diffuse'): ET.SubElement(mat, channel).text = color
    for name, xyz, size in [('front', [5.5, 0, 2], [.2, 12, 4]),
                            ('back', [-5.5, 0, 2], [.2, 12, 4]),
                            ('left', [0, 6, 2], [11, .2, 4]),
                            ('right', [0, -6, 2], [11, .2, 4])]:
        box(name, xyz, size, '.72 .75 .78 1')
    palette = ['.85 .12 .08 1', '.06 .62 .18 1', '.08 .25 .90 1', '.95 .7 .05 1', '.1 .1 .1 1', '.92 .92 .92 1']
    for i in range(12):
        for j in range(7):
            box('tile_%d_%d' % (i,j), [5.35, -4.4+i*.75, .4+j*.45], [.08,.48,.27], palette[(i*3+j*5+i*j)%6])
    for i, (xyz, size) in enumerate([([2.5,-3, .7],[.7,1,1.4]), ([3,2.8,1],[1,.8,2]),
                                    ([.3,4,.45],[1.4,1,.9]), ([-3,3.8,.9],[.6,.6,1.8])]):
        box('landmark_%d'%i,xyz,size,palette[i])
    gui = ET.SubElement(world, 'gui'); cam = ET.SubElement(gui, 'camera', name='user_camera')
    ET.SubElement(cam, 'pose').text = '11 -12 13 0 .7 2.2'
    ET.ElementTree(sdf).write(str(path), encoding='utf-8', xml_declaration=True)
