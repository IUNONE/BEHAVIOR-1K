"""Lightweight original Switch Joy-Con models, based on Nintendo's envelope.

102 x 35.9 x 28.4 mm overall. Shell/bevels/buttons/rail are approximate,
not Nintendo manufacturing CAD. Right controls are laid out independently.
The right template is reflected with the arm by build_dual_joylo.py.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

DIRECTORY=Path(__file__).resolve().parents[2]/'hardware/joylo_v2_7dof_arm/urdf'


def make_shell() -> None:
    """Build a closed beveled asymmetric round rectangle along controller Y."""
    verts=[];faces=[]
    for y,inset in [(0.,1.3),(1.3,0.),(12.6,0.),(13.9,1.3)]:
        x0,x1,z0,z1=-35.9+inset,-2.5-inset,inset,102-inset
        # Two large outside corners and two small rail-side corners.
        for cx,cz,r,start in [(x1-3,z1-3,3,0),(x0+9,z1-9,9,90),
                              (x0+9,z0+9,9,180),(x1-3,z0+3,3,270)]:
            for angle in np.deg2rad(np.linspace(start,start+90,13)):
                verts.append((cx+r*np.cos(angle),-y,cz+r*np.sin(angle)))
    n=len(verts)//4
    for layer in range(3):
        for i in range(n):
            j=(i+1)%n;a,b=layer*n+i,layer*n+j;c,d=(layer+1)*n+j,(layer+1)*n+i
            faces.extend([(a,b,c),(a,c,d)])
    for layer,reverse in [(0,False),(3,True)]:
        center=len(verts);verts.append((-19.2,-float(layer==3)*13.9,51.))
        for i in range(n):
            tri=(center,layer*n+i,layer*n+(i+1)%n)
            faces.append(tri[::-1] if reverse else tri)
    import trimesh
    mesh=trimesh.Trimesh(vertices=np.array(verts),faces=np.array(faces),process=True)
    mesh.fix_normals()
    (DIRECTORY/'support').mkdir(exist_ok=True)
    mesh.export(DIRECTORY/'support/joycon_shell.obj',include_color=False)


def attach_joycon(root: ET.Element, side: str) -> None:
    """Attach a side-specific controller to the original LJC cradle coordinates."""
    for tag,name in [('link','joycon'),('joint','joycon_mount')]:
        old=root.find(f"{tag}[@name='{name}']")
        if old is not None:root.remove(old)
    link=ET.SubElement(root,'link',name='joycon')
    color='0.03 0.68 0.87 1' if side=='left' else '0.96 0.20 0.24 1'
    counter=0
    def visual(name,xyz=(0,0,0),rpy=(0,0,0),rgba='0.055 0.065 0.075 1',box=None,cylinder=None,mesh=None):
        nonlocal counter
        counter+=1
        v=ET.SubElement(link,'visual',name='joycon_'+name)
        # Canonical front is -Y, so +X is screen-right when viewing the controls.
        xyz=np.asarray(xyz)*[1,-1,1]
        rpy=np.asarray(rpy)*[-1,1,-1]
        ET.SubElement(v,'origin',xyz=' '.join(f'{x/1000:.9g}' for x in xyz),rpy=' '.join(map(str,rpy)))
        g=ET.SubElement(v,'geometry')
        if box:ET.SubElement(g,'box',size=' '.join(f'{x/1000:.9g}' for x in box))
        if cylinder:ET.SubElement(g,'cylinder',radius=str(cylinder[0]/1000),length=str(cylinder[1]/1000))
        if mesh:ET.SubElement(g,'mesh',filename=mesh,scale='0.001 0.001 0.001')
        mat=ET.SubElement(v,'material',name=f'joycon_{side}_{counter}');ET.SubElement(mat,'color',rgba=rgba)
    front=(-np.pi/2,0,0)  # Cylinder Z -> button-face +Y.
    visual('shell',rgba=color,mesh='support/joycon_shell.obj')
    visual('rail',(-1.25,7,51),box=(2.5,3,92))
    visual('trigger',(-24,-2.5,96),box=(19,5,12))
    visual('shoulder',(-19,10.5,99),box=(25,3,3))
    stick_z=77 if side=='left' else 47
    pad_z=48 if side=='left' else 77
    visual('stick_stem',(-20,16.8,stick_z),rpy=front,cylinder=(4.2,5.8))
    visual('stick_cap',(-20,21.55,stick_z),rpy=front,cylinder=(7.5,3.7))
    visual('stick_center',(-20,23.35,stick_z),rpy=front,cylinder=(5.6,.1),rgba='0.10 0.11 0.12 1')
    for label,dx,dz in [('up',0,6.5),('right',6.5,0),('down',0,-6.5),('left',-6.5,0)]:
        visual('button_'+label,(-20+dx,15,pad_z+dz),rpy=front,cylinder=(2.4,2.2))
    if side=='left':
        visual('minus',(-6.5,15.1,94),box=(4.5,2.4,1.2))
        visual('capture',(-7,15,20),box=(5,2.2,5))
    else:
        visual('plus_horizontal',(-6.5,15.1,94),box=(4.5,2.4,1.2))
        visual('plus_vertical',(-6.5,15.1,94),box=(1.2,2.4,4.5))
        visual('home',(-7,15,20),rpy=front,cylinder=(3.1,2.2))
        visual('home_center',(-7,16.15,20),rpy=front,cylinder=(1.65,.1),rgba='0.75 0.77 0.79 1')
    # A cradle-floor/rail placement, independently editable without changing J7.
    # LJC native: inner backing X=12.28; rail groove Y=5.1..7.6; floor Z=2.
    # Controller width lies along cradle Y, thickness along X, length along Z.
    ljc=root.find("link[@name='ljc']/visual[@name='ljc_printed']/origin")
    origin=np.fromstring(ljc.get('xyz'),sep=' ')+np.array([.01228,.0051,.002])
    joint=ET.SubElement(root,'joint',name='joycon_mount',type='fixed')
    ET.SubElement(joint,'parent',link='ljc');ET.SubElement(joint,'child',link='joycon')
    ET.SubElement(joint,'origin',xyz=' '.join(f'{x:.12g}' for x in origin),rpy=f'0 0 {-np.pi/2}')


if __name__=='__main__':make_shell()
