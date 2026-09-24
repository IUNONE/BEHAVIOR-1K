"""Create a dimensioned 25 x 50 x L six-slot visualization profile.

Slot geometry is schematic, not manufacturer CAD. Core bores and threads are
omitted. X is 25 mm, Y is length, Z is 50 mm. No Boolean/CAD dependency needed.
"""
from pathlib import Path
import numpy as np


def build_profile(path: Path, length_mm: float = 200.) -> None:
    """Extrude a rectangular-cell cross section into a watertight OBJ."""
    # Rectangular cuts model six open T-slots. Nominal mouths 6 mm; cavities 10 mm.
    cuts = []
    for sign in [-1, 1]:
        for z in [-12.5, 12.5]:
            cuts += [(sign*10, sign*12.5, z-3, z+3),
                     (sign*6, sign*10, z-5, z+5)]
        cuts += [(-3, 3, sign*22.5, sign*25), (-5, 5, sign*18.5, sign*22.5)]
    cuts = [(min(a,b),max(a,b),min(c,d),max(c,d)) for a,b,c,d in cuts]
    xs = sorted({-12.5,12.5,*[v for c in cuts for v in c[:2]]})
    zs = sorted({-25.,25.,*[v for c in cuts for v in c[2:]]})
    filled = set()
    for i in range(len(xs)-1):
        for j in range(len(zs)-1):
            x,z=(xs[i]+xs[i+1])/2,(zs[j]+zs[j+1])/2
            if not any(a<x<b and c<z<d for a,b,c,d in cuts):filled.add((i,j))
    vertices=[];faces=[];indices={}
    def quad(points: list[tuple[float,float,float]], normal: tuple[int,int,int]) -> None:
        p=np.array(points)
        if np.dot(np.cross(p[1]-p[0],p[2]-p[0]),normal)<0:points=points[::-1]
        ids=[]
        for point in points:
            if point not in indices:
                indices[point]=len(vertices)+1;vertices.append(point)
            ids.append(indices[point])
        faces.extend([(ids[0],ids[1],ids[2]),(ids[0],ids[2],ids[3])])
    y0,y1=-length_mm/2,length_mm/2
    for i,j in sorted(filled):
        x0,x1,z0,z1=xs[i],xs[i+1],zs[j],zs[j+1]
        for y,n in [(y0,(0,-1,0)),(y1,(0,1,0))]:
            quad([(x0,y,z0),(x1,y,z0),(x1,y,z1),(x0,y,z1)],n)
        if (i-1,j) not in filled:quad([(x0,y0,z0),(x0,y1,z0),(x0,y1,z1),(x0,y0,z1)],(-1,0,0))
        if (i+1,j) not in filled:quad([(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)],(1,0,0))
        if (i,j-1) not in filled:quad([(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0)],(0,0,-1))
        if (i,j+1) not in filled:quad([(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)],(0,0,1))
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w') as f:
        f.write('# Schematic 2550 six-slot extrusion. Units mm. Not manufacturer CAD.\n')
        for v in vertices:f.write('v '+' '.join(f'{x:.9g}' for x in v)+'\n')
        for face in faces:f.write('f '+' '.join(map(str,face))+'\n')
