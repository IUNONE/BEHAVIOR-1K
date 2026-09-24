"""Schematic X330 front flange and passive rear idler meshes, units millimeters.

Front OD16/thickness3 are from the ROBOTIS X330 drawing. Rear thickness3 is
an envelope approximation consistent with the 29 mm hinge opening, not a
manufacturer idler CAD model. Central holes are simplified for explanation.
"""
from pathlib import Path
import numpy as np


def ring(path: Path, inner: float, outer: float = 8., thickness: float = 3.) -> None:
    """Export a closed annular flange around local Z, centered at the origin."""
    n=96;vertices=[];faces=[]
    for z,r in [(-thickness/2,outer),(thickness/2,outer),(-thickness/2,inner),(thickness/2,inner)]:
        for a in np.arange(n)*2*np.pi/n:vertices.append((r*np.cos(a),r*np.sin(a),z))
    for i in range(n):
        j=(i+1)%n
        for q in [(i,j,n+j,n+i),(2*n+i,3*n+i,3*n+j,2*n+j),
                  (i,2*n+i,2*n+j,j),(n+i,n+j,3*n+j,3*n+i)]:
            faces += [(q[0],q[1],q[2]),(q[0],q[2],q[3])]
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w') as f:
        f.write('# X330 flange schematic, mm; simplified central hole.\n')
        for p in vertices:f.write('v '+' '.join(f'{v:.12g}' for v in p)+'\n')
        for face in faces:f.write('f '+' '.join(str(i+1) for i in face)+'\n')


if __name__=='__main__':
    root=Path(__file__).resolve().parents[2]/'hardware/joylo_v2_7dof_arm/urdf/support'
    ring(root/'j6_output_flange.obj',2.)
    ring(root/'j6_passive_idler.obj',3.2)
