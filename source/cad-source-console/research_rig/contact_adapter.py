"""Add verified collision cells to an MJCF tree without changing body inertia."""
from __future__ import annotations
import xml.etree.ElementTree as ET
import numpy as np

def numbers(values):return ' '.join(format(float(v),'.17g')for v in values)
def add_partition(asset,body,part,prefix=None,body_reference_world_m=None,**geom_attributes):
 """Input body reference is the caller's initial world translation, no rotation.

Part cell data remains in the manifest's reference frame. Recenter each mesh
before MuJoCo float32 storage, and put that translation into geom.pos. The body
must have a separately supplied explicit inertial. Shell asset inertia only
prevents tiny exact wedges from failing MuJoCo's volume floor; mass stays zero.
 """
 prefix=prefix or part['source_name'];origin=np.array(part['body_reference_world_m']);body_origin=np.array(body_reference_world_m if body_reference_world_m is not None else origin)
 names=[]
 for cell in part['pieces']:
  name=prefix+'_'+cell['id'];v=np.array(cell['vertices_local_m']).reshape(-1,3);center=(v.min(0)+v.max(0))/2
  ET.SubElement(asset,'mesh',name=name,inertia='shell',vertex=numbers((v-center).ravel()),face=' '.join(map(str,cell['triangles'])))
  attrs={'name':name,'type':'mesh','mesh':name,'pos':numbers(origin-body_origin+center),'mass':'0','density':'0','margin':'0','gap':'0'};attrs.update({k:str(v)for k,v in geom_attributes.items()});ET.SubElement(body,'geom',**attrs);names.append(name)
 return names
