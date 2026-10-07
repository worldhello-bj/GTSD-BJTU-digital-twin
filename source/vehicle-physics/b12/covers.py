"""Four source-shaped removable cover proxies on an explicit fixed maintenance bench.

Full free-body mechanics after ideal fixture release. No hinge is inferred.
Source mounting poses are not asserted: two hidden bone-bound lids evaluate
at the origin, and their actual assembly location requires external evidence.
"""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring, indent
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent


@dataclass(frozen=True)
class CoverParameters:
    proxy_mass_kg: float=.25
    fixture_height_m: float=.4
    fixture_time_constant_s: float=.0005
    contact_time_constant_s: float=.001
    friction: float=.4
    timestep_s: float=.00005

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('positive finite cover proxy inputs required')


def vec(value):
    return ' '.join(format(float(v),'.12g') for v in value)


def add(parent,tag,**attributes):
    return SubElement(parent,tag,{k:str(v) for k,v in attributes.items()})


def model(parameters=None, *, with_floor=True):
    p=parameters or CoverParameters()
    source=json.loads((ROOT/'source_covers.json').read_text(encoding='utf-8'))
    root=Element('mujoco',model='B12 explicitly inferred fixed maintenance bench')
    add(root,'compiler',angle='radian',inertiafromgeom='false')
    option=add(root,'option',timestep=p.timestep_s,gravity='0 0 -9.81',integrator='implicitfast',solver='Newton',
               iterations=100,tolerance='1e-12',cone='elliptic')
    add(option,'flag',energy='enable')
    world=add(root,'worldbody'); equalities=add(root,'equality')
    if with_floor:
        add(world,'geom',name='bench_floor',type='plane',size='3 2 .1',pos='0 0 0',
            friction=f'{p.friction} .001 .0001',solref=f'{p.contact_time_constant_s} 1',solimp='.99 .99 .0001')
    records=[]
    for i,r in enumerate(source['covers']):
        key='cover_'+str(i); dims=np.array(r['aabb_dimensions_m'])
        pos=[.6*(i-1.5),0,p.fixture_height_m]
        body=add(world,'body',name=key,pos=vec(pos))
        add(body,'freejoint',name=key+'_free')
        inertia=p.proxy_mass_kg*np.array([dims[1]**2+dims[2]**2,dims[0]**2+dims[2]**2,dims[0]**2+dims[1]**2])/12
        add(body,'inertial',pos='0 0 0',mass=p.proxy_mass_kg,diaginertia=vec(inertia))
        add(body,'geom',name=key+'_proxy',type='box',size=vec(dims/2),
            friction=f'{p.friction} .001 .0001',solref=f'{p.contact_time_constant_s} 1',solimp='.99 .99 .0001',rgba='.1 .48 .52 1')
        add(equalities,'weld',name=key+'_fixture',body1=key,solref=f'{p.fixture_time_constant_s} 1',solimp='.995 .995 .0001')
        records.append(dict(key=key,source_object=r['name'],initial_bench_center_world_m=pos,
                            collision_box_dimensions_m=dims.tolist(),mass_kg=p.proxy_mass_kg,diagonal_inertia_kg_m2=inertia.tolist(),
                            source_geometry_center_world_m=r['aabb_center_world_m'],source_mesh_center_local_m=r['center_local_m']))
    indent(root)
    spec=dict(source_sha256=source['source_sha256'],parameters=asdict(p),covers=records,total_mass_kg=4*p.proxy_mass_kg,
              scope='separate fixed maintenance bench; not added to moving vehicle mass or claimed assembled mounting poses',
              assumptions=['each cover assigned .25 kg unmeasured proxy mass','uniform bounding-box inertia/contact, source fin geometry is visual',
                           'ideal releasable six-axis fixtures, no source hinge/bolt/hand geometry',
                           'two source bone-bound lids evaluate at origin; real mounting location unresolved'])
    return tostring(root,encoding='unicode'),spec


class Bench:
    def __init__(self,parameters=None,*,with_floor=True):
        self.xml,self.spec=model(parameters,with_floor=with_floor)
        self.model=mujoco.MjModel.from_xml_string(self.xml); self.data=mujoco.MjData(self.model)
        mujoco.mj_forward(self.model,self.data)
        self.release_events=[]
        self.momentum_impulse=np.zeros(3)

    def release(self,key):
        if key not in {r['key'] for r in self.spec['covers']}:
            raise ValueError('unknown cover fixture')
        eq=int(self.model.equality(key+'_fixture').id)
        self.data.eq_active[eq]=False
        self.release_events.append(dict(key=key,time_s=self.data.time))

    def apply_point_force(self,key,force_N,offset_from_com_world_m=(0,0,0)):
        if key not in {r['key'] for r in self.spec['covers']}:
            raise ValueError('known cover required')
        force,offset=np.asarray(force_N,dtype=float),np.asarray(offset_from_com_world_m,dtype=float)
        if force.shape!=(3,) or offset.shape!=(3,) or not np.isfinite(force).all() or not np.isfinite(offset).all():
            raise ValueError('finite three-component force and lever arm required')
        self.data.xfrc_applied[self.model.body(key).id]=np.concatenate((force,np.cross(offset,force)))

    def step(self):
        old=self.data.qvel.copy()
        mujoco.mj_step(self.model,self.data)
        mid=(old+self.data.qvel)/2
        work=dict(constraint=float(self.data.qfrc_constraint@mid)*self.model.opt.timestep,external=0.)
        for record in self.spec['covers']:
            bid=int(self.model.body(record['key']).id)
            index=int(self.model.joint(record['key']+'_free').dofadr[0])
            net_force=(self.data.qfrc_constraint[index:index+3]+self.data.xfrc_applied[bid,:3]
                       +record['mass_kg']*self.model.opt.gravity)
            self.momentum_impulse+=net_force*self.model.opt.timestep
            jp=np.zeros((3,self.model.nv)); jr=np.zeros_like(jp)
            mujoco.mj_jacBodyCom(self.model,self.data,jp,jr,bid)
            force=self.data.xfrc_applied[bid]
            work['external']+=float(force[:3]@(jp@mid)+force[3:]@(jr@mid))*self.model.opt.timestep
        mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or any(self.data.warning.number):
            raise RuntimeError('B12 non-finite state or MuJoCo warning')
        return work
