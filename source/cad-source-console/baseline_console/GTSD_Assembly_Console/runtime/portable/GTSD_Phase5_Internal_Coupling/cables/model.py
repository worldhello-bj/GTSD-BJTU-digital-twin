"""B11 algorithm, NEW geometry only. Finite-mass articulated capsule proxies.

Centerlines are extracted from the new Blender POLY curves. Device identities,
material law, rest curvature, attachments and ground are explicit assumptions.
No imported historical vehicle model, coordinates, or pose-driven animation.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import hashlib,json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
DEFAULT_ROUTES=('bay_wire_0','panto_air_hose')
ROUTES={**{'bay_wire_'+str(i):'BAY_routed_wire'+('' if i==0 else '.%03d'%i) for i in range(4)},'bay_air_pipe':'BAY_pneumatic_pipe','panto_air_hose':'Panto_bellows_visible_air_hose','motor_temperature_lead':'DETAIL_DRIVE_motor_temperature_lead','encoder_lead':'DETAIL_DRIVE_encoder_lead','reader_lead':'BODY_reader_local_lead'}
@dataclass(frozen=True)
class CableParameters:
    density_kg_m3:float=1200.
    reference_EI_Nm2:float=.015
    reference_radius_m:float=.012
    angular_damping_Nm_s:float=.0001
    damping_reference_length_m:float=.05
    maximum_link_length_m:float=.20
    endpoint_time_constant_s:float=.004
    contact_time_constant_s:float=.004
    friction:float=.3
    floor_z_m:float=-.05
    def __post_init__(self):
        for k,v in asdict(self).items():
            if type(v) not in (int,float) or not math.isfinite(v) or (k!='floor_z_m' and v<=0):raise ValueError('finite positive cable parameter required: '+k)
        if self.maximum_link_length_m<.02:raise ValueError('link resolution below safety/cost floor .02 m')
def vec(v):return ' '.join(format(float(x),'.15g') for x in v)
def add(p,t,**kw):return ET.SubElement(p,t,{k:str(v) for k,v in kw.items()})
def source_routes():return json.loads((ROOT/'source_routes.json').read_text())
def subdivide(points,maximum):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or not np.isfinite(p).all():raise ValueError('finite 3D centerline required')
    out=[p[0]]
    for a,b in zip(p,p[1:]):
        L=float(np.linalg.norm(b-a))
        if L<1e-8:raise ValueError('zero-length source segment')
        n=max(1,math.ceil(L/maximum));out.extend(a+(b-a)*j/n for j in range(1,n+1))
    return np.asarray(out)
def augment_xml(xml,parameters=None,bindings=None,route_keys=DEFAULT_ROUTES,with_floor=True):
    p=parameters or CableParameters()
    if isinstance(p,dict):p=CableParameters(**p)
    if not route_keys or len(set(route_keys))!=len(route_keys) or any(k not in ROUTES for k in route_keys):raise ValueError('unique known route keys required')
    source=source_routes(); sources={r['source_object']:r for r in source['records']}
    if any(ROUTES[k] not in sources for k in route_keys):raise ValueError('route has no verified new-source POLY centerline')
    root=ET.fromstring(xml);world=root.find('worldbody')
    if world is None:raise ValueError('worldbody missing')
    for el in root.iter():
        if el.get('name','').startswith('v5c_'):raise ValueError('cables already augmented')
    eq=root.find('equality')
    if eq is None:eq=add(root,'equality')
    contact=root.find('contact')
    if contact is None:contact=add(root,'contact')
    ref=mujoco.MjModel.from_xml_string(xml);state=mujoco.MjData(ref);mujoco.mj_forward(ref,state)
    records=[];bindings=bindings or {}
    for key in route_keys:
        r=sources[ROUTES[key]];points=subdivide(r['points_world_m'],p.maximum_link_length_m);lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
        if len(lengths)>32:raise ValueError('maximum 32 links per route')
        targets=bindings.get(key,('panto_base','panto_base') if key=='panto_air_hose' else ('frame','frame') if key in ('encoder_lead','motor_temperature_lead') else ('payload_secondary','payload_secondary'))
        if len(targets)!=2:raise ValueError('two endpoint body bindings required')
        anchors=[]
        for end,(point,target) in enumerate(zip((points[0],points[-1]),targets)):
            if type(target) is not str:raise ValueError('body name must be text')
            bid=mujoco.mj_name2id(ref,mujoco.mjtObj.mjOBJ_BODY,target)
            if bid<0:raise ValueError('missing endpoint target body '+target)
            body=world if target=='world' else root.find('.//body[@name="'+target+'"]')
            local=state.xmat[bid].reshape(3,3).T@(point-state.xpos[bid])
            name=f'v5c_{key}_anchor_{end}'
            add(body,'site',name=name,pos=vec(local),size='.001',rgba='1 .5 0 1')
            anchors.append(dict(body=target,site=name,local_m=local.tolist(),source_world_m=point.tolist()))
        parent=world;names=[];segments=[];radius=r['radius_m']
        for i,(a,b,L) in enumerate(zip(points,points[1:],lengths)):
            name=f'v5c_{key}_{i}';delta=b-a;mass=p.density_kg_m3*math.pi*radius**2*L
            body=add(parent,'body',name=name,pos=vec(a if i==0 else a-points[i-1]))
            if i==0:add(body,'freejoint',name=name+'_free')
            else:add(body,'joint',name=name+'_bend',type='ball',pos='0 0 0',limited='false',stiffness=p.reference_EI_Nm2*(radius/p.reference_radius_m)**4/L,damping=p.angular_damping_Nm_s*p.damping_reference_length_m/L,armature='0',frictionloss='0')
            axis=delta/L;ia=mass*radius**2/2;it=mass*(L*L/12+radius**2/4);I=np.eye(3)*it+(ia-it)*np.outer(axis,axis)
            add(body,'inertial',pos=vec(delta/2),mass=mass,fullinertia=vec([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
            add(body,'geom',name=name+'_capsule',type='capsule',fromto='0 0 0 '+vec(delta),size=radius,mass='0',contype='64',conaffinity='64',condim='3',friction=f'{p.friction} .001 .0001',solref=f'{p.contact_time_constant_s} 1',rgba='.9 .5 .04 1')
            add(body,'site',name=name+'_a',pos='0 0 0',size='.001');add(body,'site',name=name+'_b',pos=vec(delta),size='.001')
            segments.append(dict(body_name=name,geom_name=name+'_capsule',a_local_m=[0,0,0],b_local_m=delta.tolist(),radius_m=radius,mass_kg=mass,length_m=L))
            names.append(name);parent=body
        for end,(link,site) in enumerate(((names[0],names[0]+'_a'),(names[-1],names[-1]+'_b'))):
            add(eq,'connect',name=f'v5c_{key}_attachment_{end}',site1=site,site2=anchors[end]['site'],solref=f'{p.endpoint_time_constant_s} 1',solimp='.995 .995 .001')
        for i,name in enumerate(names):
            for other in names[i+1:i+3]:add(contact,'exclude',body1=name,body2=other)
        records.append(dict(key=key,source_object=r['source_object'],source_refs=r['source_refs'],termination_status=r['termination_status'],anchors=anchors,segments=segments,reference_centerline_world_m=points.tolist(),source_length_m=r['source_length_m'],link_length_sum_m=float(lengths.sum()),link_count=len(lengths),mass_kg=sum(s['mass_kg'] for s in segments)))
    if with_floor:add(world,'geom',name='v5c_ground_proxy',type='plane',pos=f'0 0 {p.floor_z_m}',size='3 3 .01',contype='64',conaffinity='64',condim='3',friction=f'{p.friction} .001 .0001',solref=f'{p.contact_time_constant_s} 1',rgba='.2 .2 .2 .2')
    spec=dict(schema='gtsd-v5-articulated-cables-v1',source_geometry_sha256=source['source_geometry_sha256'],source_blend_sha256=source['source_blend_sha256'],parameters=asdict(p),routes=records,total_links=sum(r['link_count'] for r in records),total_mass_kg=sum(r['mass_kg'] for r in records),available_route_keys=[k for k,v in ROUTES.items() if v in sources],selected_source_meshes=[r['source_object'] for r in records],hidden_source_meshes=sorted({r['source_object'] for r in source['records']}|{'DETAIL_DRIVE_motor_temperature_lead','DETAIL_DRIVE_encoder_lead','bogie_axle_probe_flexible_lead_1','bogie_axle_probe_flexible_lead_2'}),rendering='SOLVER_CAPSULE_PROXIES_ONLY; hide original flexible meshes; no unverified rigid mesh binding',mass_boundary='Added cable proxy mass is separate from unchanged uncalibrated payload equivalent; true hardware mass partition remains unresolved',limits=['Candidate endpoints bound to source support bodies; actual terminal identity and fixity unverified','Rest polyline, isotropic ball-joint bending/torsion, cylindrical lumped inertia, EI, density and damping unmeasured','No axial stretch, electrical continuity, pressure-radius coupling or door bonding route evidence','Numerical ground plane, not identified vehicle support contact; adjacent two-link contacts excluded'],algorithm_provenance='Reimplemented B11 articulated chain/connect release semantics; NO historical vehicle coordinates/models')
    ET.indent(root);return ET.tostring(root,encoding='unicode'),spec
