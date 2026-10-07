"""Read-only compatibility checks for a source-topology B8 replay and MJCF."""
from pathlib import Path
import json,hashlib,math
import numpy as np
from xml.etree import ElementTree as ET

def matrix(t):
    w,x,y,z=map(float,t['quaternion_wxyz']);n=math.sqrt(w*w+x*x+y*y+z*z)
    if n<.5:raise ValueError('Invalid quaternion')
    w,x,y,z=[v/n for v in (w,x,y,z)]
    m=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0],[0,0,0,1]],float)
    m[:3,3]=t['position'];return m

def inspect(trajectory,xml,final=False):
    jb=Path(trajectory).read_bytes();xb=Path(xml).read_bytes();j=json.loads(jb);doc=ET.fromstring(xb)
    refs=j['reference_zero_qpos_body_transforms'];frames=j['frames'];xml_bodies={};sites={};world_geoms=[]
    def vec(value,default):return list(map(float,value.split())) if value else default
    def walk(parent,parent_m,owner):
        for el in parent:
            if el.tag=='body':
                name=el.get('name');assert name,'Unnamed body is unsupported'
                if el.get('euler') or el.get('axisangle') or el.get('xyaxes') or el.get('zaxis'):raise ValueError('Body orientation representation not supported: '+name)
                tr={'position':vec(el.get('pos'),[0,0,0]),'quaternion_wxyz':vec(el.get('quat'),[1,0,0,0])};m=parent_m@matrix(tr)
                xml_bodies[name]={'world_matrix':m,'element':el,'parent':owner};walk(el,m,name)
            elif el.tag=='site' and el.get('name'):
                sites[el.get('name')]={'body':owner,'position':vec(el.get('pos'),[0,0,0]),'quat':vec(el.get('quat'),[1,0,0,0])}
            elif el.tag=='geom' and owner is None:world_geoms.append(dict(el.attrib))
    walk(doc.find('worldbody'),np.eye(4),None)
    refnames=set(refs);xmlnames=set(xml_bodies)
    if refnames!=xmlnames:raise ValueError(f'XML / reference body mismatch: only replay={sorted(refnames-xmlnames)}, only XML={sorted(xmlnames-refnames)}')
    required={'A_carbody','B_carbody','A_frame','B_frame','A_front_wheelset','A_rear_wheelset','B_front_wheelset','B_rear_wheelset','A_motor_rotor','panto_lower_arm','panto_upper_arm','panto_lower_balance','panto_upper_balance','panto_elbow_carrier','panto_collector','panto_mount'}
    if not required<=refnames:raise ValueError('Source topology missing: '+str(sorted(required-refnames)))
    if 'B_motor_rotor' in refnames:raise ValueError('B is the unpowered source trailer; unexpected second motor')
    reference_error=max(float(np.max(np.abs(matrix(refs[n])-xml_bodies[n]['world_matrix']))) for n in refs)
    if reference_error>2e-7:raise ValueError('Zero-qpos transform mismatch '+str(reference_error))
    previous=-math.inf
    for rec in frames:
        if set(rec['body_transforms'])!=refnames:raise ValueError('Frame body set changed at '+str(rec['time_s']))
        if not rec['time_s']>previous:raise ValueError('Non-increasing replay time')
        previous=rec['time_s']
        for n,t in rec['body_transforms'].items():
            vals=t['position']+t['quaternion_wxyz']
            if len(t['position'])!=3 or len(t['quaternion_wxyz'])!=4 or not all(math.isfinite(x) for x in vals):raise ValueError('Non-finite body state')
            if abs(sum(x*x for x in t['quaternion_wxyz'])-1)>1e-5:raise ValueError('Quaternion not normalized')
    xhash=hashlib.sha256(xb).hexdigest();embedded=j.get('model_xml_sha256',j.get('xml_sha256'))
    if embedded and embedded!=xhash:raise ValueError('Pinned XML SHA256 does not match replay')
    if final and not embedded:raise ValueError('Final export requires replay model_xml_sha256')
    p=j['parameters']
    for key,value in [('bogie_spacing_m',2.6),('car_center_height_m',1.095),('bogie_frame_height_m',.37),('car_length_m',2.5),('car_width_m',.95)]:
        if abs(p.get(key,math.inf)-value)>1e-6:raise ValueError('Source geometry parameter mismatch: '+key)
    report={'status':'FINAL_INPUT_CONTRACT_PASSED' if final else 'PROVISIONAL_INPUT_CONTRACT_PASSED','trajectory_sha256':hashlib.sha256(jb).hexdigest(),'xml_sha256':xhash,'embedded_model_hash_present':bool(embedded),'body_names':sorted(refnames),'bodies':len(refnames),'frames':len(frames),'duration_s':frames[-1]['time_s'],'zero_reference_max_matrix_error':reference_error,'source_layout':'Two source-model-assumption bodies, one bogie each, A powered, original Z-link pantograph','physical_validation':'Input identity and representation checks only; physical validation belongs to solver reports.'}
    return j,doc,sites,world_geoms,report,jb,xb
