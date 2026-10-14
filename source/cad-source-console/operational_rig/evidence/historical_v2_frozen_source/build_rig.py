"""Photo-source assembly, single-owner inertia and native contact force paths."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as X
import numpy as np
try:
    from .mass_properties import integrate, combine, canonical_sha
    from .contact_adapter import add_partition
except ImportError:
    from mass_properties import integrate, combine, canonical_sha
    from contact_adapter import add_partition
ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'gtsd-photo-operational-assembly-v1'
DEFAULTS = json.loads((Path(__file__).parent/'parameters.json').read_text())

def fmt(v):
    return format(float(v), '.17g') if np.isscalar(v) else ' '.join(format(float(x), '.17g') for x in v)

def validate_parameters(parameters=None):
    cfg=copy.deepcopy(DEFAULTS)
    if not isinstance(parameters or {},dict): raise ValueError('Parameters must be an object')
    if set(parameters or {})-set(cfg): raise ValueError('Unknown parameters: '+str(set(parameters)-set(cfg)))
    cfg.update(parameters or {})
    if cfg['mode']!='operational_assembly': raise ValueError('Choose operational_assembly')
    for k,v in cfg.items():
        if isinstance(v,str): continue
        if not np.isfinite(np.asarray(v,float)).all(): raise ValueError('Non-finite '+k)
        if k not in ('gravity_m_s2','wheel_start_x_m','wheel_route_y_m','messenger_datum_shift_mm') and np.any(np.asarray(v)<=0): raise ValueError('Positive value required: '+k)
    if not 0.000025<=cfg['time_step_s']<=0.0002: raise ValueError('Use 25–200 microseconds')
    if not 4.8<=cfg['pin_diameter_mm']<5.0: raise ValueError('Pin diameter range: 4.8–4.999 mm')
    if not 0<=cfg['messenger_datum_shift_mm']<=0.14: raise ValueError('Messenger datum shift range: 0–0.14 mm')
    if not 0.01<=cfg['wheel_radius_m']<=0.1: raise ValueError('Wheel radius range: 10–100 mm')
    return cfg

def cylinder(name,axis,center,lo,hi,radius,n=96):
    uv=[i for i in range(3) if i!=axis];v=[];f=[]
    for h in (lo,hi):
        for a in np.arange(n)*2*np.pi/n:
            p=list(center);p[axis]=h;p[uv[0]]+=radius*np.cos(a);p[uv[1]]+=radius*np.sin(a);v.append(p)
    for h in (lo,hi):
        p=list(center);p[axis]=h;v.append(p)
    for i in range(n):
        j=(i+1)%n;f.extend([[i,j,n+j],[i,n+j,n+i],[2*n,j,i],[2*n+1,n+i,n+j]])
    vv=np.array(v);ff=np.array(f);t=vv[ff]-vv.mean(0)
    if np.einsum('ij,ij->',t[:,0],np.cross(t[:,1],t[:,2]))<0: ff=ff[:,::-1]
    return {'name':name,'vertices':vv.ravel().tolist(),'indices':ff.ravel().tolist(),'color':[.14,.18,.23,1]}

def _load_rows(path):
    raw=gzip.open(path,'rt').read() if str(path).endswith('.gz') else path.read_text()
    obj=json.loads(raw);rows=obj.get('meshes',obj)
    return list(rows.values()) if isinstance(rows,dict) else rows

def source_meshes(root=ROOT,parameters=None):
    """World-metre complete solids used by display, inertia and contact builders."""
    root=Path(root);cfg=validate_parameters(parameters)
    rev=root/'photo_turnout/operational_revision'
    rows=[dict(m,branch='foundation' if m['name'].startswith(('ENV_deck_', 'ENV_substation_')) else 'track',body_name='fixed_foundation' if m['name'].startswith(('ENV_deck_', 'ENV_substation_')) else 'track_assembly') for m in _load_rows(rev/'regional_meshes.json.gz')]
    for original in list(rows):
        if original['name'].startswith('PHOTO_RETAIN_left_ENV_main_rail'):
            m=copy.deepcopy(original);v=np.array(m['vertices']).reshape(-1,3);lo=v[:,0].min();hi=v[:,0].max()
            v[:,0]=lo-cfg['approach_extension_m']+(v[:,0]-lo)/(hi-lo)*cfg['approach_extension_m']
            m['source_parent_name']=m['name'];m['name']='OP_approach_'+m['name'];m['vertices']=v.ravel().tolist()
            m['engineering_geometry_choice']='Complete source cross-section approach extension, meeting original left rail at its minimum X plane'
            rows.append(m)
    profile=json.loads((rev/'operational_source_profile.json').read_text())['profiles'][0]
    for g in profile['groups']:
        for m in g['add_meshes']:
            n=m['name']
            if n.startswith('FIT_BALISE_') and not n.startswith('FIT_BALISE_SOURCE_'):
                rows.append(dict(m,branch='balise',body_name='balise_'+n.split('_')[2]))
            elif n.startswith('FIT_BALISE_SOURCE_') or n.startswith('FIT_TRACK_'):
                # Only outside the regional turnout; identity set below de-duplicates.
                rows.append(dict(m,branch='track',body_name='track_assembly'))
            elif n.startswith('FIT_') and ('4079' in n or 'B535' in n or 'PIN5' in n):
                station='50DF4' if ('518DF' in n or '50DF4' in n) else '51409'
                if 'PIN5' in n:
                    v=np.array(m['vertices']).reshape(-1,3);center=(v.min(0)+v.max(0))/2
                    m=cylinder('OP_PIN_'+station,1,center,v[:,1].min(),v[:,1].max(),cfg['pin_diameter_mm']/2000,192)
                    m['source_parent_name']=n
                    rows.append(dict(m,branch='catenary',body_name='pin_'+station))
                else: rows.append(dict(m,branch='catenary',body_name='catenary_'+station))
    published=_load_rows(root/'mechanical_fit/components/clamps/published_clamp_meshes.json')
    for m in published:
        n=m['name']
        if n=='ENV_messenger_wire':
            m=copy.deepcopy(m);m['vertices']=(np.array(m['vertices']).reshape(-1,3)+[0,0,cfg['messenger_datum_shift_mm']/1000]).ravel().tolist()
            m['source_parent_name']=n;m['name']='OP_messenger_wire';rows.append(dict(m,branch='catenary',body_name='messenger_wire'))
        elif n=='ENV_contact_wire': rows.append(dict(m,branch='catenary',body_name='contact_wire_support'))
        elif ('518DF' in n or '50DF4' in n or '518DD' in n or '518E0' in n or '51409' in n or '518DE' in n):
            if '4079_vertical' in n or 'profile_2' in n or 'profile_1_B535' in n:continue
            station='50DF4' if any(s in n for s in ('518DF','50DF4','518DD')) else '51409'
            rows.append(dict(m,branch='catenary',body_name='catenary_'+station))
    wheel=cylinder('OP_guided_wheel',1,[cfg['wheel_start_x_m'],cfg['wheel_route_y_m'],cfg['wheel_radius_m']+cfg['wheel_initial_clearance_m']],cfg['wheel_route_y_m']-cfg['wheel_width_m']/2,cfg['wheel_route_y_m']+cfg['wheel_width_m']/2,cfg['wheel_radius_m'],192)
    rows.append(dict(wheel,branch='vehicle',body_name='wheel'))
    unique={}
    for m in rows:
        if m['name'] in unique:
            if m['vertices']!=unique[m['name']]['vertices'] or m['indices']!=unique[m['name']]['indices']:raise ValueError('Conflicting source '+m['name'])
            continue
        unique[m['name']]=m
    rows=list(unique.values())
    materials={m['name']:m['material_id'] for m in json.loads((rev/'mesh_interface.json').read_text())['meshes']}
    for m in rows:
        m['material_id']=materials.get(m.get('source_parent_name',m['name']),'steel')
        m['geometry_role']='visual_overlay' if '_rail_top_' in m['name'] else 'reference_context' if m['body_name']=='fixed_foundation' else 'physical_solid'
        m['mass_model']='not_in_dynamic_budget' if m['geometry_role']!='physical_solid' else 'complete_source_volume_once'
        if m['geometry_role']=='visual_overlay':
            m['source_alias_of']=m['name'].replace('_rail_top_','_rail_');m['alias_hidden_by_default']=True
        m['id']=m['name'];m['geometry_sha256']=canonical_sha({'vertices':m['vertices'],'indices':m['indices']})
    return rows,canonical_sha([{k:m[k] for k in ('name','geometry_sha256','body_name')} for m in rows])

def load_geometry(parameters=None,root=ROOT,contract=None):
    rows,sha=source_meshes(root,(contract or {}).get('parameters',parameters))
    return {'schema':'gtsd-operational-geometry-v1','units':'m','source_profile':'operational_assembly_v1','meshes':rows,'source_sha256':sha}

def parameter_sources():
    return json.loads((Path(__file__).parent/'parameter_sources.json').read_text())

def build(parameters=None,root=ROOT):
    root=Path(root);cfg=validate_parameters(parameters);meshes,source_sha=source_meshes(root,cfg)
    groups={}
    for m in meshes:groups.setdefault(m['body_name'],[]).append(m)
    props={}
    for name,group in groups.items():
        if name=='fixed_foundation':
            vv=np.concatenate([np.array(m['vertices']).reshape(-1,3) for m in group]);center=(vv.min(0)+vv.max(0))/2
            props[name]={'hypothesis_id':'FIXED_REFERENCE_CONTEXT','mass_kg':0.,'center_of_mass_world_m':center.tolist(),'inertia_com_world_kg_m2':np.zeros((3,3)).tolist(),'mesh_names':[m['name'] for m in group],'mass_contributions':[],'mass_model':'not_in_dynamic_budget','geometry_role':'reference_envelope_and_fixed_context'}
            continue
        physical=[m for m in group if m['geometry_role']=='physical_solid']
        pp=[integrate(m,density_kg_m3=cfg['source_density_kg_m3' if m['material_id']=='steel' else m['material_id']+'_density_kg_m3']) for m in physical]
        declared=cfg['balise_total_mass_kg'] if name.startswith('balise_') else cfg['wheel_mass_kg'] if name=='wheel' else None
        if declared:
            total=sum(p['mass_kg'] for p in pp)
            pp=[integrate(m,mass_kg=p['mass_kg']*declared/total) for m,p in zip(physical,pp)]
        props[name]=combine(pp,hypothesis_id='OP_RIGID_GROUP_'+name)
    tree=X.Element('mujoco',model=SCHEMA)
    X.SubElement(tree,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
    opt=X.SubElement(tree,'option',timestep=fmt(cfg['time_step_s']),gravity=fmt(cfg['gravity_m_s2']),integrator='implicitfast',solver='Newton',tolerance='1e-11',iterations='80',cone='elliptic',ccd_iterations='80',ccd_tolerance='1e-10')
    X.SubElement(opt,'flag',energy='enable',nativeccd='enable',multiccd='enable')
    default=X.SubElement(tree,'default');X.SubElement(default,'geom',friction=fmt([cfg['friction_coefficient'],.002,.00005]),solref=fmt([cfg['contact_time_constant_s'],1]),solimp='0.95 0.99 0.0001',margin='0',gap='0',mass='0',density='0')
    asset=X.SubElement(tree,'asset');world=X.SubElement(tree,'worldbody');rig=X.SubElement(world,'body',name='rig')
    X.SubElement(rig,'inertial',pos='0 0 0',mass=fmt(cfg['rig_mass_kg']),diaginertia=fmt(cfg['rig_inertia_kg_m2']))
    joint_parameters=[];bodies=[{'name':'rig','branch':'rig','reference_position_m':[0.,0.,0.],'mass_kg':cfg['rig_mass_kg']}]
    def joint(body,name,axis,k,m,kind='slide'):
        c=2*cfg['damping_ratio']*np.sqrt(k*m) if k else 0
        X.SubElement(body,'joint',name=name,type=kind,axis=fmt(axis),stiffness=fmt(k),damping=fmt(c))
        joint_parameters.append({'name':name,'kind':kind,'stiffness':k,'damping':float(c)})
    for n,a,k,m,t in [('rig_y',[0,1,0],cfg['rig_stiffness'][0],cfg['rig_mass_kg'],'slide'),('rig_z',[0,0,1],cfg['rig_stiffness'][1],cfg['rig_mass_kg'],'slide'),('rig_roll',[1,0,0],cfg['rig_stiffness'][2],cfg['rig_inertia_kg_m2'][0],'hinge')]:joint(rig,n,a,k,m,t)
    bindings=[];body_elements={};centers={}
    for name,group in groups.items():
        p=props[name];center=np.array(p['center_of_mass_world_m']);centers[name]=center;t=np.array(p['inertia_com_world_kg_m2'])
        body=X.SubElement(world if name=='fixed_foundation' else rig,'body',name=name,pos=fmt(center));body_elements[name]=body
        if p['mass_kg']:
            X.SubElement(body,'inertial',pos='0 0 0',mass=fmt(p['mass_kg']),fullinertia=fmt([t[0,0],t[1,1],t[2,2],t[0,1],t[0,2],t[1,2]]))
        if name=='fixed_foundation':
            pass
        elif name=='wheel':
            joint(body,'wheel_x',[1,0,0],0,p['mass_kg']);joint(body,'wheel_z',[0,0,1],0,p['mass_kg']);joint(body,'wheel_spin',[0,1,0],0,t[1,1],'hinge')
        else:
            key='track' if name=='track_assembly' else 'balise' if name.startswith('balise') else 'wire' if name=='messenger_wire' else 'pin' if name.startswith('pin') else 'catenary'
            k=cfg[key+'_mount_stiffness_N_m']
            # Bolted track and catenary rigid groups transmit their wrench via explicit finite mounts.
            axes=([0,0,1],) if key in ('track','pin') else ([0,1,0],[0,0,1])
            for j,a in enumerate(axes):joint(body,name+'_translation_'+str(j),a,k,p['mass_kg'])
        branch=group[0]['branch'];bodies.append({'name':name,'branch':branch,'reference_position_m':center.tolist(),'mass_kg':p['mass_kg']})
        for m in group:bindings.append({'mesh_name':m['name'],'body_name':name,'branch':branch,'source_geometry_sha256':m['geometry_sha256'],'body_reference_position_m':center.tolist(),'body_reference_quaternion_wxyz':[1.,0.,0.,0.],'rigid_group_hypothesis':p['hypothesis_id'],'geometry_role':m['geometry_role'],'mass_model':m['mass_model'],'mass_contribution_kg':next((q['mass_kg'] for q in p['mass_contributions'] if q['mesh_name']==m['name']),0.)})
    act=X.SubElement(tree,'actuator');X.SubElement(act,'velocity',name='wheel_drive',joint='wheel_x',kv=fmt(cfg['wheel_drive_gain_N_s_m']))
    colliders=[];geometry_to_interface={}
    def mesh_geom(name,vertices,faces,owner,ctype,caff,interface,source):
        v=np.array(vertices).reshape(-1,3);local_center=(v.min(0)+v.max(0))/2;meshname='M_'+name
        X.SubElement(asset,'mesh',name=meshname,inertia='shell',vertex=fmt((v-local_center).ravel()),face=' '.join(map(str,np.array(faces).ravel())))
        X.SubElement(body_elements[owner],'geom',name=name,type='mesh',mesh=meshname,pos=fmt(local_center-centers[owner]),contype=str(ctype),conaffinity=str(caff),solref=fmt([cfg['wheel_contact_time_constant_s'] if interface=='wheel_rail' else cfg['contact_time_constant_s'],1]))
        geometry_to_interface[name]=interface
        colliders.append({'geom_name':name,'source_mesh_name':source,'body_name':owner,'interface_id':interface,'mass_kg':0.})
    # Rail collision: exact original top triangles, extruded inward by 1 mm.
    # This preserves frog/flangeway/end-gap boundaries without a whole-rail convex hull.
    shell_witness=[]
    def surface_cells(m,selector,depth,ctype,caff,interface,prefix):
        v=np.array(m['vertices']).reshape(-1,3);f=np.array(m['indices']).reshape(-1,3);tri=v[f];nn=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);length=np.linalg.norm(nn,axis=1);valid=length>1e-16;nn[valid]/=length[valid,None]
        ids=np.where(valid & selector(tri,nn))[0];cnt=0
        for i in ids:
            t=tri[i];normal=nn[i];vv=np.r_[t,t-depth*normal];ff=[[0,1,2],[3,5,4],[0,3,4],[0,4,1],[1,4,5],[1,5,2],[2,5,3],[2,3,0]]
            mesh_geom(prefix+str(i),vv,ff,m['body_name'],ctype,caff,interface,m['name']);cnt+=1
        shell_witness.append({'source_mesh_name':m['name'],'interface_id':interface,'source_surface_triangles':cnt,'inward_depth_m':depth,'surface_offset_m':0.,'method':'exact source triangle boundary; inward triangular convex prisms'})
    for m in meshes:
        if m['branch']=='track' and m['geometry_role']=='physical_solid' and (m['name'].startswith(('PHOTO_FROG','PHOTO_WING','PHOTO_RETAIN','OP_approach_'))):
            surface_cells(m,lambda t,n:(n[:,2]>.999)&(np.max(abs(t[:,:,2]),axis=1)<1e-7),.001,0,1,'wheel_rail','rail_'+m['name']+'_')
        elif m['name']=='OP_guided_wheel':mesh_geom('wheel_tread',m['vertices'],m['indices'],'wheel',1,0,'wheel_rail',m['name'])
        elif 'jaw_lower' in m['name'] or 'jaw_upper' in m['name']:
            # The messenger groove at Y=0; upper/lower real groove surfaces.
            islower='jaw_lower' in m['name']
            surface_cells(m,lambda t,n:(n[:,2]>.05 if islower else n[:,2]<-.05)&(np.max(abs(t[:,:,1]),axis=1)<.003)&(np.min(t[:,:,2],axis=1)>2.095),.0002,0,8,'messenger_jaw','jaw_'+m['name']+'_')
        elif m['name']=='OP_messenger_wire':mesh_geom('messenger_cylinder',m['vertices'],m['indices'],'messenger_wire',8,0,'messenger_jaw',m['name'])
    partition=json.loads((root/'research_rig/contact_assets/core_partition.json').read_text())
    byname={m['name']:m for m in meshes}
    for station,bit in [('50DF4',2),('51409',4)]:
        for source in ['FIT_'+station+'_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE','CAD_'+station+'_F643_profile_0']:
            part=next(p for p in partition['parts'] if p['source_name']==source)
            old=byname[source]
            if part['source_geometry_sha256']!=canonical_sha({'name':old['name'],'vertices':old['vertices'],'indices':old['indices']}):raise ValueError('Pin partition/source mismatch')
            owner='catenary_'+station;names=add_partition(asset,body_elements[owner],part,prefix='OP_'+source,body_reference_world_m=centers[owner],contype=0,conaffinity=bit)
            for name in names:
                geometry_to_interface[name]='pin_'+station;colliders.append({'geom_name':name,'source_mesh_name':source,'body_name':owner,'interface_id':'pin_'+station,'mass_kg':0.})
        pin=byname['OP_PIN_'+station];mesh_geom('pin_'+station,pin['vertices'],pin['indices'],'pin_'+station,bit,0,'pin_'+station,pin['name'])
    if not any(w['interface_id']=='wheel_rail' and w['source_surface_triangles'] for w in shell_witness):raise ValueError('No exact source rail-top colliders')
    xml=X.tostring(tree,encoding='unicode')
    contract={'schema':SCHEMA,'source_profile':'operational_assembly_v1','profile_sha256':source_sha,'parameters':cfg,'parameter_sources':parameter_sources(),'bindings':bindings,'bodies':bodies,'inertial_properties':props,'joint_parameters':joint_parameters,'collision_enabled':True,'contact_acceptance':True,'scope':'PHOTO_TURNOUT_BALISE_CATENARY_COUPLED_ASSEMBLY','collision_scope':'wheel/source rail tops; two pin/source bore-slot partitions; messenger/source jaw grooves','colliders':colliders,'geometry_to_interface':geometry_to_interface,'surface_contact_witness':shell_witness,'native_contact_interfaces':[{'interface_id':i,'owner':'native_contact','boundary':'finite mounts transmit to common rig'} for i in ('wheel_rail','pin_50DF4','pin_51409','messenger_jaw')],'assembly_choices':[{'interface':'rail/seat/bolt/nut/washer/plate','model':'rigid weld within track_assembly; complete source masses counted once'},{'interface':'tube/ring/stem/4079/carrier','model':'rigid weld within each catenary station; finite station mount'},{'interface':'balise housing/lid/screws','model':'rigid instrument assembly; finite installation springs'},{'interface':'contact wire/4079','model':'equivalent finite installation; messenger jaw remains actual contact'},{'interface':'guided wheel route','model':'one guided cylindrical wheel, X velocity drive, free Z translation and Y rotation'}],'mass_accounting':{'bound_source_meshes':len(meshes),'dynamic_source_solids':sum(m['geometry_role']=='physical_solid' for m in meshes),'visual_overlay_meshes':sum(m['geometry_role']=='visual_overlay' for m in meshes),'fixed_context_meshes':len(groups.get('fixed_foundation',[])),'unique_source_solids':len(set(m['name'] for m in meshes)),'source_mass_kg':sum(p['mass_kg'] for p in props.values()),'rig_mass_kg':cfg['rig_mass_kg'],'collision_mass_kg':0.},'mass_exclusions':[{'mesh_name':m['name'],'reason':m['geometry_role'],'source_alias_of':m.get('source_alias_of'),'mass_kg':0.} for m in meshes if m['geometry_role']!='physical_solid'],'demo':{'duration_s':cfg['demo_duration_s'],'description':'Smooth wheel passage over the complete photo-source butt gap and frog, with balise/catenary pulses on the common finite support','drive':'native velocity actuator, smooth ramp; force and work recorded','wheel_route_y_m':cfg['wheel_route_y_m'],'route_events_x_m':{'butt_gap':-9.470039418889508,'frog_wing_gap_midpoint':-8.776412324,'frog_nose':-8.6923727522,'wing_to_main':-8.323039419}},'as_built':False,'old_checkpoint_allowed':False,'force_owners':{'body_mounts':'native_joint_spring_damping','rig_world':'native_joint_spring_damping','wheel_drive':'native_velocity_actuator','interfaces':'native_contact'},'xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),'source_code_sha256':{f:hashlib.sha256((Path(__file__).parent/f).read_bytes()).hexdigest() for f in ('build_rig.py','mass_properties.py','contact_adapter.py')}}
    contract['identity_sha256']=canonical_sha(contract)
    return xml,contract
