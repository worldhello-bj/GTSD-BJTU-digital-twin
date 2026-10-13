"""True native hinge, spatial-catch, inertia and selected fan-mount migration."""
from dataclasses import dataclass,asdict
import copy,hashlib,json,math
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from .bootstrap import PATHS
from .geometry import read_sources
from .moments import integrate,combine,pa,physical
from aux_mount.model import _inertial_attributes,spatial_mass_properties,vec,quatmat
from cad_body.geometry import read_sources as body_sources

@dataclass(frozen=True)
class Parameters:
 hinge_axial_depth_delta_m:float=0.
 latch_time_constant_s:float=.002
 contact_time_constant_s:float=.003
 contact_friction:float=.25
 contact_default_enabled:bool=False
 def __post_init__(self):
  for n,v in asdict(self).items():
   if n=='contact_default_enabled':
    if type(v) is not bool:raise ValueError('Contact default must be boolean')
   elif type(v) not in (int,float) or not math.isfinite(v):raise ValueError('Finite parameter required: '+n)
  if abs(self.hinge_axial_depth_delta_m)>.01:raise ValueError('Hinge-depth research variation limited to 10 mm')
  if min(self.latch_time_constant_s,self.contact_time_constant_s,self.contact_friction)<=0:raise ValueError('Positive contact/latch parameters required')

def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def inertial(node,row):
 i=node.find('inertial');i.attrib.clear();i.attrib.update(_inertial_attributes(row['mass_kg'],row['com_local_m'],row['inertia_com_local_kgm2']))

def augment_xml(xml,metadata,aux_spec,cad_spec,parameters):
 c,geometry=read_sources();p=parameters;root=ET.fromstring(xml);base=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(base);mujoco.mj_forward(base,d)
 if min(p.contact_time_constant_s,p.latch_time_constant_s)<2*base.opt.timestep:raise ValueError('Contact and latch time constants must be at least twice dt')
 if any(g.get('name','').startswith('cad7_') for g in root.iter()):raise ValueError('Door migration already applied')
 raw={m['name']:m for m in geometry['meshes']};solid={n:integrate(m) for n,m in raw.items() if m.get('solid')};shift=np.asarray(c['component_to_reference_world_translation_m'])
 payload=root.find('.//body[@name="payload_secondary"]');pbid=base.body('payload_secondary').id;origin=d.xpos[pbid];replace=c['replacement_manifest'];removed=set(replace['replaced_cad_body_mesh_names'])
 old_cad=body_sources()[0];keep=[m for m in old_cad['meshes'] if m['name'] not in removed]
 if len(keep)!=16 or len(removed)!=10:raise ValueError('Expected exactly ten replaced end ribs and sixteen retained body solids')
 remaining=combine([integrate(m) for m in keep]+[solid[n] for n in c['frame_parts']],26.6,shift,origin);inertial(payload,remaining)
 # Delete both native visual geom and matching mesh asset for replaced ribs.
 for i,m in enumerate(old_cad['meshes']):
  if m['name'] in removed:
   for parent in root.iter():
    for node in list(parent):
     if node.tag in ('geom','mesh') and node.get('name')=='cad_body_visual_'+str(i):parent.remove(node)
 dm=copy.deepcopy(metadata['doors']);aux=copy.deepcopy(aux_spec);old_dm=copy.deepcopy(dm);new_rows={};latches={};native_visuals={}
 for row in dm['doors']:
  key=row['key'];source=c['doors'][key];name=row['body_name'];body=root.find('.//body[@name="'+name+'"]');end=-1 if 'end_neg' in key else 1
  h=np.asarray(source['hinge_world_m'],dtype=float);h[0]+=end*p.hinge_axial_depth_delta_m
  body.set('pos',vec(h-origin));body.set('quat','1 0 0 0');joint=body.find('joint');joint.set('axis',vec(source['hinge_axis_unit']));joint.set('pos','0 0 0')
  member=[solid[n] for n in source['mesh_names'] if n in solid and (key!='end_neg_y_neg' or n!=source['fan_target_mesh_name'])]
  mass=.54 if key=='end_neg_y_neg' else .6;allocation=combine(member,mass,shift,h);inertial(body,allocation);new_rows[key]=allocation
  proxy=body.find('geom[@name="'+name+'_noncontact_proxy"]')
  if proxy is not None:body.remove(proxy)
  row.update(hinge_source_m=h.tolist(),hinge_payload_m=(h-origin).tolist(),axis_source=source['hinge_axis_unit'],com_source_m=(h+allocation['com_local_m']).tolist(),
   mass_kg=mass,inertia_com_source_diagonal_kgm2=np.diag(allocation['inertia_com_local_kgm2']).tolist(),inertia_com_source_full_kgm2=allocation['inertia_com_local_kgm2'],
   assembly_bounds_source_m=[[float(x[0]+shift[i]) for i,x in enumerate(source['leaf_bounds_component_m'])],[float(x[1]+shift[i]) for i,x in enumerate(source['leaf_bounds_component_m'])]],
   new_mesh_names=source['mesh_names'],mounting_evidence='CAD_OPENING_WITH_PHOTO_LEAF_AND_EXPLICIT_UNMEASURED_HINGE_DEPTH')
  # Preserve original source_parts only for the predecessor binding-builder pass;
  # the final exact-name geometry overlay replaces all those old source objects.
  point=np.asarray(source['latch_point_world_m']);s1=name+'_latch_leaf_site';s2=name+'_latch_frame_site'
  ET.SubElement(body,'site',name=s1,pos=vec(point-h),size='.002',rgba='.95 .6 .1 1')
  ET.SubElement(payload,'site',name=s2,pos=vec(point-origin),size='.002',rgba='.2 .7 .95 1')
  eq=root.find('.//equality/*[@name="'+row['equality_name']+'"]');eq.tag='connect';eq.attrib.clear();eq.attrib.update(name=row['equality_name'],site1=s1,site2=s2,active='true',solref=vec([p.latch_time_constant_s,1]),solimp='.99 .999 .001')
  latches[key]=dict(equality=row['equality_name'],leaf_site=s1,frame_site=s2,reference_point_world_m=point.tolist(),boundary='Ideal spatial catch at the free edge. Frame-side support and lock hardware are unmeasured assumptions; no rigid middle post is inferred.')
  if key=='end_neg_y_neg':
   fan=np.asarray(source['fan_center_world_m']);stator=body.find('body[@name="bay_fan_stator"]');stator.set('pos',vec(fan-h));stator.set('quat','1 0 0 0')
   shroud=solid[source['fan_target_mesh_name']];shell=combine([shroud],.04,shift,fan);inertial(stator,shell)
   for a in aux['mass_ledger']['added_bodies']:
    if a['body'] in ('bay_fan_stator','bay_fan_rotor'):
     a['com_source_world_m']=fan.tolist();a['position_parent_m']=(fan-h).tolist() if a['body']=='bay_fan_stator' else [0.,0.,0.]
     a['com_budget_local_m']=(fan-h).tolist();a['mounting_evidence']='PHOTO_CANDIDATE_RELOCATED_TO_NEW_CAD_LEAF'
     if a['body']=='bay_fan_stator':a['inertia_com_source_world_kgm2']=shell['inertia_com_local_kgm2'];a['principal_inertias_kgm2']=shell['principal_inertias_kgm2']
   aux['mounts']['bay_fan']['reference_position_world_m']=fan.tolist()
 # Native solids match exact meshes. Open grilles/decals remain exact browser
 # pose-bound visuals; no fake thickness is added to satisfy native mesh rules.
 asset=root.find('asset')
 for i,m in enumerate(geometry['meshes']):
  key=m.get('door_key');bn='door_'+key if key else 'payload_secondary';node=root.find('.//body[@name="'+bn+'"]');o=np.asarray(next(r['hinge_source_m'] for r in dm['doors'] if r['key']==key)) if key else origin
  # The selected fixed shroud is attached to the actual native stator body.
  if key=='end_neg_y_neg' and m['name']==c['doors'][key]['fan_target_mesh_name']:
   bn='bay_fan_stator';node=root.find('.//body[@name="bay_fan_stator"]');o=np.asarray(c['doors'][key]['fan_center_world_m'])
  if not m.get('solid'):
   native_visuals[m['name']]=dict(geom=None,body=bn,visual_status='BROWSER_EXACT_POSE_BOUND_ONLY_NONVOLUMETRIC_SOURCE');continue
  local=np.asarray(m['vertices']).reshape(-1,3)+shift-o;n='cad7_visual_'+str(i)
  ET.SubElement(asset,'mesh',name=n,vertex=vec(local.reshape(-1)),face=' '.join(map(str,m['indices'])))
  ET.SubElement(node,'geom',name=n,type='mesh',mesh=n,mass='0',contype='0',conaffinity='0',rgba=vec(m['color']));native_visuals[m['name']]=dict(geom=n,body=bn)
 # Contact proxy construction is intentionally a MATERIAL SUBSET. The frame
 # opening/fan bore is never filled by a convex hull. All boxes are independently
 # checked against source cap triangles before export (audit script).
 collision=[];pairs=[];contact=root.find('contact')
 if contact is None:contact=ET.SubElement(root,'contact')
 yl=c['CAD_main_opening_mm']['Y_min']/1000;yh=c['CAD_main_opening_mm']['Y_max']/1000;zl=.0275;zh=.4775
 for end in (-1,1):
  fx=(end*.647,end*.650);frameboxes=[(min(fx),max(fx),yl-.004,yl,zl+.008,zh-.008),(min(fx),max(fx),yh,yh+.004,zl+.008,zh-.008),(min(fx),max(fx),yl+.012,yh-.012,zh,zh+.004)]
  fns=[]
  for j,bounds in enumerate(frameboxes):
   n='cad7_frame_contact_'+str(end)+'_'+str(j);add_box(payload,n,bounds,origin,shift,p);collision.append(dict(geom=n,body='payload_secondary',role='frame_inner_material_subset',component_bounds_m=list(bounds),source_mesh='CAD_end_frame_'+format(end,'+d')+'_2B75_2B77'));fns.append(n)
  for key,s in c['doors'].items():
   if ('end_neg' in key)!=(end<0):continue
   h=np.asarray(next(r['hinge_source_m'] for r in dm['doors'] if r['key']==key));node=root.find('.//body[@name="door_'+key+'"]');(xl,xh),(a,b),(low,high)=s['leaf_bounds_component_m'];w=.003
   boxes=[(xl,xh,a,a+w,low,high),(xl,xh,b-w,b,low,high),(xl,xh,a+w,b-w,low,low+w),(xl,xh,a+w,b-w,high-w,high)]
   for j,bounds in enumerate(boxes):
    n='cad7_leaf_contact_'+key+'_'+str(j);add_box(node,n,bounds,h,shift,p);collision.append(dict(geom=n,body='door_'+key,role='leaf_perimeter_material_subset',component_bounds_m=list(bounds),source_mesh=s['leaf_mesh_name']))
    for fn in fns:
     pn='cad7_pair_'+str(len(pairs));ET.SubElement(contact,'pair',name=pn,geom1=n,geom2=fn,condim='3',friction=vec([p.contact_friction,p.contact_friction,.001,.0001,.0001]),solref=vec([p.contact_time_constant_s,1]),solimp='.9 .99 .001',margin='0' if p.contact_default_enabled else '-1',gap='0');pairs.append(pn)
 dm['evidence']='Four CAD-opening-derived candidate leaves, actual native hinges and spatial catches; candidate hardware and clearances.'
 dm['latch_boundary']='Four native site-to-site connect constraints at declared free-edge points. Ideal catches with unchanged capture gates, not real lock hardware.'
 dm['collision_boundary']='Optional explicitly paired perimeter/jamb subset contacts; default disabled. Not a full door/shell/equipment collision model.'
 dm['mass_ledger']={'basis':'New geometry distribution preserving scalar budgets only; no historical thin-sheet tensor subtraction','original_payload_mass_kg':30.,'residual_payload_mass_kg':26.6,'door_budget_each_kg':.6,'selected_native_fan_mass_kg':.06,'new_door_residual_allocations':new_rows}
 dm['cad_source_mesh_sha256']=c['mesh_sha256'];dm['identity_sha256']=digest({k:v for k,v in dm.items() if k!='identity_sha256'});metadata['doors']=dm
 aux['cad_door_migration']={'source_mesh_sha256':c['mesh_sha256'],'selected_door':'end_neg_y_neg','bay_fan_reference_world_m':c['doors']['end_neg_y_neg']['fan_center_world_m'],'stator_inertia_recomputed_from':'CAD_fan_shroud_end_neg_y_neg','remaining_three_fans':'Rigid door-mounted visual/allocated mass only, no independent rotor DOF'}
 aux['mass_ledger']['historical_allocation_note']='Original subtraction proof retained as provenance; door mass/inertia/mount spatial moments superseded by CAD door migration.'
 aux['identity_sha256']=digest({k:v for k,v in aux.items() if k!='identity_sha256'})
 ET.indent(root);out=ET.tostring(root,encoding='unicode');model=mujoco.MjModel.from_xml_string(out);md=mujoco.MjData(model);mujoco.mj_forward(model,md)
 before=spatial_mass_properties(base,d);after=spatial_mass_properties(model,md)
 if abs(after['mass_kg']-before['mass_kg'])>1e-11 or (model.nv,model.nq)!=(79,91):raise ValueError('Door migration changed mass/DOFs')
 for i in range(1,model.nbody):
  if model.body_mass[i]:physical(np.diag(model.body_inertia[i]),model.body(i).name)
 # Refresh current native body-frame values after changed hinge/fan allocation.
 for a in aux['mass_ledger']['added_bodies']:
  if a['body'] in ('bay_fan_stator','bay_fan_rotor'):
   bid=model.body(a['body']).id;budget=model.body('door_end_neg_y_neg').id;R=md.xmat[budget].reshape(3,3);cl=R.T@(md.xipos[bid]-md.xpos[budget]);ir=md.ximat[bid].reshape(3,3);ic=R.T@ir@np.diag(model.body_inertia[bid])@ir.T@R
   a.update(com_source_world_m=md.xipos[bid].tolist(),com_budget_local_m=cl.tolist(),inertia_com_budget_local_kgm2=ic.tolist(),inertia_budget_origin_kgm2=(ic+pa(float(model.body_mass[bid]),cl)).tolist())
 aux['identity_sha256']=digest({k:v for k,v in aux.items() if k!='identity_sha256'})
 spec=dict(schema='gtsd-cad-door-native-model-v1',parameters=asdict(p),source_mesh_sha256=c['mesh_sha256'],opening=c['CAD_main_opening_mm'],geometry_assumptions=c['assembly_assumptions'],mass_budget_kg={'payload_residual':26.6,'door_residuals':{k:r['mass_kg'] for k,r in new_rows.items()},'selected_fan':.06,'other_auxiliaries':1.,'total_original_payload':30.},payload_allocation=remaining,door_allocations=new_rows,hinges={r['key']:{'point_world_m':r['hinge_source_m'],'axis':r['axis_source'],'joint':r['joint_name'],'qpos_address':int(model.jnt_qposadr[model.joint(r['joint_name']).id]),'dof_address':int(model.jnt_dofadr[model.joint(r['joint_name']).id])} for r in dm['doors']},latches=latches,collision_geoms=collision,contact_pairs=pairs,contact_switch_method='Explicit pair margins 0 m enabled / -1 m disabled; all proxy geoms have zero broadphase masks. Parent-child contacts are only these named explicit pairs.',contact_coverage='Three verified inner-jamb/top material strips per end and four true leaf-perimeter strips. No contact coverage for bottom frame, leaf interiors, fan holes, attachments, other shell or equipment.',native_visuals=native_visuals,removed_cad_end_ribs=sorted(removed),reference_spatial_before=before,reference_spatial_after=after,geometry_solid_moments=solid,calibrated=False)
 cs=copy.deepcopy(cad_spec);cs.update(equivalent_weighting_kg_per_m3=26.6/remaining['source_total_volume_m3'],reference_spatial_before=before,phase7_body_replacement={'removed_end_ribs':sorted(removed),'added_end_frames':c['frame_parts']},residual_com_body_local_m=remaining['com_local_m'],residual_inertia_com_body_local_kgm2=remaining['inertia_com_local_kgm2'],principal_inertias_kgm2=remaining['principal_inertias_kgm2'],parts=remaining['parts'],reference_spatial_after=after)
 return out,spec,aux,cs

def add_box(parent,name,bounds,origin,shift,p):
 xl,xh,yl,yh,zl,zh=bounds;c=np.array([(xl+xh)/2,(yl+yh)/2,(zl+zh)/2])+shift-origin;half=np.array([xh-xl,yh-yl,zh-zl])/2
 ET.SubElement(parent,'geom',name=name,type='box',pos=vec(c),size=vec(half),mass='0',contype='0',conaffinity='0',rgba='.8 .25 .1 .08',group='5')
