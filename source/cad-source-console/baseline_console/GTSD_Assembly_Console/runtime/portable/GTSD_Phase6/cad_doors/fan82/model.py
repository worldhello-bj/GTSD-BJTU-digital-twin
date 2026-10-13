"""Three further native fan mounts, each split once from its 0.6 kg door."""
import copy,xml.etree.ElementTree as ET
from dataclasses import dataclass,asdict
import numpy as np,mujoco
from cad_doors.model import inertial,digest
from cad_doors.geometry import read_sources
from cad_doors.moments import integrate,combine,physical
from aux_mount.model import vec,spatial_mass_properties
KEYS=('end_neg_y_pos','end_pos_y_neg','end_pos_y_pos')
ALL_KEYS=('end_neg_y_neg',*KEYS)
NAMES={k:'door_fan_'+k for k in KEYS}

def augment(xml,metadata,door_spec):
 root=ET.fromstring(xml);base=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(base);mujoco.mj_forward(base,d)
 c,g=read_sources();meshes={r['name']:r for r in g['meshes']};solid={n:integrate(r) for n,r in meshes.items() if r.get('solid')};shift=np.asarray(c['component_to_reference_world_translation_m']);spec={'schema':'gtsd-four-door-fan-native-model-v1','added_mounts':{},'door_residuals':{},'assumptions':['Same-spec engineering assumption, not evidence of identical motor model.','Each door 0.54 kg residual + 0.04 kg stator + 0.02 kg rotor. Uniform closed-solid weighting is not material density.','Independent adjustable voltage/open-circuit branches on ideal 24 V sources; no wiring impedance, shared supply dynamics or measured control topology.','Existing rotor tensor copied as uncalibrated engineering assumption; stator tensor comes from its exact CAD candidate ring.']}
 source=root.find('.//body[@name="bay_fan_stator"]');sensor=root.find('sensor');asset=root.find('asset')
 for k in KEYS:
  s=c['doors'][k];n=NAMES[k];body=root.find('.//body[@name="door_'+k+'"]');h=np.asarray(door_spec['hinges'][k]['point_world_m']);center=np.asarray(s['fan_center_world_m']);shroud=s['fan_target_mesh_name']
  alloc=combine([solid[x] for x in s['mesh_names'] if x in solid and x!=shroud],.54,shift,h);inertial(body,alloc);spec['door_residuals'][k]=alloc
  stator=copy.deepcopy(source)
  for item in list(stator):
   if item.tag=='geom':stator.remove(item)
  for node in stator.iter():
   for attr in ('name',):
    if node.get(attr):node.set(attr,node.get(attr).replace('bay_fan',n))
  stator.set('pos',vec(center-h));stat=combine([solid[shroud]],.04,shift,center);inertial(stator,stat);body.append(stator)
  # Native shroud vertices were in door coordinates; relocate to actual stator.
  row=door_spec['native_visuals'][shroud];geom=body.find('geom[@name="'+row['geom']+'"]');body.remove(geom);stator.append(geom)
  mesh=asset.find('mesh[@name="'+geom.get('mesh')+'"]');v=np.fromstring(mesh.get('vertex'),sep=' ').reshape(-1,3);mesh.set('vertex',vec((v+h-center).reshape(-1)))
  for typ in ('force','torque'):ET.SubElement(sensor,typ,name=n+'_mount_site_'+typ,site=n+'_mount_site',cutoff='0',noise='0')
  spec['added_mounts'][k]={'stator':n+'_stator','rotor':n+'_rotor','joint':n+'_rotor','site':n+'_mount_site','parent':'door_'+k,'source_shroud_mesh':shroud,'reference_center_world_m':center.tolist(),'stator_allocation':stat,'rotor_mass_kg':.02,'rotor_tensor_source':'Frozen bay_fan_rotor research tensor, copied without scaling'}
  for row in metadata['doors']['doors']:
   if row['key']==k:row.update(mass_kg=.54,com_source_m=(h+alloc['com_local_m']).tolist(),inertia_com_source_full_kgm2=alloc['inertia_com_local_kgm2'],inertia_com_source_diagonal_kgm2=np.diag(alloc['inertia_com_local_kgm2']).tolist())
 metadata['doors']['identity_sha256']=digest({k:v for k,v in metadata['doors'].items() if k!='identity_sha256'})
 ET.indent(root);out=ET.tostring(root,encoding='unicode');m=mujoco.MjModel.from_xml_string(out);md=mujoco.MjData(m);mujoco.mj_forward(m,md)
 spec['reference_spatial_before']=spatial_mass_properties(base,d);spec['reference_spatial_after']=spatial_mass_properties(m,md)
 assert (m.nq,m.nv)==(94,82) and abs(sum(m.body_mass)-sum(base.body_mass))<1e-11
 for i in range(1,m.nbody):
  if m.body_mass[i]:physical(np.diag(m.body_inertia[i]),m.body(i).name)
 spec['mass_budget_kg']={'residual_payload':26.6,'four_residual_doors':2.16,'four_native_fans':.24,'other_auxiliaries':1.,'original_payload_budget':30.}
 spec['identity_sha256']=digest(spec);return out,spec
