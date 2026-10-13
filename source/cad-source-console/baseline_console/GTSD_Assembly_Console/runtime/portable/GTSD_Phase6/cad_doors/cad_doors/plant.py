"""Independent 79-DOF CAD-door variant with native spatial catches."""
import copy,hashlib,json,math
from pathlib import Path
import mujoco,numpy as np
from . import bootstrap
from cad_body.plant import CadBodyPlant
from aux_mount.plant import ROTORS,AUX_COLUMNS,BASE_WORK_COLUMNS,STATE_KIND,sha,canonical
from .model import Parameters,augment_xml
from .geometry import resource,patch_binding,read_sources

class CadDoorsPlant(CadBodyPlant):
 def __init__(self,dt=.0002,*,include_cables=True,brake_parameters=None,support_parameters=None,auxiliary_parameters=None,door_parameters=None):
  self.door_parameters=Parameters(**door_parameters) if isinstance(door_parameters,dict) else door_parameters or Parameters()
  self.door_contact_enabled=self.door_parameters.contact_default_enabled
  super().__init__(dt,include_cables=include_cables,brake_parameters=brake_parameters,support_parameters=support_parameters,auxiliary_parameters=auxiliary_parameters)
  self.model_version='cad-doors-79dof-spatial-catch-research'
  self.binding=patch_binding(self.binding,self.model,self.data)
  for mesh,row in self.door_spec['native_visuals'].items():
   if row['body']=='bay_fan_stator':next(r for r in self.binding['meshes'] if r['mesh_name']==mesh)['body_name']='bay_fan_stator'
  self.binding['cad_door_augmentation']=copy.deepcopy(self.door_spec)
  self.identity.update(schema='gtsd-cad-doors-mounted-model-identity-v1',source_geometry_sha256=resource()[1])
  self.identity['extensions'].update(cad_doors=copy.deepcopy(self.door_spec),door_geometry_overlay=copy.deepcopy(resource()[2]),door_readonly_dependencies=copy.deepcopy(bootstrap.MANIFEST))
  for n in ('__init__.py','bootstrap.py','geometry.py','moments.py','model.py','plant.py'):
   self.identity['extensions']['source_sha256']['cad_doors/'+n]=hashlib.sha256((Path(__file__).parent/n).read_bytes()).hexdigest()
  self.identity['extensions']['source_sha256']['door_READONLY_DEPENDENCIES.json']=hashlib.sha256((bootstrap.ROOT/'READONLY_DEPENDENCIES.json').read_bytes()).hexdigest()
  self.mass_ledger.update(cad_doors=copy.deepcopy(self.door_spec),reference_com_world_m=self.door_spec['reference_spatial_after']['com_world_m'],reference_inertia_about_com_world_kgm2=self.door_spec['reference_spatial_after']['inertia_world_com_kgm2'],boundary='Original 30 kg payload budget: 26.6 kg fixed residual + four complete 0.6 kg door assemblies + 1.0 kg other auxiliaries. Selected door is 0.54 kg residual + 0.06 kg native fan; other three fans are rigid visual/mass candidates. All tensors reallocated from new geometry.')
  self.command_registry['door_contact']={'label':'门周边/端框局部原生接触（候选，默认关）','args':{'enabled':{'type':'boolean'}}}
  self.capabilities.update(cad_doors=True,native_spatial_latches=True,optional_partial_door_contact=True,independent_native_door_fans=1,photo_visible_door_fan_mounts=4)
  self.limits=[x for x in self.limits if '门、辅机安装及轮轨' not in x]+['CAD端框与四门真实迁移；铰深、间隙、安装和质量仍为研究候选。空间锁扣为理想约束；接触仅覆盖门周边和端框内边。','照片四门各1风机，当前仅1个原生转子；其余3个没有独立电机/转子动力学。']
  self._set_door_contact(self.door_contact_enabled)

 def augment_extra_xml(self,xml,metadata):
  xml=super().augment_extra_xml(xml,metadata)
  xml,self.door_spec,self.auxiliary_spec,self.cad_body_spec=augment_xml(xml,metadata,self.auxiliary_spec,self.cad_body_spec,self.door_parameters)
  return xml

 def visual_resource_bytes(self):return resource()[0]

 def _set_door_contact(self,enabled):
  if type(enabled) is not bool:raise ValueError('Door contact state must be boolean')
  for n in self.door_spec['contact_pairs']:self.model.pair_margin[self.model.pair(n).id]=0. if enabled else -1.
  self.door_contact_enabled=enabled

 def command(self,name,args):
  if name=='door_contact':
   if self.failed:raise ValueError('Plant stopped after rejected interval')
   if type(args) is not dict or set(args)!={'enabled'} or type(args['enabled']) is not bool:raise ValueError('door_contact requires exactly one boolean enabled')
   self._set_door_contact(args['enabled']);return
  return super().command(name,args)

 def closure_observation(self):
  # Detached endpoint evaluation: this is explicitly not the accepted midpoint
  # constraint sample and is never another applied force or energy column.
  m=self.model;s=mujoco.MjData(m);mujoco.mj_setState(m,s,self._mechanical_state(),STATE_KIND);mujoco.mj_forward(m,s);out={}
  for key,row in self.door_spec['latches'].items():
   a=m.site(row['leaf_site']).id;b=m.site(row['frame_site']).id;eq=m.equality(row['equality']).id
   rows=(s.efc_type==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))&(s.efc_id==eq);f=np.where(rows,s.efc_force,0.);q=np.zeros(m.nv);mujoco.mj_mulJacTVec(m,s,q,f);j=m.joint(self.door_spec['hinges'][key]['joint']).id;dof=int(m.jnt_dofadr[j]);wrench=s.efc_force[rows]
   out[key]=dict(leaf_point_world_m=s.site_xpos[a].tolist(),frame_point_world_m=s.site_xpos[b].tolist(),point_gap_m=float(np.linalg.norm(s.site_xpos[a]-s.site_xpos[b])),active=bool(s.eq_active[eq]),native_constraint_rows=int(sum(rows)),native_constraint_force_components=wrench.tolist(),hinge_generalized_reaction_Nm=float(q[dof]),sampling='DETACHED_ENDPOINT_MJ_FORWARD_NATIVE_CONSTRAINT; not accepted-step work',hardware_latch_certified=False)
  return out

 def snapshot(self):
  value=super().snapshot();value['variant']='cad_doors_mounted';value['source_geometry_sha256']=resource()[1]
  if hasattr(self,'door_spec') and hasattr(self,'doors'):
   value['telemetry']['cad_doors']={'latches':self.closure_observation(),'door_contact_enabled':self.door_contact_enabled,'contact_coverage':self.door_spec['contact_coverage'],'native_independent_door_fans':1,'photo_visible_door_fan_mounts':4,'mass_budget_kg':self.door_spec['mass_budget_kg']}
  return value

 def checkpoint(self):
  v=super().checkpoint();v['schema']='gtsd-cad-doors-assembly-checkpoint-v1';v['door_contact']=self.door_contact_enabled;v['sha256']=sha({k:x for k,x in v.items() if k!='sha256'});return v

 # Restore guards below derive from frozen Phase6, preserving every guard.
 def restore(self,value):
     expected={'schema','plant','support','auxiliary','door_contact','sha256'}
     if type(value) is not dict or set(value)!=expected or value['schema']!='gtsd-cad-doors-assembly-checkpoint-v1':raise ValueError('Phase6 checkpoint schema mismatch')
     canonical(value)
     if value['sha256']!=sha({k:v for k,v in value.items() if k!='sha256'}):raise ValueError('Phase6 checkpoint checksum mismatch')
     candidate=type(self)(self.dt,include_cables=True,brake_parameters=self.brake_parameters,support_parameters=self.support_parameters,auxiliary_parameters=self.auxiliary_parameters,door_parameters=self.door_parameters)
     candidate._set_door_contact(value['door_contact'])
     candidate.mount_runtime.restore(value['auxiliary'],validate_native=False);candidate._restore_validated_candidate(value['plant']);candidate.support.restore(value['support'])
     if candidate.steps!=candidate.mount_runtime.steps or candidate.steps!=candidate.support.steps:raise ValueError('Host/support/aux count mismatch')
     native=candidate.mount_runtime.observe()
     if native!=candidate.aux.mechanical_state['auxiliary']:raise ValueError('Native/auxiliary observed endpoint mismatch')
     if candidate.steps:
         last=candidate.mount_runtime.last
         if last['end']!=native:raise ValueError('Runtime last endpoint differs from true native state')
         start=candidate.last_native_brake
         q0=np.asarray(start['qpos_start']);v0=np.asarray(start['qvel_start']);r=candidate.mount_runtime
         observed0={'angle_rad':{n:float(q0[r.qadr[n]]) for n in ROTORS},'omega_rad_s':{n:float(v0[r.dof[n]]) for n in ROTORS},'piston_displacement_m':float(q0[r.qadr['compressor_piston']]),'piston_velocity_m_s':float(v0[r.dof['compressor_piston']])}
         if last['begin']!=observed0:raise ValueError('Aux beginning differs from native recorded start')
         if not np.allclose(last['generalized_displacement'],start['generalized_displacement'],atol=1e-13,rtol=0):raise ValueError('Aux/native interval displacement mismatch')
         scratch=mujoco.MjData(candidate.model);scratch.qpos[:]=q0;scratch.qvel[:]=v0;mujoco.mj_forward(candidate.model,scratch)
         old_data=r.data;r.data=scratch
         try:basis=r.basis()
         finally:r.data=old_data
         for n,b in basis.items():
             if not np.allclose(b['projection'],last['air_projection'][n],atol=1e-12,rtol=0) or not np.allclose(b['axis_world'],last['air_axis_world'][n],atol=1e-12,rtol=0):raise ValueError('Air world basis differs from native interval start')
         if abs(last['work_J']['compressor_gas']-candidate.aux.last_step['compressor_gas_work_J']-last['residual']['gas_J'])>1e-10:raise ValueError('Last auxiliary gas residual mismatch')
         for n in ROTORS:
             if abs(last['motor_torque_Nm'][n]-candidate.aux.last_step['auxiliary_motor_torque_Nm'][n])>candidate.acceptance['auxiliary_effort']:raise ValueError('Last native/electric motor torque mismatch')
             if abs(last['motor_work_J'][n]-candidate.aux.last_step['motor_shaft_work_J'][n]-last['motor_residual_J'][n])>1e-10:raise ValueError('Last native/electric motor work mismatch')
     for i,n in enumerate(AUX_COLUMNS):
         if abs(candidate.work[len(BASE_WORK_COLUMNS)+i]-candidate.mount_runtime.work[n])>1e-9:raise ValueError('Auxiliary work double-count/mismatch')
     motors={'compressor_rotor':candidate.aux.network.pump.motor,**{n+'_rotor':f.motor for n,f in candidate.aux.network.fans.items()}}
     if abs(sum(m.shaft_work_J for m in motors.values())-candidate.mount_runtime.work['auxiliary_motor'])>max(1,candidate.steps)*candidate.acceptance['auxiliary_work_J']:raise ValueError('Native/electrical shaft ledger mismatch')
     if abs(candidate.mount_runtime.work['compressor_gas']-candidate.aux.gas.node_useful_work_J['compressor_cylinder'])>max(1,candidate.steps)*candidate.acceptance['auxiliary_work_J']:raise ValueError('Native compressor/gas cumulative mismatch')
     bearing=candidate.aux.network.pump.friction_heat_J+sum(f.bearing_heat_J for f in candidate.aux.network.fans.values())
     air=sum(f.air_work_J for f in candidate.aux.network.fans.values())
     if abs(candidate.mount_runtime.work['auxiliary_bearing']+bearing)>max(1,candidate.steps)*1e-10 or abs(candidate.mount_runtime.work['fan_external_air']+air)>max(1,candidate.steps)*1e-10:raise ValueError('Aux native dissipation/gas-electric ledger mismatch')
     if abs(candidate.mount_runtime.work['yoke_spring']+candidate.mount_runtime.yoke_energy(native)-candidate.mount_runtime.initial_yoke_energy_J)>1e-8:raise ValueError('Yoke conservative cumulative work mismatch')
     original=np.asarray(value['plant']['mujoco_integration_state'],dtype=float);mujoco.mj_setState(candidate.model,candidate.data,original,STATE_KIND)
     if not np.array_equal(candidate._mechanical_state(),original):raise ValueError('Exact Phase6 native restore failed')
     self.__dict__.clear();self.__dict__.update(candidate.__dict__)
