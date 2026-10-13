"""Three independent quasi-static electric ports; native motion is sole owner."""
import copy,math
from dataclasses import dataclass,asdict
import mujoco,numpy as np
from aux_mount.runtime import AuxiliaryMountRuntime
from pneumatic_aux.kernels import b10
from .model import KEYS,NAMES
COLUMNS=('extra_door_fan_motor','extra_door_fan_bearing','extra_door_fan_air')
@dataclass(frozen=True)
class Parameters:
 voltage_V:float=24.
 resistance_ohm:float=24.
 torque_back_emf_constant:float=.03
 bearing_drag_Nm_s:float=1e-6
 aerodynamic_drag_Nm_s2:float=1e-7
 def __post_init__(self):
  if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in asdict(self).values()):raise ValueError('Positive finite four-fan engineering parameters required')

class ExtraFans(AuxiliaryMountRuntime):
 def __init__(self,model,data,parameters):
  self.model,self.data,self.parameters=model,data,parameters
  self.qadr={k:int(model.jnt_qposadr[model.joint(NAMES[k]+'_rotor').id]) for k in KEYS};self.dof={k:int(model.jnt_dofadr[model.joint(NAMES[k]+'_rotor').id]) for k in KEYS}
  self.bid={k:model.body(NAMES[k]+'_rotor').id for k in KEYS};self.mounts={k:NAMES[k]+'_mount_site' for k in KEYS}
  self.motors={k:b10.Motor(b10.MotorParameters(voltage_V=parameters.voltage_V,resistance_ohm=parameters.resistance_ohm,torque_back_emf_constant=parameters.torque_back_emf_constant)) for k in KEYS}
  self.controls={k:{'voltage_fraction':1.,'power_failed':False} for k in KEYS};self.steps=0;self.work={k:0. for k in COLUMNS};self.last={};self.last_sensor={}
 def observe(self):return {k:{'angle_rad':float(self.data.qpos[self.qadr[k]]),'omega_rad_s':float(self.data.qvel[self.dof[k]])} for k in KEYS}
 def basis(self):
  out={};m,d=self.model,self.data
  for k in KEYS:
   axis=d.xaxis[m.joint(NAMES[k]+'_rotor').id].copy();jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(m,d,jp,jr,self.bid[k]);out[k]={'axis_world':axis,'projection':axis@jr}
  return out
 def preview(self,delta,h,basis):
  p=self.parameters;parts={n:np.zeros(self.model.nv) for n in COLUMNS};motor={};rel={};air={};torque={}
  for k in KEYS:
   w=delta[self.dof[k]]/h;a=float(basis[k]['projection']@delta/h);c=self.controls[k];tau=0. if c['power_failed'] else p.torque_back_emf_constant*(p.voltage_V*c['voltage_fraction']-p.torque_back_emf_constant*w)/p.resistance_ohm
   drag=-p.aerodynamic_drag_Nm_s2*abs(a)*a
   parts[COLUMNS[0]][self.dof[k]]=tau;parts[COLUMNS[1]][self.dof[k]]=-p.bearing_drag_Nm_s*w;parts[COLUMNS[2]]+=drag*basis[k]['projection'];motor[k]=tau;rel[k]=w;air[k]=a;torque[k]=drag
  return {'parts':parts,'total':sum(parts.values()),'motor_torque_Nm':motor,'relative_speed_rad_s':rel,'air_speed_rad_s':air,'air_torque_Nm':torque}
 def commit(self,begin,delta,h,basis,used,sensors):
  work={n:float(v@delta) for n,v in used['parts'].items()};motorwork={};res={}
  for k,m in self.motors.items():
   c=self.controls[k];m.power_failed=c['power_failed'];old=m.shaft_work_J;m.record(h,c['voltage_fraction'],float(delta[self.dof[k]]/h));motorwork[k]=m.shaft_work_J-old;res[k]=used['motor_torque_Nm'][k]*delta[self.dof[k]]-motorwork[k]
  if max(abs(x) for x in res.values())>1e-10 or work[COLUMNS[1]]>1e-12 or work[COLUMNS[2]]>1e-12:raise ArithmeticError('Extra fan passive/work port mismatch')
  for n in COLUMNS:self.work[n]+=work[n]
  self.steps+=1;self.last_sensor=copy.deepcopy(sensors)
  self.last={'begin':copy.deepcopy(begin),'end':self.observe(),'dt_s':h,'controls':copy.deepcopy(self.controls),'generalized_displacement':delta.tolist(),'generalized_force':used['total'].tolist(),'component_generalized_forces':{n:q.tolist() for n,q in used['parts'].items()},'work_J':work,'motor_work_J':motorwork,'motor_residual_J':res,'motor_torque_Nm':used['motor_torque_Nm'],'air_speed_rad_s':used['air_speed_rad_s'],'relative_speed_rad_s':used['relative_speed_rad_s'],'air_torque_Nm':used['air_torque_Nm'],'air_axis_world':{k:basis[k]['axis_world'].tolist() for k in KEYS},'air_projection':{k:basis[k]['projection'].tolist() for k in KEYS}}
 def checkpoint(self):return copy.deepcopy({'steps':self.steps,'controls':self.controls,'work':self.work,'last':self.last,'last_sensor':self.last_sensor,'motors':{k:{n:getattr(m,n) for n in ('electrical_work_J','copper_heat_J','shaft_work_J','current_A','power_failed')} for k,m in self.motors.items()}})
 def restore(self,value,validate_native=False):
  import json
  json.dumps(value,allow_nan=False)
  finite=lambda x:type(x) in (int,float) and math.isfinite(x)
  if type(value) is not dict or set(value)!=set(self.checkpoint()) or type(value['steps']) is not int or value['steps']<0:raise ValueError('Extra fan checkpoint schema/count')
  if set(value['controls'])!=set(KEYS) or set(value['motors'])!=set(KEYS) or set(value['work'])!=set(COLUMNS):raise ValueError('Extra fan key mismatch')
  for k,c in value['controls'].items():
   if type(c) is not dict or set(c)!={'voltage_fraction','power_failed'} or not finite(c['voltage_fraction']) or not 0<=c['voltage_fraction']<=1 or type(c['power_failed']) is not bool:raise ValueError('Extra fan control invalid')
  if any(not finite(x) for x in value['work'].values()) or any(value['work'][n]>1e-10 for n in COLUMNS[1:]):raise ValueError('Extra fan nonpassive work')
  for k,row in value['motors'].items():
   if set(row)!=set(self.checkpoint()['motors'][k]) or type(row['power_failed']) is not bool or any(not finite(v) for n,v in row.items() if n!='power_failed') or row['copper_heat_J']<0 or abs(row['electrical_work_J']-row['copper_heat_J']-row['shaft_work_J'])>1e-8:raise ValueError('Extra motor electrical ledger')
  if abs(sum(x['shaft_work_J'] for x in value['motors'].values())-value['work'][COLUMNS[0]])>max(1,value['steps'])*1e-10:raise ValueError('Extra motor/native cumulative ledger')
  if not value['steps']:
   if value['last'] or value['last_sensor'] or any(value['work'].values()) or any(any(v for v in row.values()) for row in value['motors'].values()):raise ValueError('Extra fan nonzero initial history')
  else:
   r=value['last']
   required={'begin','end','dt_s','controls','generalized_displacement','generalized_force','component_generalized_forces','work_J','motor_work_J','motor_residual_J','motor_torque_Nm','air_speed_rad_s','relative_speed_rad_s','air_torque_Nm','air_axis_world','air_projection'}
   if type(r) is not dict or set(r)!=required or set(r['controls'])!=set(KEYS) or set(r['begin'])!=set(KEYS) or set(r['end'])!=set(KEYS):raise ValueError('Extra last step schema')
   for k in KEYS:
    c=r['controls'][k]
    if type(c) is not dict or set(c)!={'voltage_fraction','power_failed'} or type(c['power_failed']) is not bool or not finite(c['voltage_fraction']) or not 0<=c['voltage_fraction']<=1:raise ValueError('Extra accepted control invalid')
    for state in (r['begin'][k],r['end'][k]):
     if set(state)!={'angle_rad','omega_rad_s'} or any(not finite(v) for v in state.values()):raise ValueError('Extra native observation schema')
   dq=np.asarray(r['generalized_displacement']);parts={n:np.asarray(r['component_generalized_forces'][n]) for n in COLUMNS}
   if dq.shape!=(self.model.nv,) or not np.isfinite(dq).all() or any(q.shape!=dq.shape or not np.isfinite(q).all() for q in parts.values()):raise ValueError('Extra vector shape')
   if abs(r['dt_s']-self.model.opt.timestep)>1e-15 or not np.allclose(sum(parts.values()),r['generalized_force'],atol=1e-12,rtol=0):raise ValueError('Extra step/time/force sum')
   basis={k:{'axis_world':np.asarray(r['air_axis_world'][k]),'projection':np.asarray(r['air_projection'][k])} for k in KEYS}
   old=self.controls;self.controls=r['controls']
   try:expected=self.preview(dq,r['dt_s'],basis)
   finally:self.controls=old
   for n in COLUMNS:
    if not np.allclose(parts[n],expected['parts'][n],atol=2e-9,rtol=0) or abs(parts[n]@dq-r['work_J'][n])>1e-10:raise ValueError('Extra force law/work mismatch')
   for k in KEYS:
    if basis[k]['axis_world'].shape!=(3,) or abs(basis[k]['axis_world']@basis[k]['axis_world']-1)>1e-10 or basis[k]['projection'].shape!=dq.shape:raise ValueError('Extra air basis shape')
    if abs(r['motor_torque_Nm'][k]-expected['motor_torque_Nm'][k])>1e-9 or abs(r['air_torque_Nm'][k]-expected['air_torque_Nm'][k])>1e-9:raise ValueError('Extra torque law metadata')
    if abs(r['end'][k]['angle_rad']-r['begin'][k]['angle_rad']-dq[self.dof[k]])>1e-12 or abs(r['motor_torque_Nm'][k]*dq[self.dof[k]]-r['motor_work_J'][k]-r['motor_residual_J'][k])>1e-10 or abs(r['motor_residual_J'][k])>1e-10:raise ValueError('Extra displacement/motor work')
    row=value['motors'][k];c=r['controls'][k];p=self.parameters;current=0. if c['power_failed'] else (p.voltage_V*c['voltage_fraction']-p.torque_back_emf_constant*dq[self.dof[k]]/r['dt_s'])/p.resistance_ohm
    if abs(current-row['current_A'])>1e-12 or row['power_failed']!=c['power_failed']:raise ValueError('Extra observed electric endpoint')
   if set(value['last_sensor'])!=set(KEYS):raise ValueError('Extra sensor set')
   for row in value['last_sensor'].values():
    R=np.asarray(row['site_frame_world']);f=np.asarray(row['force_local_N']);t=np.asarray(row['torque_local_Nm'])
    if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3),atol=1e-10,rtol=0):raise ValueError('Extra sensor rotation')
    for n,v in [('force_world_N',R@f),('torque_world_Nm',R@t),('force_on_parent_world_N',-R@f),('torque_on_parent_world_Nm',-R@t)]:
     if not np.allclose(row[n],v,atol=1e-11,rtol=0):raise ValueError('Extra sensor sign/frame')
  for n in ('steps','controls','work','last','last_sensor'):setattr(self,n,copy.deepcopy(value[n]))
  for k,row in value['motors'].items():
   for n,v in row.items():setattr(self.motors[k],n,v)
 def snapshot(self):return self.checkpoint()|{'native':self.observe(),'state_owner':'MUJOCO_ONLY; electric current is algebraic, inductance neglected','air_reference':'WORLD_REST_AIR_FULL_ANGULAR_JACOBIAN','mount_wrenches_reapplied':False,'extra_thermal_boundary':'Copper/bearing heat exported as diagnostic calorimeters, no feedback into gas/cabinet temperature; thermal network for these three not calibrated'}
