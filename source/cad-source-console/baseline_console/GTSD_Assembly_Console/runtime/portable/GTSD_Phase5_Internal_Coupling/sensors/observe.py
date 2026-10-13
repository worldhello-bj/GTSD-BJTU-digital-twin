"""Read-only ideal SI observations. Never a hardware decoder or calibrated sensor.
No extrapolation, noise synthesis, raw-code encoding, or mechanical state writes.
"""
import hashlib,json,math
import mujoco
import numpy as np
SCHEMA='gtsd-ideal-solver-observations-v1'
CANDIDATES={'gas.reservoir.absolute_pressure':('DTB-C0047','name-based pressure transmitter candidate; port and absolute/gauge output unknown'),
 'joint.wheel_spin_1.velocity':('DTB-C0073','list-order candidate only; actual axle identity, sign and transfer function unknown'),
 'joint.wheel_spin_2.velocity':('DTB-C0074','list-order candidate only; actual axle identity, sign and transfer function unknown')}
def observe(model,data,telemetry,source_geometry_sha256,ambient_pressure_Pa=101325.):
 if not isinstance(telemetry,dict):raise ValueError('telemetry mapping required')
 if not isinstance(source_geometry_sha256,str)or len(source_geometry_sha256)!=64 or any(c not in '0123456789abcdef' for c in source_geometry_sha256):raise ValueError('source geometry SHA256 required')
 if type(ambient_pressure_Pa)not in (float,int)or not math.isfinite(ambient_pressure_Pa)or ambient_pressure_Pa<=0:raise ValueError('finite positive ambient pressure required')
 t=float(data.time);items=[]
 def add(id_,value,unit,source,meaning,body=None):
  if isinstance(value,np.generic):value=value.item()
  if type(value)not in (bool,int,float)or(type(value)is not bool and not math.isfinite(value)):raise ValueError('invalid observation '+id_)
  candidate=CANDIDATES.get(id_)
  items.append({'signal_id':id_,'value':value,'unit':unit,'sample_time_s':t,'quality':'IDEAL_SOLVER_NOT_HARDWARE_CALIBRATED','source':source,'definition':meaning,'body_name':body,'candidate_component_id':candidate[0]if candidate else None,'candidate_status':candidate[1]if candidate else 'NO_PHYSICAL_SENSOR_BINDING','hardware_address':None,'hardware_raw':None,'measurement_uncertainty':None})
 for j in range(model.njnt):
  name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j)
  if not name:continue
  jt=int(model.jnt_type[j]);body=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.jnt_bodyid[j]))
  if jt in (int(mujoco.mjtJoint.mjJNT_HINGE),int(mujoco.mjtJoint.mjJNT_SLIDE)):
   rot=jt==int(mujoco.mjtJoint.mjJNT_HINGE);pos_unit='rad'if rot else'm';vel_unit='rad/s'if rot else'm/s'
   add('joint.'+name+'.position',float(data.qpos[int(model.jnt_qposadr[j])]),pos_unit,'MuJoCo qpos['+name+']','relative joint displacement; not an encoder count',body)
   add('joint.'+name+'.velocity',float(data.qvel[int(model.jnt_dofadr[j])]),vel_unit,'MuJoCo qvel['+name+']','relative joint speed; not world axle speed or train speed',body)
 for bid in range(1,model.nbody):
  name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,bid)
  if not name:continue
  for k,axis in enumerate('xyz'):add('body.'+name+'.world_position_'+axis,float(data.xpos[bid,k]),'m','MuJoCo xpos['+name+']','world body-frame origin coordinate',name)
 for key,meaning in [('total_normal_N','sum of modeled wheel/rail normal magnitudes'),('panto_contact_normal_N','sum of modeled pantograph contact normal magnitudes'),('head_external_test_force_N','world vertical externally applied test input, not measured air-cylinder force')]:
  if key in telemetry:add('mechanical.'+key,telemetry[key],'N','telemetry.'+key,meaning)
 gas=telemetry.get('pneumatic_aux',telemetry.get('pneumatic',{}))
 if isinstance(gas,dict):
  for name,value in gas.get('pressure_Pa',gas.get('pressures_Pa',{})).items():
   add('gas.'+name+'.absolute_pressure',value,'Pa','gas EOS node '+name,'absolute pressure computed from gas mass, temperature and volume')
   add('gas.'+name+'.gauge_pressure',float(value)-ambient_pressure_Pa,'Pa','gas absolute pressure minus declared ambient','gauge pressure relative to '+str(ambient_pressure_Pa)+' Pa ambient assumption')
  for name,spool in gas.get('spools',{}).items():
   for field,unit in [('position_m','m'),('velocity_m_s','m/s'),('coil_failed','boolean')]:
    if field in spool:add('valve.'+name+'.'+field,spool[field],unit,'dynamic spool state '+name,'actual numerical spool state or imposed failure flag')
  for name,node in gas.get('thermal',{}).items():
   if 'temperature_K'in node:add('thermal.'+name+'.temperature',node['temperature_K'],'K','auxiliary lumped thermal state','unverified lumped node temperature; not a thermometer reading')
  for name,fan in gas.get('fans',{}).items():
   if 'omega_rad_s'in fan:add('fan.'+name+'.speed',fan['omega_rad_s'],'rad/s','force-driven fan state','auxiliary fixed-mount equivalent angular speed')
 for i,motor in enumerate(telemetry.get('motors',[]),1):
  if 'current_A'in motor:add('motor.'+str(i)+'.current',motor['current_A'],'A','RL circuit integrated current','equivalent axle motor port current; real motor count unverified')
 registry=[{k:v for k,v in x.items()if k not in ('value','sample_time_s')}for x in items]
 return {'schema':SCHEMA,'sample_time_s':t,'source_geometry_sha256':source_geometry_sha256,'registry_sha256':hashlib.sha256(json.dumps(registry,ensure_ascii=False,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'hardware_connected':False,'calibrated':False,'interpolation':False,'ambient_pressure_assumption_Pa':ambient_pressure_Pa,'observations':items}
