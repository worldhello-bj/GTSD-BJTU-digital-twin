"""Native auxiliary force ports. No rotor or piston time integrator exists here.

Body mass, gyroscopic terms and all mounting reactions are owned by MuJoCo.
The three motor conversions are internal relative-coordinate ports. World-air
fan torques alone are external; their full Jacobian projection includes host
motion. Mount sensors are read-only and never reapplied as forces.
"""
from __future__ import annotations
import copy, math
import mujoco
import numpy as np
from . import bootstrap
from pneumatic_aux.kernels import b10

ROTORS=('compressor_rotor','compressor_fan_rotor','bay_fan_rotor')
FAN_KEYS=('compressor_fan','bay_fan')
JOINTS=(*ROTORS,'compressor_piston')
COLUMNS=('auxiliary_motor','auxiliary_bearing','compressor_gas','yoke_spring','yoke_damping','fan_external_air')
STATE_OWNER='MUJOCO_QPOS_QVEL_ONLY; endpoint copies are observations, never integrated'

def sinc(x):return math.sin(x)/x if x else 1.

class AuxiliaryMountRuntime:
    def __init__(self,model,data,spec=None,*,yoke_stiffness_N_m=1e5,yoke_damping_N_s_m=20.):
        self.model,self.data,self.spec=model,data,copy.deepcopy(spec or {})
        self.ky=float(yoke_stiffness_N_m);self.cy=float(yoke_damping_N_s_m)
        if not (math.isfinite(self.ky) and self.ky>0 and math.isfinite(self.cy) and self.cy>=0):raise ValueError('Invalid yoke parameters')
        self.qadr={n:int(model.jnt_qposadr[model.joint(n).id]) for n in JOINTS}
        self.dof={n:int(model.jnt_dofadr[model.joint(n).id]) for n in JOINTS}
        self.bid={n:model.body(n).id for n in ROTORS}
        self.p=b10.PumpParameters();self.fp={n:b10.FanParameters() for n in FAN_KEYS}
        self.steps=0;self.work={n:0. for n in COLUMNS};self.last={}
        self.max_residual={'gas_J':0.,'motor_J':0.,'yoke_J':0.,'bearing_J':0.,'air_J':0.,'positive_air_J':0.}
        self.mounts={n:site for n,site in (('compressor','compressor_mount_site'),('compressor_fan','compressor_fan_mount_site'),('bay_fan','bay_fan_mount_site')) if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,site)>=0}
        self.initial_yoke_energy_J=self.yoke_energy(self.observe())
        self.last_sensor={}

    def observe(self):
        d=self.data
        return {'angle_rad':{n:float(d.qpos[self.qadr[n]]) for n in ROTORS},
                'omega_rad_s':{n:float(d.qvel[self.dof[n]]) for n in ROTORS},
                'piston_displacement_m':float(d.qpos[self.qadr['compressor_piston']]),
                'piston_velocity_m_s':float(d.qvel[self.dof['compressor_piston']])}

    def yoke_error(self,state):
        a=state['angle_rad']['compressor_rotor']
        return state['piston_displacement_m']-self.p.crank_radius_m*(1-math.cos(a))

    def yoke_energy(self,state):return .5*self.ky*self.yoke_error(state)**2

    def basis(self):
        """Freeze force application geometry at interval start, not endpoint."""
        m,d=self.model,self.data;result={}
        for key in FAN_KEYS:
            n=key+'_rotor';joint=m.joint(n).id
            # MuJoCo xaxis is the actual world joint axis including mount pose.
            axis=np.asarray(d.xaxis[joint]).copy()
            jacp=np.zeros((3,m.nv));jacr=np.zeros_like(jacp)
            mujoco.mj_jacBody(m,d,jacp,jacr,self.bid[n])
            result[key]={'axis_world':axis,'angular_jacobian_world':jacr,'projection':axis@jacr}
        return result

    def air_speeds(self,basis,delta,h):return {n:float(basis[n]['projection']@delta/h) for n in FAN_KEYS}

    def preview(self,begin,end,delta,h,basis,motor_torques,piston_gas_force):
        """Conservative/discrete-passive efforts for one fixed-point trial."""
        nv=self.model.nv;parts={n:np.zeros(nv) for n in COLUMNS}
        rel={n:(end['angle_rad'][n]-begin['angle_rad'][n])/h for n in ROTORS}
        for n in ROTORS:parts['auxiliary_motor'][self.dof[n]]=motor_torques[n]
        parts['auxiliary_bearing'][self.dof['compressor_rotor']]=-self.p.viscous_friction_Nm_s*rel['compressor_rotor']
        for n in FAN_KEYS:parts['auxiliary_bearing'][self.dof[n+'_rotor']]=-self.fp[n].bearing_drag_Nm_s*rel[n+'_rotor']
        parts['compressor_gas'][self.dof['compressor_piston']]=piston_gas_force
        a,b=begin['angle_rad']['compressor_rotor'],end['angle_rad']['compressor_rotor']
        da=b-a;df=self.p.crank_radius_m*math.sin(.5*(a+b))*sinc(.5*da)
        e0,e1=self.yoke_error(begin),self.yoke_error(end)
        f_s=.5*self.ky*(e0+e1);f_d=self.cy*(e1-e0)/h
        for key,force in (('yoke_spring',f_s),('yoke_damping',f_d)):
            parts[key][self.dof['compressor_piston']]=-force
            parts[key][self.dof['compressor_rotor']]=force*df
        air=self.air_speeds(basis,delta,h);torque_air={}
        for n in FAN_KEYS:
            torque_air[n]=-self.fp[n].aerodynamic_drag_Nm_s2*abs(air[n])*air[n]
            parts['fan_external_air']+=torque_air[n]*basis[n]['projection']
        return {'parts':parts,'total':sum(parts.values()),'relative_speed_rad_s':rel,'air_speed_rad_s':air,
                'air_torque_Nm':torque_air,'motor_torque_Nm':dict(motor_torques),'piston_gas_force_N':float(piston_gas_force),
                'yoke_spring_force_N':f_s,'yoke_damping_force_N':f_d,'yoke_dx_dtheta_m_rad':df,
                'yoke_error_begin_m':e0,'yoke_error_end_m':e1}

    def capture_sensors(self,*,evaluation_time_s=None):
        m,d=self.model,self.data;out={}
        for name,site in self.mounts.items():
            sid=m.site(site).id;R=d.site_xmat[sid].reshape(3,3)
            def read(suffix):
                sensor=m.sensor(site+'_'+suffix).id;a=int(m.sensor_adr[sensor]);return d.sensordata[a:a+3].copy()
            f,t=read('force'),read('torque')
            out[name]={'force_local_N':f.tolist(),'torque_local_Nm':t.tolist(),
                'force_world_N':(R@f).tolist(),'torque_world_Nm':(R@t).tolist(),
                'site_position_world_m':d.site_xpos[sid].tolist(),
                'force_on_parent_world_N':(-R@f).tolist(),'torque_on_parent_world_Nm':(-R@t).tolist(),
                'site_frame_world':R.tolist(),'solver_interval_end_s':float(d.time),
                'force_evaluation_start_s':float(d.time-self.model.opt.timestep) if evaluation_time_s is None else float(evaluation_time_s),
                'sign':'Raw native support wrench uses MuJoCo site convention; force_on_parent/torque_on_parent are its explicit negative, verified by gravity and known loads',
                'source':'NATIVE_SOLVER_VIRTUAL_SENSOR_NOT_HARDWARE'}
        return out

    def commit(self,begin,end,delta,h,basis,used,auxiliary_step,sensors):
        work={n:float(q@delta) for n,q in used['parts'].items()}
        gas=float(auxiliary_step['compressor_gas_work_J'])
        motor={n:float(used['motor_torque_Nm'][n]*(end['angle_rad'][n]-begin['angle_rad'][n])) for n in ROTORS}
        motor_res={n:motor[n]-auxiliary_step['motor_shaft_work_J'][n] for n in ROTORS}
        yoke=work['yoke_spring']+self.yoke_energy(end)-self.yoke_energy(begin)
        residual={'gas_J':work['compressor_gas']-gas,'motor_J':max(abs(v) for v in motor_res.values()),'yoke_J':yoke,'bearing_J':work['auxiliary_bearing']+auxiliary_step['bearing_heat_J'],'air_J':work['fan_external_air']+auxiliary_step['fan_air_work_J'],'positive_air_J':max(0.,work['fan_external_air'])}
        if max(abs(residual[n]) for n in ('gas_J','motor_J','yoke_J','bearing_J','air_J'))>1e-9 or residual['positive_air_J']>1e-12:
            raise ArithmeticError('Auxiliary port work mismatch: '+str(residual))
        for n in COLUMNS:self.work[n]+=work[n]
        for n,v in residual.items():self.max_residual[n]=max(self.max_residual[n],abs(v))
        self.steps+=1;self.last_sensor=copy.deepcopy(sensors)
        self.last={'work_J':work,'residual':residual,'motor_work_J':motor,'motor_residual_J':motor_res,
            'generalized_force':used['total'].tolist(),'component_generalized_forces':{n:q.tolist() for n,q in used['parts'].items()},
            'generalized_displacement':delta.tolist(),'begin':copy.deepcopy(begin),'end':copy.deepcopy(end),
            'yoke_energy_J':self.yoke_energy(end),'yoke_error_m':self.yoke_error(end),
            'yoke_damping_heat_J':-work['yoke_damping'],'bearing_heat_J':-work['auxiliary_bearing'],
            'air_work_to_environment_J':-work['fan_external_air'],'air_speed_rad_s':dict(used['air_speed_rad_s']),
            'accepted_air_speed_rad_s':self.air_speeds(basis,delta,h),
            'air_torque_Nm':dict(used['air_torque_Nm']),'motor_torque_Nm':dict(used['motor_torque_Nm']),
            'air_axis_world':{n:basis[n]['axis_world'].tolist() for n in FAN_KEYS},
            'air_projection':{n:basis[n]['projection'].tolist() for n in FAN_KEYS},'dt_s':h}

    def checkpoint(self):
        return copy.deepcopy({'steps':self.steps,'work':self.work,'last':self.last,'max_residual':self.max_residual,'last_sensor':self.last_sensor})

    def restore(self,value,*,validate_native=True):
        import json
        json.dumps(value,allow_nan=False)
        if type(value) is not dict or set(value)!=set(self.checkpoint()):raise ValueError('Aux runtime checkpoint fields mismatch')
        if type(value['steps']) is not int or value['steps']<0 or set(value['work'])!=set(COLUMNS):raise ValueError('Aux runtime count/work mismatch')
        finite=lambda x:type(x) in (int,float) and math.isfinite(x)
        for n,v in value['work'].items():
            if not finite(v):raise ValueError('Nonfinite auxiliary work')
        for n in ('auxiliary_bearing','yoke_damping','fan_external_air'):
            if value['work'][n]>1e-10:raise ValueError('Nonpassive auxiliary work')
        if set(value['max_residual'])!=set(self.max_residual) or any(not finite(x) or x<0 or x>1e-8 for x in value['max_residual'].values()):raise ValueError('Invalid maximum auxiliary residual')
        if value['steps']==0:
            if value['last'] or value['last_sensor'] or any(value['work'].values()):raise ValueError('Nonzero zero-step auxiliary state')
        else:
            r=value['last'];required={'work_J','residual','motor_work_J','motor_residual_J','generalized_force','component_generalized_forces','generalized_displacement','begin','end','yoke_energy_J','yoke_error_m','yoke_damping_heat_J','bearing_heat_J','air_work_to_environment_J','air_speed_rad_s','accepted_air_speed_rad_s','air_torque_Nm','motor_torque_Nm','air_axis_world','air_projection','dt_s'}
            if type(r) is not dict or set(r)!=required:raise ValueError('Aux last-step schema mismatch')
            if not finite(r['dt_s']) or abs(r['dt_s']-self.model.opt.timestep)>1e-15:raise ValueError('Aux dt mismatch')
            dq=np.asarray(r['generalized_displacement']);total=np.asarray(r['generalized_force'])
            if dq.shape!=(self.model.nv,) or total.shape!=(self.model.nv,) or not np.isfinite(np.r_[dq,total]).all():raise ValueError('Aux vector shape/finite mismatch')
            if set(r['component_generalized_forces'])!=set(COLUMNS) or set(r['work_J'])!=set(COLUMNS):raise ValueError('Aux component schema mismatch')
            parts={n:np.asarray(r['component_generalized_forces'][n],dtype=float) for n in COLUMNS}
            if any(q.shape!=dq.shape or not np.isfinite(q).all() for q in parts.values()):raise ValueError('Aux component dimension mismatch')
            if not np.allclose(total,sum(parts.values()),atol=1e-12,rtol=0):raise ValueError('Aux total force is not component sum')
            if any(not finite(r['work_J'][n]) or abs(parts[n]@dq-r['work_J'][n])>1e-10 for n in COLUMNS):raise ValueError('Aux work projection mismatch')
            for state in (r['begin'],r['end']):
                if type(state) is not dict or set(state)!=set(self.observe()):raise ValueError('Native observation schema mismatch')
                for key in ('angle_rad','omega_rad_s'):
                    if type(state[key]) is not dict or set(state[key])!=set(ROTORS) or any(not finite(v) for v in state[key].values()):raise ValueError('Native observed rotor mismatch')
                if any(not finite(state[k]) for k in ('piston_displacement_m','piston_velocity_m_s')):raise ValueError('Invalid observed piston')
            if validate_native and r['end']!=self.observe():raise ValueError('Aux end observation differs from native state')
            for n in ROTORS:
                if abs(r['end']['angle_rad'][n]-r['begin']['angle_rad'][n]-dq[self.dof[n]])>1e-12:raise ValueError('Native rotor displacement mismatch')
            if abs(r['end']['piston_displacement_m']-r['begin']['piston_displacement_m']-dq[self.dof['compressor_piston']])>1e-12:raise ValueError('Native piston displacement mismatch')
            basis={}
            for n in FAN_KEYS:
                axis=np.asarray(r['air_axis_world'][n]);projection=np.asarray(r['air_projection'][n])
                if axis.shape!=(3,) or projection.shape!=dq.shape or abs(axis@axis-1)>1e-10:raise ValueError('Air reference geometry mismatch')
                basis[n]={'axis_world':axis,'projection':projection}
                speed=float(projection@dq/r['dt_s'])
                used_speed=r['air_speed_rad_s'][n]
                if abs(speed-r['accepted_air_speed_rad_s'][n])>1e-10 or abs(r['air_torque_Nm'][n]+self.fp[n].aerodynamic_drag_Nm_s2*abs(used_speed)*used_speed)>1e-12 or abs(r['air_torque_Nm'][n]+self.fp[n].aerodynamic_drag_Nm_s2*abs(speed)*speed)>2e-9:raise ValueError('Air reference/torque mismatch')
            expected=self.preview(r['begin'],r['end'],dq,r['dt_s'],basis,r['motor_torque_Nm'],parts['compressor_gas'][self.dof['compressor_piston']])
            if any(not np.allclose(parts[n],expected['parts'][n],atol=2e-9,rtol=0) for n in COLUMNS):raise ValueError('Aux component force law mismatch')
            for n in ROTORS:
                if abs(r['motor_work_J'][n]-r['motor_torque_Nm'][n]*dq[self.dof[n]])>1e-10:raise ValueError('Aux motor work mismatch')
            if abs(sum(r['motor_work_J'].values())-r['work_J']['auxiliary_motor'])>1e-10:raise ValueError('Aux motor work sum mismatch')
            if abs(r['yoke_energy_J']-self.yoke_energy(r['end']))>1e-12 or abs(r['yoke_error_m']-self.yoke_error(r['end']))>1e-12:raise ValueError('Yoke observed geometry mismatch')
            if abs(r['work_J']['yoke_spring']+self.yoke_energy(r['end'])-self.yoke_energy(r['begin']))>1e-9:raise ValueError('Yoke gradient work mismatch')
            for field,column in [('yoke_damping_heat_J','yoke_damping'),('bearing_heat_J','auxiliary_bearing'),('air_work_to_environment_J','fan_external_air')]:
                if not finite(r[field]) or r[field]<-1e-12 or abs(r[field]+r['work_J'][column])>1e-10:raise ValueError('Aux dissipation/work mismatch')
            if set(r['residual'])!=set(self.max_residual) or any(not finite(x) or abs(x)>1e-8 for x in r['residual'].values()):raise ValueError('Invalid aux last residual')
            bounds={'gas_J':1e-10,'motor_J':1e-10,'yoke_J':1e-9,'bearing_J':1e-9,'air_J':1e-9,'positive_air_J':1e-12}
            if set(r['motor_residual_J'])!=set(ROTORS) or any(not finite(x) or abs(x)>1e-10 for x in r['motor_residual_J'].values()):raise ValueError('Invalid per-motor port residual')
            if abs(r['residual']['motor_J']-max(abs(x) for x in r['motor_residual_J'].values()))>1e-15:raise ValueError('Motor residual maximum mismatch')
            for key,bound in bounds.items():
                if abs(r['residual'][key])>bound or value['max_residual'][key]+1e-15<abs(r['residual'][key]) or value['max_residual'][key]>bound:raise ValueError('Auxiliary accepted residual/history mismatch')
            if r['residual']['motor_J']<0 or r['residual']['positive_air_J']<0:raise ValueError('Negative nonnegative auxiliary residual')
            if set(value['last_sensor'])!=set(self.mounts):raise ValueError('Mount sensor schema mismatch')
            for n,record in value['last_sensor'].items():
                R=np.asarray(record['site_frame_world']);f=np.asarray(record['force_local_N']);t=np.asarray(record['torque_local_Nm'])
                if R.shape!=(3,3) or f.shape!=(3,) or t.shape!=(3,) or not np.allclose(R.T@R,np.eye(3),atol=1e-10,rtol=0):raise ValueError('Mount sensor frame mismatch')
                for field,expected_value in [('force_world_N',R@f),('torque_world_Nm',R@t),('force_on_parent_world_N',-R@f),('torque_on_parent_world_Nm',-R@t)]:
                    if not np.allclose(record[field],expected_value,atol=1e-11,rtol=0):raise ValueError('Mount sensor transformation mismatch')
        for n,v in value.items():setattr(self,n,copy.deepcopy(v))

    def snapshot(self):
        return self.checkpoint()|{'state_owner':STATE_OWNER,'native':self.observe(),'yoke_energy_J':self.yoke_energy(self.observe()),
            'yoke_error_m':self.yoke_error(self.observe()),'air_reference':'WORLD_REST_AIR; full rotor angular Jacobian; fluid reaction external',
            'mass_boundary':'Native rotor/piston/stator masses split from existing budgets; no extra rotor armature',
            'work_boundary':'Mount wrenches are internal transfer diagnostics, never an additional global work column',
            'limitations':['Inferred finite-flexibility Scotch-yoke, not reconstructed compressor hardware','B9 valve spools remain fixed-bench external mechanisms','Fluid linear inertia and jet thrust excluded']}
