"""Actual MuJoCo integration with pinned RL/gas co-simulation, no pose driving."""
from collections import defaultdict
from dataclasses import asdict
import argparse, json, shutil, time
import numpy as np
import mujoco
from integrated_model import ROOT, build_model, metadata, poses, helpers, panto, coupling, FROZEN_GEOMETRY_SHA, DEFAULT_DT

CASES={'gravity_head_load':8.0,'traction_air_brake_head_contact':10.0}
WHEELS=[f'wheel_{i}_{s}' for i in (1,2) for s in (-1,1)]

def ramp(t,start,end):
    x=min(1.,max(0.,(t-start)/(end-start)))
    return .5-.5*np.cos(np.pi*x)

def command(case,t):
    # Smooth, explicit experiment commands, never coordinate targets.
    if case=='gravity_head_load':
        force=-20.*ramp(t,3.,4.)*(1.-ramp(t,6.,7.))
        return dict(traction_enable=False,brake_supply=False,brake_exhaust=False,head_force_N=force)
    force=30.*ramp(t,3.,4.)*(1.-ramp(t,8.5,9.5))
    return dict(traction_enable=4.0<=t<6.0,brake_supply=6.1<=t<8.1,brake_exhaust=t>=8.1,head_force_N=force)

def apply_head(m,d,f):
    if not np.isfinite(f) or abs(f)>panto.DEFAULTS['head_load_limit_N']:
        raise ValueError('External head test force must be finite and within declared +/-100 N port envelope')
    d.xfrc_applied[:]=0
    body=m.body('panto_head_float').id;site=m.site('head_load_point').id
    force=np.array([0.,0.,f]);point=d.site_xpos[site]
    d.xfrc_applied[body,:3]=force
    d.xfrc_applied[body,3:]=np.cross(point-d.xipos[body],force)
    qf=np.zeros(m.nv);mujoco.mj_applyFT(m,d,force,np.zeros(3),point,body,qf)
    return qf

def run(case,dt,out,baseline_window=(2.5,2.95),loaded_window=None):
    m,xml,bp,pp,mp,g=build_model(dt);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    brake=coupling.FixedVolumeBrake(coupling.BrakeParameters(gas_substep_s=min(.0005,dt)))
    motors=[coupling.Motor(mp),coupling.Motor(mp)]
    ids=np.array([m.jnt_dofadr[m.joint(f'wheel_spin_{i}').id] for i in (1,2)])
    base=m.body('panto_base').id;frame=m.body('frame').id
    head=m.body('panto_head_float').id;site=m.site('head_load_point').id
    nv=m.nv;steps=round(CASES[case]/dt);stride=round(.02/dt)
    state_kind=mujoco.mjtState.mjSTATE_INTEGRATION;state=np.zeros(mujoco.mj_stateSize(m,state_kind))
    trace=defaultdict(list);frames=[];start=time.perf_counter();energy0=float(sum(d.energy));work=np.zeros(5)
    # Columns: motor mechanical; brake signed; external head; damping signed;
    # contact+limit constraint signed. The residual is separately reported.
    last_step_work=np.zeros(5);last_motor=[v.preview(dt,0.,False) for v in motors]
    last_brake=[brake.force_torque(i,0.) for i in (1,2)];last_f=0.;last_qext=np.zeros(nv)
    max_power_identity_error=0.;max_motor_electrical_residual=0.;max_coupling_residual=0.
    min_brake_power=0.;warnings=np.zeros(len(d.warning),int)
    last_corrector_error=np.zeros(2);last_constraint_generalized=np.zeros(nv);last_limit_generalized=np.zeros(nv);last_mid_velocity=np.zeros(nv);constraint_work_parts=np.zeros(2)
    for step in range(steps+1):
        t=step*dt;mujoco.mj_forward(m,d);mujoco.mj_rnePostConstraint(m,d);mujoco.mj_subtreeVel(m,d)
        wc=helpers.contacts(m,d);cn,cf,cr=panto.contacts(m,d)
        rail=np.sum([wc[k]['world_force_on_wheel_N'] for k in WHEELS],axis=0)
        warnings=np.maximum(warnings,[w.number for w in d.warning])
        if not np.isfinite(np.r_[d.qpos,d.qvel,d.qacc]).all():raise ArithmeticError('Nonfinite mechanics')
        ga=brake.network.audit();sa=brake.network.spool_audit();cmd=command(case,t)
        # World momentum of all moving bodies for independent force-balance checks.
        momentum=float(m.body_subtreemass[frame])*d.subtree_linvel[frame]
        jac=np.zeros((3,nv));rot=np.zeros((3,nv));mujoco.mj_jacSite(m,d,jac,rot,site)
        force_power=last_f*float((jac@d.qvel)[2]);projected=float(np.dot(last_qext,d.qvel))
        max_power_identity_error=max(max_power_identity_error,abs(force_power-projected))
        dense={
          'time_s':t,'qpos':d.qpos.copy(),'qvel':d.qvel.copy(),'qacc':d.qacc.copy(),
          'body_position_m':d.xpos.copy(),'body_quaternion_wxyz':d.xquat.copy(),
          'body_com_position_m':d.xipos.copy(),'ctrl':d.ctrl.copy(),
          'qfrc_passive':d.qfrc_passive.copy(),'qfrc_constraint':d.qfrc_constraint.copy(),
          'qfrc_actuator':d.qfrc_actuator.copy(),'head_generalized_force':last_qext.copy(),
          'head_force_world_N':np.array([0.,0.,last_f]),'head_load_point_world_m':d.site_xpos[site].copy(),
          'head_load_point_velocity_world_m_s':jac@d.qvel,
          'wheel_normal_N':np.array([wc[k]['normal_N'] for k in WHEELS]),
          'wheel_contact_world_force_N':np.array([wc[k]['world_force_on_wheel_N'] for k in WHEELS]),
          'collector_contact_normal_N':cn,'collector_contact_world_force_N':cf.copy(),
          'panto_base_parent_force_on_subtree_world_N':d.cfrc_int[base,3:].copy(),
          'panto_reaction_on_payload_world_N':-d.cfrc_int[base,3:].copy(),
          'panto_base_cfrc_int_torque_force':d.cfrc_int[base].copy(),
          'total_linear_momentum_kg_m_s':momentum.copy(),
          'external_total_world_force_N':rail+cf+np.array([0,0,last_f-sum(m.body_mass)*9.81]),
          'energy_potential_kinetic_J':d.energy.copy(),
          'work_columns_cumulative_J':work.copy(),'previous_step_work_J':last_step_work.copy(),
          'previous_step_constraint_generalized_force':last_constraint_generalized.copy(),
          'previous_step_limit_generalized_force':last_limit_generalized.copy(),
          'previous_step_midpoint_velocity':last_mid_velocity.copy(),
          'previous_step_corrector_speed_error_rad_s':last_corrector_error.copy(),
          'constraint_limit_and_other_work_J':constraint_work_parts.copy(),
          'energy_residual_after_all_reported_work_J':float(sum(d.energy))-energy0-sum(work),
          'commands_next_step':np.array([cmd[k] for k in cmd]),
          'pressure_absolute_Pa':np.array(list(brake.network.pressures_Pa().values())),
          'gas_mass_kg':np.array([v.mass_kg for v in brake.network.volumes.values()]),
          'gas_temperature_K':np.array([brake.network.temperature_K for v in brake.network.volumes.values()]),
          'spool_position_m':np.array([v.position_m for v in brake.network.spools.values()]),
          'spool_velocity_m_s':np.array([v.velocity_m_s for v in brake.network.spools.values()]),
          'spool_opening_fraction':np.array([v.opening for v in brake.network.spools.values()]),
          'last_port_flow_kg_s':np.array(list(brake.network.last_flows_kg_s.values())),
          'motor_current_A':np.array([v.current_A for v in motors]),
          'last_motor_torque_Nm':np.array([v['torque_Nm'] for v in last_motor]),
          'last_brake_torque_Nm':np.array([v['torque_Nm'] for v in last_brake]),
          'last_motor_mean_current_A':np.array([v['current_mean_A'] for v in last_motor]),
          'last_motor_back_emf_V':np.array([v['emf_V'] for v in last_motor]),
          'gas_audit':np.array(list(ga.values())),'spool_audit':np.array(list(sa.values())),
          'motor_audit':np.array([list(v.audit().values()) for v in motors]),
          'warnings':warnings.copy(),'solver_iterations':int(d.solver_niter[0]),
          'primary_lengths_m':d.ten_length.copy(),'primary_velocities_m_s':d.ten_velocity.copy(),
        }
        for k,v in dense.items():trace[k].append(v)
        if step%stride==0 or step==steps:
            tel=helpers.telemetry(m,d,bp,d.ctrl[:2],0.,work[0]+work[1]+work[2],-work[3])
            tel.update({'panto_contact_normal_N':cn,'panto_contact_world_force_N':cf.tolist(),'head_external_test_force_N':last_f,
              'panto_reaction_on_payload_world_N':(-d.cfrc_int[base,3:]).tolist(),
              'pneumatic':brake.snapshot(),'motors':[v.audit() for v in motors],
              'command_next_step':cmd,'energy_work_columns_J':work.tolist(),
              'energy_residual_after_reported_work_J':dense['energy_residual_after_all_reported_work_J'],
              'signal_status':'IDEAL_SOLVER_SI_OBSERVATIONS_NOT_CALIBRATED_HARDWARE_SENSOR','hardware_raw':None})
            frames.append({'time_s':t,'body_poses':poses(m,d),'telemetry':tel})
        if step==steps:break
        # Gas is advanced ONCE. Mechanical predictor trials restore the same
        # initial integrated state; this is rollback, not kinematic prescription.
        old_p=[brake.network.pressure_Pa(f'brake_axle_{i}') for i in (1,2)]
        brake.advance(dt,cmd['brake_supply'],cmd['brake_exhaust'])
        mid_p=[.5*(old_p[i-1]+brake.network.pressure_Pa(f'brake_axle_{i}')) for i in (1,2)]
        v0=d.qvel.copy();ten0=d.ten_velocity.copy();omega0=v0[ids].copy();guess=omega0.copy()
        mujoco.mj_getState(m,d,state,state_kind)
        for iteration in range(4):
            evaluated_speed=guess.copy()
            ms=[motors[i].preview(dt,float(guess[i]),cmd['traction_enable']) for i in range(2)]
            bs=[brake.force_torque(i+1,float(guess[i]),mid_p[i]) for i in range(2)]
            mujoco.mj_setState(m,d,state,state_kind);mujoco.mj_forward(m,d)
            d.ctrl[:]=0;d.ctrl[:2]=[ms[i]['torque_Nm']+bs[i]['torque_Nm'] for i in range(2)]
            qext0=apply_head(m,d,cmd['head_force_N']);mujoco.mj_forward(m,d)
            con0=d.qfrc_constraint.copy()
            mujoco.mj_step(m,d)
            actual_constraint=d.qfrc_constraint.copy()
            actual_limit=np.zeros(nv)
            is_limit=np.isin(d.efc_type,[int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT),int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)])
            mujoco.mj_mulJacTVec(m,d,actual_limit,np.where(is_limit,d.efc_force,0.))
            observed=.5*(omega0+d.qvel[ids])
            if np.max(abs(observed-guess))<1e-10:break
            guess=observed
        v1=d.qvel.copy();vmid=.5*(v0+v1);wmid=vmid[ids]
        last_corrector_error=wmid-evaluated_speed
        try:
            coupling.validate_discrete_coupling_step(dt,evaluated_speed,wmid,[v['torque_Nm'] for v in bs])
        except coupling.CouplingStepRejected as error:
            mujoco.mj_setState(m,d,state,state_kind)
            failure=out/case;failure.mkdir(parents=True,exist_ok=True)
            (failure/'failure.json').write_text(json.dumps({'accepted':False,'case':case,'time_s':t,'dt_s':dt,'diagnostics':error.diagnostics,'note':'Run aborted. Existing success files, if present, are from an earlier run and are not current-run outputs; advanced trial gas state is discarded with this run.'},indent=2))
            raise
        last_constraint_generalized=actual_constraint.copy();last_limit_generalized=actual_limit.copy();last_mid_velocity=vmid.copy()
        constraint_work_parts+=dt*np.array([float(np.dot(actual_limit,vmid)),float(np.dot(actual_constraint-actual_limit,vmid))])
        mujoco.mj_forward(m,d)
        # Force direction is global Z. Point follows solved body pose.
        last_qext=apply_head(m,d,cmd['head_force_N']);mujoco.mj_forward(m,d)
        con1=d.qfrc_constraint.copy();ten1=d.ten_velocity.copy()
        damping=-dt*float(np.dot(m.dof_damping,vmid*vmid)+np.dot(m.tendon_damping,(.5*(ten0+ten1))**2))
        head_work=dt*float(np.dot(qext0,vmid))
        constraint_work=dt*float(np.dot(actual_constraint,vmid))
        motor_work=sum(ms[i]['torque_Nm']*wmid[i]*dt for i in range(2))
        brake_work=sum(bs[i]['torque_Nm']*wmid[i]*dt for i in range(2))
        for i in range(2):
            mw=ms[i]['torque_Nm']*float(wmid[i])*dt;motors[i].commit(ms[i],mw)
            max_motor_electrical_residual=max(max_motor_electrical_residual,abs(ms[i]['electrical_balance_residual_J']))
            max_coupling_residual=max(max_coupling_residual,abs(mw-ms[i]['electromagnetic_conversion_work_J']))
            min_brake_power=min(min_brake_power,-bs[i]['torque_Nm']*float(wmid[i]))
        last_step_work=np.array([motor_work,brake_work,head_work,damping,constraint_work]);work+=last_step_work
        brake.brake_heat_J=-work[1];last_motor=ms;last_brake=bs;last_f=cmd['head_force_N']
    data={k:np.asarray(v) for k,v in trace.items()};tt=data['time_s']
    baseline=(tt>=baseline_window[0])&(tt<=baseline_window[1]);loaded_window=loaded_window or ((5.5,5.95) if case=='gravity_head_load' else (7.5,8.0));loaded=(tt>=loaded_window[0])&(tt<=loaded_window[1])
    resid=data['energy_residual_after_all_reported_work_J']
    summary={'case_id':case,'engine':'MuJoCo 3.3.7 + frozen SI RL motor + fixed-volume B8/B9 gas and spool equivalents',
      'scope':'Uncalibrated new-geometry mechanically integrated engineering equivalent, not a field-calibrated full vehicle',
      'dt_s':dt,'duration_s':CASES[case],'steps':steps,'dense_rows':len(tt),'frames':len(frames),'wall_s':time.perf_counter()-start,
      'total_dynamic_mass_kg':float(sum(m.body_mass)),'baseline_wheel_normal_N':data['wheel_normal_N'][baseline].mean(axis=0).tolist(),
      'loaded_wheel_normal_N':data['wheel_normal_N'][loaded].mean(axis=0).tolist(),
      'baseline_panto_reaction_on_payload_N':data['panto_reaction_on_payload_world_N'][baseline].mean(axis=0).tolist(),
      'loaded_panto_reaction_on_payload_N':data['panto_reaction_on_payload_world_N'][loaded].mean(axis=0).tolist(),
      'baseline_weight_balance_relative_error':abs(data['wheel_normal_N'][baseline].sum(axis=1).mean()-sum(m.body_mass)*9.81)/(sum(m.body_mass)*9.81),
      'peak_wire_contact_N':float(data['collector_contact_normal_N'].max()),'min_wheel_normal_after_settle_N':float(data['wheel_normal_N'][tt>.8].min()),
      'peak_forward_speed_m_s':float(data['qvel'][:,0].max()),'final_forward_speed_m_s':float(d.qvel[0]),'travel_m':float(d.qpos[0]),
      'peak_brake_gauge_pressure_Pa':float(data['pressure_absolute_Pa'][:,1:].max()-coupling.AMBIENT_PA),
      'final_brake_gauge_pressure_Pa':data['pressure_absolute_Pa'][-1,1:].tolist(),
      'final_energy_change_J':float(sum(d.energy))-energy0,'work_columns_J':work.tolist(),'final_energy_residual_J':float(resid[-1]),'max_abs_energy_residual_J':float(abs(resid).max()),
      'energy_note':'Signed contact/limit work is explicitly quadrature-integrated. Remaining energy residual includes quadrature and integration error; contact penalty potential is not separately available. No whole-system energy-closure or calibrated efficiency claim.',
      'max_head_Fv_generalized_power_identity_error_W':max_power_identity_error,
      'max_electrical_step_balance_residual_J':max_motor_electrical_residual,'max_electromechanical_split_step_residual_J':max_coupling_residual,
      'min_brake_dissipated_power_W':min_brake_power,'gas_audit':ga,'spool_audit':sa,'motor_audits':[x.audit() for x in motors],
      'max_gas_mass_residual_kg':float(abs(data['gas_audit'][:,list(ga).index('mass_balance_residual_kg')]).max()),
      'max_gas_energy_residual_J':float(abs(data['gas_audit'][:,list(ga).index('gas_energy_balance_residual_J')]).max()),
      'max_spool_energy_residual_J':float(abs(data['spool_audit'][:,list(sa).index('balance_residual_J')]).max()),
      'warning_counts':warnings.tolist(),'finite':True,
      'max_accepted_corrector_speed_error_rad_s':float(abs(data['previous_step_corrector_speed_error_rad_s']).max()),
      'constraint_joint_limit_work_J':float(constraint_work_parts[0]),'constraint_other_work_J':float(constraint_work_parts[1]),
      'discrete_acceptance':'Each accepted interval has corrector midpoint speed error <=1e-7 rad/s and per-axle brake work <=1e-12 J. Otherwise the run fails without returning an accepted trajectory.',
      'active_limit_note':('Unmeasured soft panto_head_pitch upper limit participates.' if case=='gravity_head_load' else 'Unmeasured soft panto_head_slide lower limit participates.')+' Constraint work is explicitly separated into joint/tendon limits and other constraints. These are numerical stops, not measured real travel limits.'}
    active_limits=[]
    for j in range(m.njnt):
        if not m.jnt_limited[j]:continue
        values=data['qpos'][:,m.jnt_qposadr[j]];lo,hi=m.jnt_range[j]
        below=np.maximum(0.,lo-values);above=np.maximum(0.,values-hi)
        if max(below.max(),above.max())>1e-10:
            active_limits.append({'joint':m.joint(j).name,'unit':'m' if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'rad','range':[float(lo),float(hi)],'min':float(values.min()),'max':float(values.max()),'max_below_lower':float(below.max()),'max_above_upper':float(above.max()),'steps_beyond_limit':int(np.sum((below>1e-10)|(above>1e-10)))})
    summary['active_joint_limits']=active_limits
    summary['active_limit_note']='Unmeasured numerical joint stops participate as listed in active_joint_limits; limit work is explicitly separated. These are not measured real travel limits.'
    summary['final_brake_gauge_pressure_Pa']=(data['pressure_absolute_Pa'][-1,1:]-coupling.AMBIENT_PA).tolist()
    folder=out/case;folder.mkdir(parents=True,exist_ok=True)
    schema={'schema':'gtsd-integrated-every-step-v1','rows':len(tt),'dt_s':dt,
      'time_convention':'Endpoint states include previous accepted interval loads and work; commands_next_step applies after that endpoint. No preload settling is discarded. Mechanical predictor retries restore the same initial state; only accepted endpoints are stored.',
      'body_names':[m.body(i).name for i in range(m.nbody)],'wheel_columns':WHEELS,
      'joint_layout':[{'name':m.joint(i).name,'qpos_start':int(m.jnt_qposadr[i]),'qvel_start':int(m.jnt_dofadr[i]),'type':int(m.jnt_type[i])} for i in range(m.njnt)],
      'work_columns':['motor_mechanical','brake_signed','head_external','viscous_signed','constraint_signed'],
      'constraint_work_part_columns':['joint_and_tendon_limits_signed','other_constraints_signed'],
      'gas_node_columns':list(brake.network.volumes),'gas_audit_columns':list(ga),'spool_audit_columns':list(sa),'motor_audit_columns':list(motors[0].audit()),
      'spool_columns':list(brake.network.spools),'port_columns':list(brake.network.orifices),'command_columns':list(cmd),
      'unit_convention':'SI: translation m and m/s; rotation rad and rad/s; world forces N; moments N m; pressures absolute Pa unless explicitly gauge; power W; energy J; quaternions wxyz.',
      'reaction_convention':'mj_rnePostConstraint cfrc_int on panto_base is parent-on-subtree wrench, world-aligned at subtree COM; force negated yields reaction on payload. Moment remains native c-frame origin; not silently moved to mount.',
      'source_geometry_sha256':FROZEN_GEOMETRY_SHA,'version':'3.3.7','hardware_raw':None}
    np.savez_compressed(folder/'every_step.npz',**data)
    (folder/'model.xml').write_text(xml)
    (folder/'trace_schema.json').write_text(json.dumps(schema,indent=2))
    (folder/'summary.json').write_text(json.dumps(summary,indent=2))
    (folder/'trajectory.json').write_text(json.dumps({'schema':'gtsd-physics-trajectory-v1','subsystem':'integrated','case_id':case,
      'source_geometry_sha256':FROZEN_GEOMETRY_SHA,'binding_reference':'../../binding.json','frames':frames,'simulation_status':summary,
      'scope':summary['scope'],'units':'SI metres rad seconds, quaternion wxyz'},separators=(',',':')))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('motor_audits','gas_audit','spool_audit')}),flush=True)
    return summary

def main():
    p=argparse.ArgumentParser();p.add_argument('--case',choices=['all',*CASES],default='all');p.add_argument('--dt',type=float,default=DEFAULT_DT);p.add_argument('--out',type=str,default=str(ROOT/'results'));a=p.parse_args()
    out=ROOT/a.out if not a.out.startswith('/') else __import__('pathlib').Path(a.out)
    if out==ROOT/'results':metadata(*build_model(a.dt))
    sums=[run(case,a.dt,out) for case in (CASES if a.case=='all' else [a.case])]
    (out/'summary.json').write_text(json.dumps(sums,indent=2))
    if out==ROOT/'results' and a.case=='all':shutil.copyfile(out/'traction_air_brake_head_contact/trajectory.json',ROOT/'trajectory.json')
if __name__=='__main__':main()
