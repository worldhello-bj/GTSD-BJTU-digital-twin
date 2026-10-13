"""Reproducible force-driven runs; JSON frames are actual MuJoCo xpos/xquat.
Native contact wrenches: mj_contactForce contact-frame output, rotated to world
and sign-corrected onto the wheel (geom1 receives negative contact force).
"""
from pathlib import Path
import argparse, csv, json, time, hashlib, shutil
import numpy as np
import mujoco
from model import ROOT,GEOMETRY,C,G,DEFAULT,make_model,reference_positions,write_parameters

def jid(m,n):return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
def bid(m,n):return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
def sid(m,n):return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,n)
def name(m,typ,i):return mujoco.mj_id2name(m,typ,i)
def poses(m,d):return {name(m,mujoco.mjtObj.mjOBJ_BODY,i):{'position':d.xpos[i].tolist(),'quaternion':d.xquat[i].tolist()} for i in range(1,m.nbody)}

def contacts(m,d):
    out={f'wheel_{i}_{s}':{'normal_N':0.,'tangential_N':0.,'world_force_on_wheel_N':[0.,0.,0.],'contact_count':0,'max_penetration_m':0.} for i in (1,2) for s in (-1,1)}
    for k in range(d.ncon):
        c=d.contact[k];a=name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom1);b=name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom2)
        wheel=a if a and a.startswith('wheel_') else b if b and b.startswith('wheel_') else None
        if wheel not in out:continue
        wrench=np.zeros(6);mujoco.mj_contactForce(m,d,k,wrench)
        world=c.frame.reshape(3,3).T@wrench[:3];world*=(-1 if a==wheel else 1)
        q=out[wheel];q['normal_N']+=float(wrench[0]);q['tangential_N']+=float(np.linalg.norm(wrench[1:3]));q['contact_count']+=1;q['max_penetration_m']=max(q['max_penetration_m'],max(0.,-float(c.dist)))
        q['world_force_on_wheel_N']=(np.array(q['world_force_on_wheel_N'])+world).tolist()
    return out

def telemetry(m,d,p,controls,external_force,work,diss):
    con=contacts(m,d);primary={}
    for i in (1,2):
      for s in (-1,1):
        tn=f'primary_{i}_{s}';idx=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_TENDON,tn)
        L=float(d.ten_length[idx]);v=float(d.ten_velocity[idx]);L0=float(m.tendon_lengthspring[idx,0])
        primary[tn]={'length_m':L,'length_change_from_geometry_m':L-.0835,'compression_from_unloaded_m':L0-L,'compression_force_N':p['primary_spring_stiffness_Npm']*(L0-L)-p['primary_spring_damping_Nspm']*v,'length_velocity_mps':v,'lower_world':d.site_xpos[sid(m,f'primary_lower_{i}_{s}')].tolist(),'upper_world':d.site_xpos[sid(m,f'primary_upper_{i}_{s}')].tolist()}
    sec=jid(m,'secondary_z');sq=float(d.qpos[m.jnt_qposadr[sec]]);sv=float(d.qvel[m.jnt_dofadr[sec]])
    wheels={}
    for i in (1,2):
        vel=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,bid(m,f'wheelset_{i}'),vel,0)
        j=jid(m,f'wheel_spin_{i}');sj=jid(m,f'swing_{i}')
        wheels[f'wheelset_{i}']={'axle_forward_velocity_mps':float(vel[3]),'world_angular_y_radps':float(vel[1]),'longitudinal_creep_velocity_mps':float(vel[3]-p['rolling_radius_m']*vel[1]),'spin_angle_rad':float(d.qpos[m.jnt_qposadr[j]]),'spin_velocity_radps':float(d.qvel[m.jnt_dofadr[j]]),'swing_angle_rad':float(d.qpos[m.jnt_qposadr[sj]]),'torque_Nm':float(controls[i-1])}
    return {'solver_time_s':float(d.time),'frame_position_m':d.xpos[bid(m,'frame')].tolist(),'frame_velocity_mps':d.qvel[:3].tolist(),'wheel_rail':con,'total_normal_N':sum(q['normal_N'] for q in con.values()),'total_contact_world_force_N':np.sum([q['world_force_on_wheel_N'] for q in con.values()],axis=0).tolist(),'primary_suspension':primary,'secondary_endpoints':{'lower_world':d.site_xpos[sid(m,'secondary_lower')].tolist(),'upper_world':d.site_xpos[sid(m,'secondary_upper')].tolist()},'secondary_deflection_m':sq,'secondary_velocity_mps':sv,'wheelsets':wheels,'external_frame_force_z_N':external_force,'potential_energy_J':float(d.energy[0]),'kinetic_energy_J':float(d.energy[1]),'mechanical_energy_J':float(sum(d.energy)),'cumulative_external_work_J':work,'cumulative_suspension_and_bearing_dissipation_J':diss,'warning_counts':[int(w.number) for w in d.warning],'contacts_count':int(d.ncon)}

def run(case,p,output):
    m=mujoco.MjModel.from_xml_string(make_model(p));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    if case!='gravity_settle':
        for _ in range(round(p['settle_s']/p['timestep_s'])):mujoco.mj_step(m,d)
        d.time=0.;mujoco.mj_forward(m,d)
    duration={'gravity_settle':3.,'vertical_disturbance':5.,'traction_brake':p['traction_observation_s']}[case]
    frames=[];work=0.;diss=0.;positive_motor_work=0.;negative_motor_work=0.;force_work=0.;dt=p['timestep_s'];sample=round(p['sample_interval_s']/dt);start=time.perf_counter();maxwarnings=np.zeros(mujoco.mjtWarning.mjNWARNING,dtype=int)
    e0=float(sum(d.energy));x0=float(d.xpos[bid(m,'frame'),0]);z0=float(d.xpos[bid(m,'frame'),2]);max_qvel=0.;max_swing=0.;max_pen=0.;max_primary=0.;end_qvel=0.;speed_peak=0.;step_count=round(duration/dt)
    for step in range(step_count+1):
        t=step*dt;torques=[0.,0.];force=0.
        if case=='vertical_disturbance' and p['vertical_pulse_start_s']<=t<p['vertical_pulse_end_s']:force=p['vertical_pulse_force_N']
        if case=='traction_brake':
            if .50<=t<2.50:torques=[p['traction_torque_each_axle_Nm']]*2
            elif 3.0<=t:
                torques=[float(np.clip(-p['brake_velocity_gain_Nmsprad']*d.qvel[m.jnt_dofadr[jid(m,f'wheel_spin_{i}')]],-p['brake_torque_limit_each_axle_Nm'],p['brake_torque_limit_each_axle_Nm'])) for i in (1,2)]
        d.ctrl[:]=torques;d.xfrc_applied[:]=0;d.xfrc_applied[bid(m,'frame'),2]=force
        mujoco.mj_forward(m,d)
        maxwarnings=np.maximum(maxwarnings,[w.number for w in d.warning]);max_qvel=max(max_qvel,float(np.max(np.abs(d.qvel))));speed_peak=max(speed_peak,abs(float(d.qvel[0])))
        max_swing=max(max_swing,*[abs(float(d.qpos[m.jnt_qposadr[jid(m,f'swing_{i}')]])) for i in (1,2)])
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Nonfinite state')
        if step%sample==0 or step==step_count:
            tel=telemetry(m,d,p,torques,force,work,diss);frames.append({'time_s':t,'body_poses':poses(m,d),'telemetry':tel})
            max_pen=max(max_pen,*[q['max_penetration_m'] for q in tel['wheel_rail'].values()]);max_primary=max(max_primary,*[abs(q['length_change_from_geometry_m']) for q in tel['primary_suspension'].values()])
        if step==step_count:break
        power=float(np.dot(d.actuator_force,d.actuator_velocity)+force*d.qvel[2]);loss=float(np.dot(m.dof_damping,d.qvel*d.qvel)+np.dot(m.tendon_damping,d.ten_velocity*d.ten_velocity))
        motor_power=float(np.dot(d.actuator_force,d.actuator_velocity));positive_motor_work+=max(0.,motor_power)*dt;negative_motor_work+=min(0.,motor_power)*dt;force_work+=force*d.qvel[2]*dt
        work+=power*dt;diss+=loss*dt;mujoco.mj_step(m,d)
    wall=time.perf_counter()-start;tail=[f['telemetry'] for f in frames if f['time_s']>=duration-.5]
    last=frames[-1]['telemetry'];total_mass=float(sum(m.body_mass));normal=sum(t['total_normal_N'] for t in tail)/len(tail)
    summary={'case_id':case,'engine':'MuJoCo','engine_version':mujoco.__version__,'duration_s':duration,'physics_steps':step_count,'wall_s':wall,'sample_count':len(frames),'dof':m.nv,'mass_kg':total_mass,'weight_N':total_mass*p['gravity_mps2'],'tail_mean_normal_N':normal,'tail_weight_relative_error':abs(normal-total_mass*p['gravity_mps2'])/(total_mass*p['gravity_mps2']),'final_frame_velocity_mps':last['frame_velocity_mps'],'final_max_abs_qvel':float(np.max(np.abs(d.qvel))),'peak_frame_forward_speed_mps':speed_peak,'travel_m':float(d.xpos[bid(m,'frame'),0])-x0,'peak_frame_vertical_departure_m':max(abs(f['body_poses']['frame']['position'][2]-z0) for f in frames),'tail_frame_vertical_range_m':float(np.ptp([t['frame_position_m'][2] for t in tail])),'peak_primary_length_change_m':max_primary,'peak_swing_angle_rad':max_swing,'max_sampled_penetration_m':max_pen,'initial_energy_J':e0,'final_energy_J':last['mechanical_energy_J'],'external_work_J':work,'positive_motor_work_J':positive_motor_work,'negative_motor_work_J':negative_motor_work,'vertical_force_work_J':force_work,'known_damping_dissipation_J':diss,'unmodeled_contact_and_integration_energy_residual_J':last['mechanical_energy_J']-e0-work+diss,'warning_counts':maxwarnings.tolist(),'finite_state':True,'all_four_wheels_loaded_at_end':all(v['normal_N']>0 for v in last['wheel_rail'].values()),'no_joint_limit_hit':max_swing<.179,'scope':'uncalibrated new-geometry planar mechanism; numeric demonstration, no test-rig validation'}
    summary['braking_speed_threshold_mps']=0.001 if case=='traction_brake' else None
    summary['first_braking_sample_below_threshold_s']=next((f['time_s'] for f in frames if f['time_s']>=3.0 and abs(f['telemetry']['frame_velocity_mps'][0])<.001),None) if case=='traction_brake' else None
    summary['numerical_acceptance_pass']=bool(not maxwarnings.any() and summary['all_four_wheels_loaded_at_end'] and summary['no_joint_limit_hit'] and summary['tail_weight_relative_error']<.02 and ((summary['peak_frame_forward_speed_mps']>.1 and abs(summary['final_frame_velocity_mps'][0])<.25*summary['peak_frame_forward_speed_mps']) if case=='traction_brake' else summary['final_max_abs_qvel']<.002))
    payload={'schema':'gtsd-physics-trajectory-v1','subsystem':'bogie','case_id':case,'source_geometry_sha256':hashlib.sha256((GEOMETRY/'web/geometry.json').read_bytes()).hexdigest(),'frames':frames,'parameters':p,'simulation_status':summary,'coordinate_system':'metres; new geometry world X along car, Y transverse, Z up; wxyz quaternion; no scaling'}
    folder=output/case;folder.mkdir(parents=True,exist_ok=True);(folder/'trajectory.json').write_text(json.dumps(payload,ensure_ascii=False,separators=(',',':')));(folder/'summary.json').write_text(json.dumps(summary,indent=2))
    with (folder/'telemetry.csv').open('w',newline='') as f:
        keys=['time_s','x_m','z_m','vx_mps','vz_mps','normal_N','kinetic_J','potential_J','external_work_J','damping_J'];writer=csv.writer(f);writer.writerow(keys)
        for row in frames:
            t=row['telemetry'];writer.writerow([row['time_s'],t['frame_position_m'][0],t['frame_position_m'][2],t['frame_velocity_mps'][0],t['frame_velocity_mps'][2],t['total_normal_N'],t['kinetic_energy_J'],t['potential_energy_J'],t['cumulative_external_work_J'],t['cumulative_suspension_and_bearing_dissipation_J']])
    np.savez_compressed(folder/'final_state.npz',qpos=d.qpos,qvel=d.qvel,time=d.time)
    print(json.dumps(summary,ensure_ascii=False));return summary

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['all','gravity_settle','vertical_disturbance','traction_brake'],default='all');parser.add_argument('--overrides',type=Path);parser.add_argument('--output',type=Path,default=ROOT/'results');args=parser.parse_args();p=DEFAULT|(json.loads(args.overrides.read_text()) if args.overrides else {})
    (ROOT/'model.xml').write_text(make_model(p));write_parameters(p)
    m=mujoco.MjModel.from_xml_string(make_model(p));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    refs={'schema':'gtsd-reference-poses-v1','reference_poses':poses(m,d),'joints':[{ 'name':name(m,mujoco.mjtObj.mjOBJ_JOINT,i),'world_anchor_m':d.xanchor[i].tolist(),'world_axis':d.xaxis[i].tolist()} for i in range(m.njnt)]}
    (ROOT/'body_reference_poses.json').write_text(json.dumps(refs,indent=2))
    cases=['gravity_settle','vertical_disturbance','traction_brake'] if args.case=='all' else [args.case];summaries=[run(case,p,args.output) for case in cases];args.output.mkdir(parents=True,exist_ok=True);(args.output/'summary.json').write_text(json.dumps(summaries,indent=2))
    if args.case=='all':shutil.copyfile(args.output/'vertical_disturbance/trajectory.json',ROOT/'trajectory.json')

if __name__=='__main__':main()
