"""Coupled v5 independent drive/brake cases; no modification of bogie sources."""
from pathlib import Path
import argparse, csv, hashlib, importlib.util, json, shutil, sys, time
from dataclasses import asdict
from xml.etree import ElementTree as ET
import numpy as np
import mujoco
from coupling import ROOT, Motor, DriveParameters, BrakeParameters, FixedVolumeBrake, command_at, VENDOR_HASHES

BOGIE=ROOT.parent/'bogie'
sys.path.insert(0,str(BOGIE))
import model as bogie_model
spec=importlib.util.spec_from_file_location('v5_bogie_helpers',BOGIE/'run_suite.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)


def coupled_model(p,mp):
    xml=ET.fromstring(bogie_model.make_model(p))
    for i in (1,2):
        j=xml.find(f".//joint[@name='wheel_spin_{i}']")
        j.set('armature',str(mp.rotor_inertia_kgm2*mp.gear_ratio_motor_per_axle**2))
        j.set('damping',str(p['wheel_bearing_damping_Nmsprad']+mp.rotor_viscous_friction_Nms_rad*mp.gear_ratio_motor_per_axle**2))
    xml.set('model','GTSD_v5_finite_volume_brake_RL_drive_equivalent')
    return ET.tostring(xml,encoding='unicode')


def source_hashes():
    paths=[BOGIE/'model.py',BOGIE/'run_suite.py',BOGIE/'binding.json',ROOT.parent/'web/geometry.json',ROOT.parent/'assembly_contract.json']
    return {str(x.relative_to(ROOT.parent)):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths}


def observation(m,d,brake):
    # Every SI value below is a solver observation; there is no ADC/encoder raw.
    out={'reservoir_pressure_absolute_Pa':brake.network.pressure_Pa('reservoir'),
         'reservoir_pressure_gauge_Pa':brake.network.pressure_Pa('reservoir')-brake.network.ambient_pressure_Pa,
         'frame_forward_speed_m_s':float(d.qvel[0])}
    for i in (1,2):
        j=helpers.jid(m,f'wheel_spin_{i}')
        out[f'axle_{i}_relative_spin_angle_rad']=float(d.qpos[m.jnt_qposadr[j]])
        out[f'axle_{i}_relative_spin_speed_rad_s']=float(d.qvel[m.jnt_dofadr[j]])
        out[f'axle_{i}_brake_pressure_absolute_Pa']=brake.network.pressure_Pa(f'brake_axle_{i}')
        out[f'axle_{i}_brake_pressure_gauge_Pa']=brake.network.pressure_Pa(f'brake_axle_{i}')-brake.network.ambient_pressure_Pa
    return {'quality':'IDEAL_SOLVER_OBSERVATION_NOT_CALIBRATED_SENSOR','values':out,
            'hardware_raw':None,'hardware_encoding':'unknown; no fabricated raw values',
            'wheel_speed_caution':'rad/s relative wheel joint is distinct from world angular speed, axle linear speed, and unidentified course-point units'}


def run(case,p,output,mp=None,bp=None):
    mp=mp or DriveParameters();bp=bp or BrakeParameters(gas_substep_s=min(.0005,p['timestep_s']))
    xml=coupled_model(p,mp);m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    for _ in range(round(p['settle_s']/p['timestep_s'])):mujoco.mj_step(m,d)
    d.time=0.;mujoco.mj_forward(m,d)
    motors=[Motor(mp),Motor(mp)];brake=FixedVolumeBrake(bp)
    ids=[int(m.jnt_dofadr[helpers.jid(m,f'wheel_spin_{i}')]) for i in (1,2)]
    duration=7.0;dt=p['timestep_s'];steps=round(duration/dt);sample=max(1,round(p['sample_interval_s']/dt))
    state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
    state=np.zeros(mujoco.mj_stateSize(m,state_spec))
    dense=[];frames=[];initial_E=float(sum(d.energy));initial_x=float(d.qpos[0]);damping_J=0.;brake_J=0.;motor_work_J=0.;constraint_work_J=0.
    max_elec_residual=0.;max_mass_residual=0.;max_gas_energy_residual=0.;max_spool_residual=0.;max_coupling_step=0.;min_brake_power=0.;max_warnings=np.zeros(len(d.warning),dtype=int)
    max_abs_swing=0.;min_contact_normal=float('inf');last_motor=[x.preview(dt,0.,False) for x in motors];last_brake=[brake.force_torque(i,0.) for i in (1,2)]
    start=time.perf_counter();previous_vx=float(d.qvel[0]);accel=0.
    for step in range(steps+1):
        t=step*dt;cmd=command_at(case,t)
        mujoco.mj_forward(m,d)
        max_warnings=np.maximum(max_warnings,[w.number for w in d.warning])
        max_abs_swing=max(max_abs_swing,*[abs(float(d.qpos[m.jnt_qposadr[helpers.jid(m,f'swing_{i}')]])) for i in (1,2)])
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise ArithmeticError('Nonfinite MuJoCo state')
        gas_audit=brake.network.audit();spool_audit=brake.network.spool_audit()
        max_mass_residual=max(max_mass_residual,abs(gas_audit['mass_balance_residual_kg']))
        max_gas_energy_residual=max(max_gas_energy_residual,abs(gas_audit['gas_energy_balance_residual_J']))
        max_spool_residual=max(max_spool_residual,abs(spool_audit['balance_residual_J']))
        dense.append({
          'time_s':t,'qpos':d.qpos.copy(),'qvel':d.qvel.copy(),
          'body_position_m':d.xpos[1:].copy(),'body_quaternion_wxyz':d.xquat[1:].copy(),
          'commands_next_step':np.array(list(cmd.values()),dtype=np.uint8),
          'last_step_applied_axle_torque_Nm':d.ctrl.copy(),
          'pressure_absolute_Pa':np.array(list(brake.network.pressures_Pa().values())),
          'gas_mass_kg':np.array([v.mass_kg for v in brake.network.volumes.values()]),
          'gas_volume_m3':np.array([v.volume_m3 for v in brake.network.volumes.values()]),
          'last_step_port_flow_kg_s':np.array(list(brake.network.last_flows_kg_s.values())),
          'spool_position_m':np.array([v.position_m for v in brake.network.spools.values()]),
          'spool_velocity_m_s':np.array([v.velocity_m_s for v in brake.network.spools.values()]),
          'spool_opening_fraction':np.array([v.opening for v in brake.network.spools.values()]),
          'motor_current_A':np.array([v.current_A for v in motors]),
          'last_step_motor_torque_Nm':np.array([v['torque_Nm'] for v in last_motor]),
          'last_step_brake_torque_Nm':np.array([v['torque_Nm'] for v in last_brake]),
          'last_step_back_emf_V':np.array([v['emf_V'] for v in last_motor]),
          'last_step_supply_power_W':np.array([v['electrical_supply_work_J']/dt for v in last_motor]),
          'last_step_copper_loss_W':np.array([v['copper_loss_J']/dt for v in last_motor]),
          'last_step_diode_loss_W':np.array([v['freewheel_diode_loss_J']/dt for v in last_motor]),
          'motor_magnetic_energy_J':np.array([v.audit()['magnetic_energy_J'] for v in motors]),
          'potential_and_kinetic_energy_J':d.energy.copy(),
          'motor_work_brake_heat_damping_heat_J':np.array([motor_work_J,brake_J,damping_J]),
          'constraint_work_J':constraint_work_J,
          'constraint_generalized_force':d.qfrc_constraint.copy(),
          'gas_audit_values':np.array(list(gas_audit.values())),
          'spool_audit_values':np.array(list(spool_audit.values())),
          'motor_audit_values':np.array([list(v.audit().values()) for v in motors]),
        })
        if step%sample==0 or step==steps:
            tel=helpers.telemetry(m,d,p,d.ctrl.tolist(),0.,motor_work_J-brake_J,damping_J)
            min_contact_normal=min(min_contact_normal,*[v['normal_N'] for v in tel['wheel_rail'].values()])
            tel.update({'commands':cmd,'frame_acceleration_m_s2':accel,'pneumatic':brake.snapshot(),
              'motors':{f'axle_{i+1}':dict(last_motor[i],audit=motors[i].audit(),
                  rotor_speed_rad_s=mp.gear_ratio_motor_per_axle*float(d.qvel[ids[i]]),
                  reflected_rotor_kinetic_energy_J=.5*mp.rotor_inertia_kgm2*(mp.gear_ratio_motor_per_axle*float(d.qvel[ids[i]]))**2) for i in range(2)},
              'brakes':{f'axle_{i+1}':last_brake[i] for i in range(2)},
              'software_signals':observation(m,d,brake),
              'energy_boundary':{'motor_mechanical_work_J':motor_work_J,'friction_brake_heat_J':brake_J,
                'suspension_bearing_rotor_damping_heat_J':damping_J,
                'constraint_work_J':constraint_work_J,
                'mechanical_after_constraint_work_residual_J':float(sum(d.energy))-initial_E-motor_work_J+brake_J+damping_J-constraint_work_J,
                'mechanical_contact_and_integration_residual_J':float(sum(d.energy))-initial_E-motor_work_J+brake_J+damping_J}})
            frames.append({'time_s':t,'body_poses':helpers.poses(m,d),'telemetry':tel})
        if step==steps:break
        # Midpoint pressure from one finite-volume advancement, never keyframes.
        old_pressure=[brake.network.pressure_Pa(f'brake_axle_{i}') for i in (1,2)]
        brake.advance(dt,cmd['brake_supply'],cmd['brake_exhaust'])
        mid_pressure=[.5*(old_pressure[i-1]+brake.network.pressure_Pa(f'brake_axle_{i}')) for i in (1,2)]
        v0=d.qvel.copy();tv0=d.ten_velocity.copy();omega0=v0[ids].copy();guess=omega0.copy()
        mujoco.mj_getState(m,d,state,state_spec)
        # Predictor/corrector co-simulation: same initial mechanical state each
        # trial. Spools/gas are advanced only once; no fictitious repeated flow.
        for iteration in range(4):
            ms=[motors[i].preview(dt,float(guess[i]),cmd['traction_enable']) for i in range(2)]
            bs=[brake.force_torque(i+1,float(guess[i]),mid_pressure[i]) for i in range(2)]
            mujoco.mj_setState(m,d,state,state_spec)
            d.ctrl[:]=[ms[i]['torque_Nm']+bs[i]['torque_Nm'] for i in range(2)]
            mujoco.mj_step(m,d)
            observed=.5*(omega0+d.qvel[ids])
            if np.max(np.abs(observed-guess))<1e-10:break
            guess=observed
        wmid=.5*(omega0+d.qvel[ids]);v1=d.qvel.copy();constraint_force=d.qfrc_constraint.copy();mujoco.mj_forward(m,d);tv1=d.ten_velocity.copy()
        constraint_work_J+=float(np.dot(constraint_force,.5*(v0+v1)))*dt
        damping_J+=float(np.dot(m.dof_damping,(.5*(v0+v1))**2)+np.dot(m.tendon_damping,(.5*(tv0+tv1))**2))*dt
        for i in range(2):
            motor_work=ms[i]['torque_Nm']*float(wmid[i])*dt
            absorbed=-bs[i]['torque_Nm']*float(wmid[i])*dt
            motors[i].commit(ms[i],motor_work);motor_work_J+=motor_work;brake_J+=absorbed
            max_elec_residual=max(max_elec_residual,abs(ms[i]['electrical_balance_residual_J']))
            max_coupling_step=max(max_coupling_step,abs(motor_work-ms[i]['electromagnetic_conversion_work_J']))
            min_brake_power=min(min_brake_power,absorbed/dt)
        brake.brake_heat_J=brake_J;last_motor=ms;last_brake=bs
        accel=(float(d.qvel[0])-previous_vx)/dt;previous_vx=float(d.qvel[0])
    end=frames[-1]['telemetry'];gas=brake.network.audit();ma=[motor.audit() for motor in motors]
    sample_at=lambda time_:min(frames,key=lambda f:abs(f['time_s']-time_))['telemetry']
    brake_start=sample_at(2.6);brake_end=sample_at(4.6)
    summary={'case_id':case,'engine':'MuJoCo + explicit midpoint RL motor + finite-volume B8/B9 gas/spool',
      'engine_version':mujoco.__version__,'duration_s':duration,'physics_steps':steps,'sample_count':len(frames),
      'dense_trace_samples':len(dense),'wall_s':time.perf_counter()-start,'finite_state':True,'numerical_acceptance_pass':False,
      'mass_kg':float(sum(m.body_mass)),'new_geometry_axle_distance_m':bogie_model.C['axle_distance_m'],
      'peak_frame_forward_speed_m_s':max(f['telemetry']['frame_velocity_mps'][0] for f in frames),
      'speed_at_supply_start_m_s':brake_start['frame_velocity_mps'][0],
      'speed_at_exhaust_start_m_s':brake_end['frame_velocity_mps'][0],
      'final_frame_forward_speed_m_s':float(d.qvel[0]),'travel_m':float(d.qpos[0])-initial_x,
      'peak_brake_gauge_pressure_Pa':max(f['telemetry']['software_signals']['values']['axle_1_brake_pressure_gauge_Pa'] for f in frames),
      'final_brake_gauge_pressure_Pa':end['software_signals']['values']['axle_1_brake_pressure_gauge_Pa'],
      'initial_reservoir_gauge_pressure_Pa':bp.reservoir_initial_gauge_pressure_Pa,
      'final_reservoir_gauge_pressure_Pa':end['software_signals']['values']['reservoir_pressure_gauge_Pa'],
      'maximum_mass_balance_residual_kg':max_mass_residual,'gas_mass_balance_residual_kg':gas['mass_balance_residual_kg'],
      'maximum_gas_energy_balance_residual_J':max_gas_energy_residual,
      'maximum_spool_energy_balance_residual_J':max_spool_residual,
      'maximum_motor_step_electrical_balance_residual_J':max_elec_residual,
      'maximum_motor_step_split_residual_J':max_coupling_step,
      'motors':ma,'gas_audit':gas,'spool_audit':brake.network.spool_audit(),
      'initial_mechanical_energy_J':initial_E,'final_mechanical_energy_J':float(sum(d.energy)),
      'motor_mechanical_work_J':motor_work_J,'brake_heat_J':brake_J,'known_damping_heat_J':damping_J,
      'unmodeled_contact_and_integration_energy_residual_J':float(sum(d.energy))-initial_E-motor_work_J+brake_J+damping_J,
      'constraint_work_J':constraint_work_J,
      'mechanical_after_constraint_work_residual_J':float(sum(d.energy))-initial_E-motor_work_J+brake_J+damping_J-constraint_work_J,
      'constraint_work_boundary':'Signed MuJoCo qfrc_constraint dot midpoint generalized velocity; includes normal stabilization/compliance and tangential contact, not all classed as friction heat. No joint limits activated.',
      'minimum_brake_dissipation_power_W':min_brake_power,'max_abs_swing_angle_rad':max_abs_swing,
      'minimum_sampled_wheel_normal_N':min_contact_normal,'warning_counts':max_warnings.tolist(),
      'piston_displacement_modeled':False,'piston_boundary_work_J':gas['gas_boundary_work_J'],
      'hardware_io_performed':False,'sensor_raw_generated':False,
      'scope':'UNCALIBRATED ENGINEERING EQUIVALENTS; no verified brake cylinder, traction hardware, sensor encoding, or experimental validation'}
    summary['numerical_acceptance_pass']=bool(not max_warnings.any() and max_mass_residual<1e-11 and max_gas_energy_residual<1e-7 and max_spool_residual<1e-8 and max_elec_residual<1e-9 and min_brake_power>-1e-7 and max_abs_swing<.179 and min_contact_normal>0 and summary['peak_frame_forward_speed_m_s']>.1 and (summary['final_frame_forward_speed_m_s']<.01 if case=='traction_supply_exhaust' else summary['final_frame_forward_speed_m_s']>.1))
    payload={'schema':'gtsd-physics-trajectory-v1','subsystem':'drive_brake','case_id':case,
      'source_geometry_sha256':hashlib.sha256((ROOT.parent/'web/geometry.json').read_bytes()).hexdigest(),
      'binding_reference':'../bogie/binding.json','frames':frames,
      'parameters':{'bogie':p,'drive':asdict(mp),'brake':asdict(bp)},'simulation_status':summary,
      'coordinate_system':'metres; new geometry world X along car, Y transverse, Z up; wxyz quaternion; no scaling'}
    folder=output/case;folder.mkdir(parents=True,exist_ok=True)
    (folder/'trajectory.json').write_text(json.dumps(payload,ensure_ascii=False,separators=(',',':')))
    (folder/'summary.json').write_text(json.dumps(summary,indent=2));(folder/'model.xml').write_text(xml)
    with (folder/'telemetry.csv').open('w',newline='') as f:
        keys=['time_s','x_m','speed_m_s','acceleration_m_s2','reservoir_gauge_Pa','brake_gauge_Pa','axle_1_speed_rad_s','motor_1_current_A','motor_1_torque_Nm','brake_1_torque_Nm','motor_work_J','brake_heat_J','mass_residual_kg']
        writer=csv.writer(f);writer.writerow(keys)
        for row in frames:
            q=row['telemetry'];s=q['software_signals']['values'];a=q['motors']['axle_1'];b=q['brakes']['axle_1']
            writer.writerow([row['time_s'],q['frame_position_m'][0],q['frame_velocity_mps'][0],q['frame_acceleration_m_s2'],s['reservoir_pressure_gauge_Pa'],s['axle_1_brake_pressure_gauge_Pa'],s['axle_1_relative_spin_speed_rad_s'],a['audit']['current_A'],a['torque_Nm'],b['torque_Nm'],q['energy_boundary']['motor_mechanical_work_J'],q['energy_boundary']['friction_brake_heat_J'],q['pneumatic']['audit']['mass_balance_residual_kg']])
    np.savez_compressed(folder/'final_state.npz',qpos=d.qpos,qvel=d.qvel,time=d.time)
    np.savez_compressed(folder/'dense_trace.npz',**{key:np.array([row[key] for row in dense]) for key in dense[0]})
    trace_schema={
      'schema':'gtsd-drive-brake-dense-trace-v1','rows':len(dense),'timestep_s':dt,
      'time_convention':'Each row is an endpoint state. last_step_* fields describe the preceding interval, zero at initial row; commands_next_step are applied after this row.',
      'qpos_qvel_units':'MuJoCo SI generalized coordinates/velocities: translations m,m/s; hinges rad,rad/s; free-body quaternion dimensionless wxyz.',
      'joints':[{'name':helpers.name(m,mujoco.mjtObj.mjOBJ_JOINT,j),'type':int(m.jnt_type[j]),'qpos_start':int(m.jnt_qposadr[j]),'qvel_start':int(m.jnt_dofadr[j])} for j in range(m.njnt)],
      'body_names':[helpers.name(m,mujoco.mjtObj.mjOBJ_BODY,j) for j in range(1,m.nbody)],
      'command_columns':list(command_at(case,0)), 'gas_node_columns':list(brake.network.volumes),
      'port_columns':list(brake.network.orifices),'spool_columns':list(brake.network.spools),
      'axle_columns':['axle_1','axle_2'],'gas_audit_columns':list(gas_audit),
      'spool_audit_columns':list(spool_audit),'motor_audit_columns':list(motors[0].audit()),
      'stored_energy_note':'MuJoCo kinetic energy includes reflected rotor armature. Do not add rotor kinetic energy again.',
      'raw_sensor_encoding':None,'source_hashes':source_hashes(),
    }
    (folder/'dense_trace_schema.json').write_text(json.dumps(trace_schema,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('motors','gas_audit','spool_audit')}),flush=True)
    return summary


def write_metadata(mp,bp,p):
    hashes=source_hashes()
    rows=[dict(parameter='drive.'+k,value=v,evidence_level='EXPLICIT_UNCALIBRATED_ENGINEERING_ASSUMPTION',source='RL permanent-magnet/DC-equivalent motor, not identified physical motor') for k,v in asdict(mp).items()]
    rows += [dict(parameter='brake.'+k,value=v,evidence_level='EXPLICIT_UNCALIBRATED_ENGINEERING_ASSUMPTION',source='Fixed-volume pneumatic clamp equivalent; no inferred cylinder stroke/area/lever geometry') for k,v in asdict(bp).items()]
    rows += [dict(parameter='bogie.'+k,value=v,evidence_level='INHERITED_NEW_BOGIE_PARAMETER',source='../bogie/PARAMETER_PROVENANCE.csv; no added calibration') for k,v in p.items()]
    (ROOT/'parameters.json').write_text(json.dumps({'schema':'gtsd-drive-brake-parameters-v1','drive':asdict(mp),'brake':asdict(bp),'bogie':p,'source_sha256':hashes,'vendor_sha256':VENDOR_HASHES,'parameter_provenance':rows,'spool_parameters':asdict(brake_spool_parameters())},ensure_ascii=False,indent=2))
    with (ROOT/'PARAMETER_PROVENANCE.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['parameter','value','evidence_level','source']);w.writeheader();w.writerows(rows)
    binding=json.loads((BOGIE/'binding.json').read_text());binding['subsystem']='drive_brake';binding['reused_body_binding_source']='../bogie/binding.json';binding['drive_brake_note']='Same new-geometry rigid bodies; no physical brake-cylinder geometry or stroke animation added.'
    (ROOT/'binding.json').write_text(json.dumps(binding,ensure_ascii=False,indent=2))
    (ROOT/'vendor_manifest.json').write_text(json.dumps({'copies_are_byte_identical':True,'source_tree':'gtsd-bjtu-digital-twin/source/vehicle-physics','sha256':VENDOR_HASHES,'review':'Only B8 finite-volume gas network and B9 balanced spool kernel imported. No B8/B9 vehicle mechanics or vehicle factory called.'},indent=2))

def brake_spool_parameters():
    from coupling import SpoolParameters
    return SpoolParameters()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['all','traction_supply_exhaust','power_loss_coast'],default='all');parser.add_argument('--dt',type=float,default=.0005);parser.add_argument('--output',type=Path,default=ROOT/'results');args=parser.parse_args()
    p=bogie_model.DEFAULT|{'timestep_s':args.dt};mp=DriveParameters();bp=BrakeParameters(gas_substep_s=min(.0005,args.dt))
    if args.output==ROOT/'results':write_metadata(mp,bp,p)
    cases=['traction_supply_exhaust','power_loss_coast'] if args.case=='all' else [args.case]
    summaries=[run(c,p,args.output,mp,bp) for c in cases]
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2))
    if args.output==ROOT/'results' and args.case=='all':shutil.copyfile(args.output/'traction_supply_exhaust/trajectory.json',ROOT/'trajectory.json')

if __name__=='__main__':main()
