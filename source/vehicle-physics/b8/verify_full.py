"""Numerical/causal acceptance, not hardware validation. Honest incomplete/failed state."""
from pathlib import Path
import sys,json,hashlib,ast
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'results'
from simulate_full import CASES
from full_model import PARAMS

def main():
 metrics={};data={};checks=[];notes=[]
 def check(name,condition,detail):checks.append(dict(name=name,passed=bool(condition),detail=detail))
 for n in CASES:
  path=OUT/f'{n}_metrics.json'
  if not path.exists():check('complete_'+n,False,'Case missing');continue
  m=json.loads(path.read_text());metrics[n]=m;a=np.genfromtxt(OUT/f'{n}.csv',names=True,delimiter=',');data[n]=a
  check('final_model_'+n,m['dofs']==77 and m['parameters'].get('compliant_wheel_rail') is True and abs(m['dt_s']-CASES[n].get('timestep_s',PARAMS['timestep_s']))<1e-12,{'dofs':m['dofs'],'dt_s':m['dt_s'],'HC':m['parameters'].get('compliant_wheel_rail')})
  check('finite_'+n,all(np.isfinite(a[k]).all() for k in a.dtype.names),'All sampled physical states finite')
  check('solver_'+n,not any(m['warning_counts']),m['warning_counts'])
  check('mass_'+n,m['max_air_mass_residual_kg']<1e-10,m['max_air_mass_residual_kg'])
  check('gas_work_'+n,max(abs(a['gas_energy_residual_J']))<1e-5,float(max(abs(a['gas_energy_residual_J']))))
  check('coupling_work_'+n,max(abs(a['coupling_work_discrepancy_J']))<.05,float(max(abs(a['coupling_work_discrepancy_J']))))
  check('mechanical_work_'+n,max(abs(a['mechanical_balance_residual_J']))<.15,float(max(abs(a['mechanical_balance_residual_J']))))
  check('xml_identity_'+n,m.get('model_xml_sha256')==hashlib.sha256((OUT/f'{n}.xml').read_bytes()).hexdigest(),'Model bytes match metrics')
  check('contact_passivity_'+n, max(a['contact_friction_work_J'])<1e-8 and max(a['contact_dissipation_work_J'])<1e-8,{'friction_work_J':float(a['contact_friction_work_J'][-1]),'normal_dissipation_work_J':float(a['contact_dissipation_work_J'][-1])})
  replay=json.loads((OUT/f'{n}_replay.json').read_text())
  check('replay_identity_'+n,replay['model_xml_sha256']==m['model_xml_sha256'] and len(replay['reference_zero_qpos_body_transforms'])==44,{'body_count':len(replay['reference_zero_qpos_body_transforms'])})
  if 'momentum_balance_residual_Ns' in a.dtype.names:
   check('momentum_'+n,max(abs(a['momentum_balance_residual_Ns']))<.05,float(max(abs(a['momentum_balance_residual_Ns']))))
  else:notes.append(n+': saved before the added momentum observer; no momentum-audit claim for this dataset. Forces/parameters unchanged, fine-step low-mu has the observer.')
  if m.get('wheel_partial_contact_steps',0)>0:notes.append(f'{n}: {m["wheel_partial_contact_steps"]} steps with fewer than 8 active tread contacts; flange or transient unloading may support loads. Not derailment validated.')
  if m['max_contact_penetration_m']>.0005:notes.append(f'{n}: maximum soft-contact penetration {m["max_contact_penetration_m"]*1000:.3f} mm exceeds nominal-case scale; high-fidelity contact forces not validated.')
  if m['peak_collector_single_contact_N']>100:notes.append(f'{n}: single-point collector impact reaches {m["peak_collector_single_contact_N"]:.1f} N; no acceptable-force certification.')
 if len(metrics)==len(CASES):
  M=metrics;A=data;s=M['service_brake'];f=M['half_dt'];tight=M['solver_tight']
  check('no_uncommanded_drive',abs(M['power_off']['travel_m'])<.002,M['power_off']['travel_m'])
  check('torque_causes_motion',M['coast']['final_speed_mps']>.2,M['coast']['final_speed_mps'])
  check('pressure_friction_stops',s['stop_distance_m'] is not None and abs(s['final_speed_mps'])<.01,s['stop_distance_m'])
  check('added_mass_slows_acceleration',M['loaded_brake']['speed_at_brake_start_mps']<s['speed_at_brake_start_mps']*.95,M['loaded_brake']['speed_at_brake_start_mps'])
  check('zero_pad_mu_prevents_stop',M['brake_mu_zero']['final_speed_mps']>.15,M['brake_mu_zero']['final_speed_mps'])
  check('no_supply_prevents_brake',M['no_supply_pressure']['final_speed_mps']>.15,M['no_supply_pressure']['final_speed_mps'])
  check('no_pressure_no_panto_lift',float(max(A['no_supply_pressure']['pantograph_cylinder_m']))<.0001 and float(max(A['no_supply_pressure']['collector_normal_N']))<.01,{'max_cylinder_retraction_m':float(max(A['no_supply_pressure']['pantograph_cylinder_m'])),'max_contact_N':float(max(A['no_supply_pressure']['collector_normal_N']))})
  check('brake_leak_changes_outcome',M['brake_cylinder_branch_leak']['final_speed_mps']>s['final_speed_mps']+.05,M['brake_cylinder_branch_leak']['final_speed_mps'])
  check('reservoir_loss_is_physical',A['reservoir_loss']['reservoir_pressure_Pa'][-1]<A['service_brake']['reservoir_pressure_Pa'][-1]-.1e6,float(A['reservoir_loss']['reservoir_pressure_Pa'][-1]))
  check('air_spring_leak_changes_roll',M['air_spring_leak']['max_roll_rad']>s['max_roll_rad']+.0005,M['air_spring_leak']['max_roll_rad'])
  check('lateral_yaw_dofs_respond',M['lateral_disturbance']['max_lateral_m']>.001 and M['lateral_disturbance']['max_yaw_rad']>.001,[M['lateral_disturbance']['max_lateral_m'],M['lateral_disturbance']['max_yaw_rad']])
  check('flange_unilateral_limit_active',M['flange_disturbance']['peak_flange_single_contact_N']>0,M['flange_disturbance']['peak_flange_single_contact_N'])
  nom=A['service_brake'];dist=A['collector_disturbance'];after=dist[(dist['time_s']>5)&(dist['time_s']<5.21)]
  check('collector_disturbance_separates',min(after['collector_normal_N'])<.01 and min(after['collector_height_m'])<1.89,float(min(after['collector_height_m'])))
  check('panto_lifts_and_exhausts',max(nom['collector_normal_N'])>1 and nom['collector_height_m'][-1]<max(nom['collector_height_m'])-.10,[float(max(nom['collector_normal_N'])),float(nom['collector_height_m'][-1])])
  q=np.argmin(abs(nom['time_s']-8.3));normal=A['normal_exhaust'];nn=np.argmin(abs(normal['time_s']-8.3))
  check('quick_exhaust_is_faster',nom['pantograph_pressure_Pa'][q]<normal['pantograph_pressure_Pa'][nn],float(normal['pantograph_pressure_Pa'][nn]-nom['pantograph_pressure_Pa'][q]))
  rel=A['brake_release'];check('brake_exhaust_and_redrive',rel['speed_mps'][-1]>.15 and rel['brake_pressure_Pa'][-1]<103000,[float(rel['speed_mps'][-1]),float(rel['brake_pressure_Pa'][-1])])
  check('eight_treads_baseline',s['minimum_wheel_treads_in_contact']==8,s['minimum_wheel_treads_in_contact'])
  if s['stop_distance_m'] and f['stop_distance_m']:
   conv=abs(f['stop_distance_m']/s['stop_distance_m']-1)
   check('halfstep_stop_distance',conv<.025,conv)
  else:check('halfstep_stop_distance',False,'No sustained stop in one case')
  lm=M['low_adhesion'];lh=M['low_adhesion_half_dt']
  check('lowmu_halfstep_stop_distance',abs(lh['stop_distance_m']/lm['stop_distance_m']-1)<.025,abs(lh['stop_distance_m']/lm['stop_distance_m']-1))
  check('lowmu_collector_peak_convergence',abs(lh['peak_collector_single_contact_N']/lm['peak_collector_single_contact_N']-1)<.1,abs(lh['peak_collector_single_contact_N']/lm['peak_collector_single_contact_N']-1))
  check('lowmu_flange_peak_convergence',abs(lh['peak_flange_single_contact_N']/lm['peak_flange_single_contact_N']-1)<.2,{'relative_change':abs(lh['peak_flange_single_contact_N']/lm['peak_flange_single_contact_N']-1),'dt_0_05ms_N':lm['peak_flange_single_contact_N'],'dt_0_025ms_N':lh['peak_flange_single_contact_N'],'limit':.2,'meaning':'Original 20% criterion unchanged. Numerical convergence is not physical calibration.'})
  lc=M['low_adhesion_coarse_dt']
  check('lowmu_coarse_to_main_flange_peak',abs(lm['peak_flange_single_contact_N']/lc['peak_flange_single_contact_N']-1)<.2,{'coarse_N':lc['peak_flange_single_contact_N'],'main_N':lm['peak_flange_single_contact_N'],'relative_change':abs(lm['peak_flange_single_contact_N']/lc['peak_flange_single_contact_N']-1),'limit':.2})
  fa=M['flange_disturbance'];fb=M['flange_half_dt']
  check('axle_disturbance_flange_peak',abs(fb['peak_flange_single_contact_N']/fa['peak_flange_single_contact_N']-1)<.2,{'main_N':fa['peak_flange_single_contact_N'],'fine_N':fb['peak_flange_single_contact_N'],'relative_change':abs(fb['peak_flange_single_contact_N']/fa['peak_flange_single_contact_N']-1),'limit':.2})
  check('unloading_not_disabled',fa['minimum_wheel_treads_in_contact']<8 and fb['minimum_wheel_treads_in_contact']<8,[fa['wheel_partial_contact_steps'],fb['wheel_partial_contact_steps']])
  rail=A['service_brake']['rail_force_world_z_N'][0];weight=s['total_mass_kg']*9.81
  check('settled_rail_weight',abs(rail/weight-1)<.01,{'rail_force_z_N':float(rail),'train_weight_N':weight})
  da=M['doors_open_close'];db=M['doors_half_dt']
  check('six_dynamic_doors',len(da['door_max_opening_rad'])==6,da['door_max_opening_rad'])
  check('doors_open_to_limits',all(abs(angle-(1.31 if key.startswith('bay') else np.deg2rad(110)))<.005 for key,angle in da['door_max_opening_rad'].items()),da['door_max_opening_rad'])
  check('doors_passively_reclose_and_latch',all(da['door_final_latched'].values()) and all(abs(v)<.006 for v in da['door_final_angles_rad'].values()),da['door_final_angles_rad'])
  door_diff=max(float(max(abs(A['doors_open_close'][k]-A['doors_half_dt'][k]))) for k in A['doors_open_close'].dtype.names if k.startswith('door_') and k.endswith('_angle_rad'))
  check('doors_halfstep_trajectory',door_diff<.01,{'max_angle_difference_rad':door_diff,'limit_rad':.01})
  check('doors_latched_baseline',all(s['door_final_latched'].values()) and max(s['door_max_opening_rad'].values())<1e-5,s['door_max_opening_rad'])
  check('tight_solver_speed',abs(tight['final_speed_mps']-s['final_speed_mps'])<.003,abs(tight['final_speed_mps']-s['final_speed_mps']))
  check('low_adhesion_slips',max(abs(A['low_adhesion']['front_slip_mps']))>.2,float(max(abs(A['low_adhesion']['front_slip_mps']))))
 source=(ROOT/'simulate_full.py').read_text();tree=ast.parse(source)
 writes=[]
 for node in ast.walk(tree):
  if isinstance(node,(ast.Assign,ast.AugAssign,ast.AnnAssign)):
   targets=node.targets if isinstance(node,ast.Assign) else [node.target]
   for target in targets:
    st=ast.unparse(target)
    if st.startswith(('d.qpos[','d.qvel[')):writes.append(st)
 check('no_mechanical_state_overrides',not writes,writes)
 historical_path=OUT/'overload_800N_metrics.json'
 if not historical_path.exists():historical_path=ROOT/'history/overload_800N_metrics.json'
 overload=json.loads(historical_path.read_text()) if historical_path.exists() else None
 if overload:notes.append('Historical native-71 800 N high-carbody overload FAILED the intended model domain: overturn/derailment and large discrete-work residual. It is retained and is not counted as a passing operational case.')
 report=dict(out_of_domain_case=({'case':'overload_800N','status':'OUTSIDE_VALIDATED_DOMAIN','mechanical_final_residual_J':overload['final']['mechanical_balance_residual_J']} if overload else None),status='PASS_WITH_MODEL_LIMITATIONS' if all(c['passed'] for c in checks) and len(metrics)==len(CASES) else 'INCOMPLETE_OR_FAILED',passed=all(c['passed'] for c in checks),checks=checks,case_count=len(metrics),limitations=notes+['All equipment masses, inertia, stiffness, friction and air-flow parameters remain uncalibrated.','Two-car topology preserves source-project assumptions; as-built vehicle count is not confirmed.','Wheel contact, ideal transmission, pressure struts and rigid overhead are documented reduced models.','Discrete work closure includes explicit contact storage/dissipation and native soft-contact constraint work; it is not physical-test energy validation.'])
 (OUT/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 return report
if __name__=='__main__':main()
