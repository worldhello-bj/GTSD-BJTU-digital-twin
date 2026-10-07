"""Generate the B11 solver derivative from the byte-pinned B8 source.

The frozen source is never edited. Every local change is an exact, guarded
replacement here, including the additional native cable reaction in momentum.
"""
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    data = (ROOT.parent/"b8/simulate_full.py").read_bytes()
    if hashlib.sha256(data).hexdigest() != "b96df8388a6f9fda55acb4b88d083397fe61ff73890462e6729b4a9d1a8e4766":
        raise RuntimeError("unexpected frozen solver version")
    text = data.decode("utf-8")
    def replace(old, new):
        nonlocal text
        if text.count(old) != 1:
            raise RuntimeError("guarded B8 derivative replacement did not match uniquely: "+old[:80])
        text = text.replace(old, new)
    replace('"""Coupled forward dynamics', '"""B11 source-cable derivative of frozen B8: coupled forward dynamics')
    replace("sys.path.insert(0,str(ROOT.parent/'vendor'))", "sys.path.insert(0,str(ROOT.parent/'vendor'))\nsys.path.append(str(ROOT.parent/'b8'))\nsys.path.append(str(ROOT.parent/'b10'))")
    replace("from full_model import model,PARAMS", "from full_model import PARAMS as B8_PARAMS\nPARAMS=B8_PARAMS|dict(cable_maximum_link_length_m=.6)\nimport cables\nfrom cables import model")
    replace("from pneumatics import make_vehicle_network,GasVolume,AMBIENT_PA", "from pneumatics import GasVolume,AMBIENT_PA\nfrom auxiliaries import make_vehicle_network")
    start, end = text.index("CASES={"), text.index("\n\ndef run(")
    text = text[:start]+"""CASES={
 'service_brake':dict(torque=True,brake=True),
 'half_dt':dict(torque=True,brake=True,timestep_s=.000025),
 'spatial_refinement':dict(torque=True,brake=True,cable_maximum_link_length_m=.3),
 'doors_open_close':dict(torque=False,brake=False,panto=False,doors=True),
 'doors_half_dt':dict(torque=False,brake=False,panto=False,doors=True,timestep_s=.000025),
 'tether_release':dict(torque=True,brake=True,tether_release=True),
}
"""+text[end:]
    replace(" startwall=time.time()", """ startwall=time.time()
 sources={str(f.relative_to(ROOT.parent)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [ROOT/'simulate.py',ROOT/'cables.py',ROOT/'source_cables.json',ROOT.parent/'build_model.py',ROOT.parent/'b10/auxiliaries.py',ROOT.parent/'b9/valve_dynamics.py',*[ROOT.parent/'b8'/n for n in ('full_model.py','compliant_contact.py','pantograph.py','access_doors.py','pneumatics.py')]]}""")
    replace(" m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);dt=float(m.opt.timestep)", """ m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);dt=float(m.opt.timestep)
 cable_spec=cables.LAST_SPEC
 cable_ends={r['key']:(int(m.site('cable_'+r['key']+'_end').id),int(m.site('cable_'+r['key']+'_anchor').id)) for r in cable_spec['routes']}
 tether_eq=[int(m.equality('cable_'+r['key']+'_attachment').id) for r in cable_spec['routes'] if r['key'].startswith('supply_')]""")
    old_start = text.index(" collector_mask=np.array")
    old_end = text.index(" def comvelocity():", old_start)
    text = text[:old_start]+""" momentum_impulse=np.zeros(3)
 root_linear_dofs=[int(m.joint(b+'_car_free').dofadr[0]) for b in ('A','B')]
"""+text[old_end:]
    old_start = text.index("   if d.ncon:\n    head_contacts=")
    old_end = text.index("   momentum_impulse+=netforce*dt", old_start)
    text = text[:old_start]+"""   # World translation DOFs of each train root include all native external
   # constraints, including cable attachment and collector contact reactions.
   # Internal train contacts cancel. Cable floor force is outside train mass.
   for vi in root_linear_dofs:netforce+=d.qfrc_constraint[vi:vi+3]
"""+text[old_end:]
    replace(" rows=[];frames=[];force=np.zeros(6);", " rows=[];frames=[];aux_rows=[];tether_released=False;force=np.zeros(6);")
    replace(" for _ in range(round(6/dt)):step(settlecmd)", """ settle_s=cfg.get('settle_s',6.)
 for jj in range(round(settle_s/dt)):
  step(settlecmd)
  if (jj+1)%round(1/dt)==0:print(f'{name}: settling {(jj+1)*dt:.1f}s wall={time.time()-startwall:.1f}s',flush=True)""")
    replace("  vals=dict(rail_normal_N=0.,", "  vals=dict(cable_normal_N=0.,cable_contacts=0,cable_friction_power_W=0.,rail_normal_N=0.,")
    replace("   if any('flange_' in n for n in nn):kind='flange'", "   if any(n.startswith('cable_') for n in nn):kind='cable'\n   elif any('flange_' in n for n in nn):kind='flange'")
    replace("   if kind in ['brake','rail']:", "   if kind in ['brake','rail','cable']:")
    replace("  t=d.time-start\n  if door_state:", """  t=d.time-start
  if cfg.get('tether_release') and t>=2 and not tether_released:
   d.eq_active[tether_eq]=False;tether_released=True
  if door_state:""")
    replace("   rows.append(row)", """   row.update({'cable_'+key+'_attachment_error_m':float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for key,(a,b) in cable_ends.items()})
   for key in cable_ends:
    eqid=int(m.equality('cable_'+key+'_attachment').id)
    mask=(d.efc_type[:d.nefc]==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))&(d.efc_id[:d.nefc]==eqid)
    row['cable_'+key+'_attachment_force_N']=float(np.linalg.norm(d.efc_force[:d.nefc][mask]))
   row['native_train_constraint_force_x_N']=sum(float(d.qfrc_constraint[vi]) for vi in root_linear_dofs)
   row['pump_geometry_work_J']=net.pump.gas_geometry_work_J
   aux=dict(time_s=t,network_time_s=net.time_s,spool_balance_residual_J=net.spool_audit()['balance_residual_J'])
   for obj,state in {'pump':net.pump.snapshot(),**{n:f.snapshot() for n,f in net.fans.items()},**{n:th.snapshot() for n,th in net.thermal.items()}}.items():
    aux.update({obj+'_'+key:float(value) for key,value in state.items() if isinstance(value,(int,float,bool))})
   aux_rows.append(aux)
   rows.append(row)""")
    marker = " (OUT/f'{name}_metrics.json').write_text"
    at = text.index(marker)
    text = text[:at]+""" metrics.update(version='B11 source-anchored articulated cable equivalent',settle_s=settle_s,cable_configuration=cable_spec,auxiliary_final_state=net.snapshot(),momentum_scope='HC rail + gravity + applied forces + all native train root constraint reactions')
 for relative,expected in sources.items():
  if hashlib.sha256((ROOT.parent/relative).read_bytes()).hexdigest()!=expected:raise RuntimeError('Executed source changed during B11 run')
 with (OUT/f'{name}_auxiliaries.csv').open('w',newline='',encoding='utf-8') as f:
  writer=csv.DictWriter(f,fieldnames=aux_rows[0]);writer.writeheader();writer.writerows(aux_rows)
"""+text[at:]
    # Original B8 writes CSV/replay below metrics; attach identity last.
    marker = " return metrics"
    replace(marker, """ paths=[OUT/f'{name}{suffix}' for suffix in ('.xml','.csv','_metrics.json','_auxiliaries.csv')]
 if replay:paths.append(OUT/f'{name}_replay.json')
 xmlpath=OUT/f'{name}.xml';xmlpath.write_bytes(xmlpath.read_text(encoding='utf-8').encode('utf-8'))
 identity=dict(case=name,solver=mujoco.__version__,numpy=np.__version__,sources_sha256=sources,outputs_sha256={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},configuration=cable_spec)
 (OUT/f'{name}_identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
 return metrics""")
    replace("'door_final_latched']}),flush=True)", "'door_final_latched','cable_configuration','auxiliary_final_state']}),flush=True)")
    replace("  return work", "  if np.any(d.warning.number):raise RuntimeError('MuJoCo warning during B11 integration: '+str(d.warning.number.tolist()))\n  return work")
    replace("schema='B8 force driven full3D replay v1'", "schema='B11 force driven source-cable full3D replay v1'")
    replace("parameters=p,access_doors=door_metadata,frames=frames", "parameters=p,access_doors=door_metadata,cable_configuration=cable_spec,frames=frames")
    replace("from cables import model", "from cables import model\nfrom cable_contact_audit import CableContactAudit")
    replace("ROOT/'source_cables.json',ROOT.parent/'build_model.py'", "ROOT/'source_cables.json',ROOT/'cable_contact_audit.py',ROOT.parent/'build_model.py'")
    replace(" def step(commands):", " settling_cable_contact_audit=CableContactAudit(m)\n observation_cable_contact_audit=CableContactAudit(m)\n active_cable_contact_audit=settling_cable_contact_audit\n active_cable_contact_audit.record(d)\n def step(commands):")
    replace("  mujoco.mj_forward(m,d)\n  work['damping']", "  mujoco.mj_forward(m,d)\n  active_cable_contact_audit.record(d)\n  work['damping']")
    replace(" net.close_all_valves();mujoco.mj_forward(m,d)", " net.close_all_valves();mujoco.mj_forward(m,d)\n active_cable_contact_audit=observation_cable_contact_audit\n active_cable_contact_audit.record(d)")
    replace(" metrics.update(version='B11", " metrics.update(cable_contact_geometry_all_steps=dict(settling=settling_cable_contact_audit.snapshot(),observation=observation_cable_contact_audit.snapshot()))\n metrics.update(version='B11")
    (ROOT/"simulate.py").write_text(text, encoding="utf-8", newline="\n")
    print("Generated B11 solver derivative with guarded frozen-source replacements")


if __name__ == "__main__":
    main()
