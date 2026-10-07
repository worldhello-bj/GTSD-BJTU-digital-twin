"""Build three independent, hash-verified ZIPs, each below 15,000,000 bytes.
Only final cases are selected by CASES. Historical evidence has explicit paths.
"""
from pathlib import Path
import json,hashlib,zipfile,zlib
from simulate_full import CASES
ROOT=Path(__file__).resolve().parent;PROJ=ROOT.parent;OUT=ROOT/'results';DEST=ROOT/'deliverable';DEST.mkdir(exist_ok=True)
verify=json.loads((OUT/'verification.json').read_text());assert verify['passed'] and verify['case_count']==22
files=[]
def add(p,arc=None,group='B'):
 p=Path(p)
 if not p.is_file():raise FileNotFoundError(p)
 arc=arc or 'vehicle-physics/b8/'+str(p.relative_to(ROOT))
 if any(x[1]==arc for x in files):return
 files.append((p,arc,group))
for n in ['build_model.py','requirements.txt','DELIVERY_MANIFEST.json']:add(PROJ/n,'vehicle-physics/'+n)
for n in ['full_model.py','pneumatics.py','pantograph.py','access_doors.py','compliant_contact.py','simulate_full.py','run_suite.py','verify_full.py','plot_full.py','plot_contact_acceptance.py','render_full.py','write_report.py','live_viewer_full.py','test_live_controls.py','test_pneumatics.py','test_pantograph.py','test_access_doors.py','test_compliant_contact.py','run_overload_diagnostic.py','package_delivery.py','diagnose_flange.py','test_contact_candidates.py','run_compliant_candidate.py','README.md','RESULTS_ZH.txt','FINAL_ACCEPTANCE.md','CONTACT_REPAIR.md','BRAKE_CIRCUIT_SCOPE.md','COVERAGE.md','SOURCE_CONSISTENCY.md','PNEUMATICS.md','PANTOGRAPH.md','ACCESS_DOORS.md','LIVE_CONTROLS.md','PARAMETER_PROVENANCE.csv','parameters.json','B7_PRESERVATION.json']:add(ROOT/n)
for directory in ['pneumatic_results','pantograph_results','door_results']:
 for p in sorted((ROOT/directory).iterdir()):
  if p.is_file() and p.suffix in ['.json','.csv','.xml','.png','.log']:add(p)
main_replays={'service_brake','doors_open_close','flange_disturbance','low_adhesion','low_adhesion_half_dt'}
for n in CASES:
 for suffix in ['.xml','_metrics.json']:add(OUT/(n+suffix))
 add(OUT/(n+'.csv'),group='C')
 add(OUT/(n+'_replay.json'),group='B' if n in main_replays else 'D')
for n in ['verification.json','metrics.json','case_summary.csv','physics_run_identity.json','live_controls_verification.json','final_unit_tests.log','final_hc77_regression.log','validation_summary.png','failure_modes.png','contact_and_doors_acceptance.png','service_brake_poster.png','service_brake_braking.png','service_brake_exhaust.png'] :add(OUT/n)
# Exact old failure and failed coarse-step evidence; never relabel as final cases.
for stem in ['overload_800N','low_adhesion_quarter_dt','hc77_service_brake']:
 for suffix in ['.xml','.csv','_metrics.json']:add(OUT/(stem+suffix), 'vehicle-physics/b8/history/'+stem+suffix)
for n in ['historical_native77_peak_failures.json','door_integration_trial.log','integrated_refinement.log']:add(OUT/n,'vehicle-physics/b8/history/'+n)
for p in (ROOT/'contact_diagnostics/frozen_project').rglob('*'):
 if p.is_file() and p.suffix in ['.py','.txt']:add(p)
for p in (ROOT/'contact_diagnostics').iterdir():
 if p.is_file() and p.suffix in ['.py','.md','.json']:add(p)
for n in ['low_adhesion','low_adhesion_half_dt']:
 for p in (ROOT/'contact_diagnostics'/n).iterdir():
  if p.is_file() and (p.suffix in ['.json','.csv','.xml'] or p.name in ['peak_full.pkl','active_contacts.jsonl.gz']):add(p)
for candidate in ['constant_95','impratio_1','impratio1_friction_tau50ms','soft_95']:
 for p in (ROOT/'contact_candidates'/candidate).iterdir():
  if p.is_file() and p.suffix in ['.csv','.json','.xml']:add(p)
# The 71-coordinate research branch is historical. Current 77-case reports supersede its status.
hc=ROOT/'contact_candidates/hunt_crossley'
for n in ['REPORT.md','comparison.json','geometry_checks.json','check_geometry.py','unit_tests.txt']:add(hc/n)
for p in hc.glob('*/summary.json'):add(p)
for n in ['source_motion_inventory.json','source_mechanism_coverage.json']:
 add(PROJ.parent/'blender-optimization/b8/reports'/n,'vehicle-physics/b8/source_evidence/'+n)
entries=[dict(path=a,package=g,bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p,a,g in files]
manifest=dict(schema='B8 final engineering delivery v2',acceptance=verify['status'],cases=22,checks=len(verify['checks']),note='Uncalibrated source-topology prototype. Historical native-contact failures remain historical; not asserted as passing current HC77.',files=entries,excluded='Third-party binaries; superseded single-car/diamond experiments; large exploratory per-step caches. Final 22 CSV/XML/metrics/replays are complete. Diagnostic scripts/reports and selected original failure states are retained.',separate_video='service_brake_solver_replay.mp4',package_roles={'B':'Runnable source, independent tests, final reports, primary replays and historical failure evidence','C':'Complete 100Hz tables for all 22 final cases','D':'Remaining complete 30Hz body-pose replays'})
man=DEST/'MANIFEST.json';man.write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
results=[]
for g,title in [('B','物理源码与验收'),('C','全部工况数值数据'),('D','其余完整动态轨迹')]:
 batch=[(p,a) for p,a,grp in files if grp==g];path=DEST/f'GTSD_BJTU_B8_{g}_{title}.zip'
 with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
  for p,a in batch:z.write(p,a)
  z.write(man,'MANIFEST.json')
 assert path.stat().st_size<15_000_000,(str(path),path.stat().st_size)
 with zipfile.ZipFile(path) as z:
  assert z.testzip() is None
  for ent in entries:
   if ent['package']==g:assert hashlib.sha256(z.read(ent['path'])).hexdigest()==ent['sha256']
 results.append(dict(path=str(path),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),file_count=len(batch)+1,crc_verified=True,manifest_hashes_verified=True))
(DEST/'PACKAGE_VERIFICATION.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,ensure_ascii=False,indent=2))
