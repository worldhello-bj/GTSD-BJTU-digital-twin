"""Rebuild the complete audit from the published repository without saved exports.
Usage: python reproduce.py --project-root path/to/cad-source-console --output audit-output
Requires Python + NumPy, Node.js, and the preinstalled GEOS C library. No Blender.
"""
import argparse,json,os,shutil,subprocess,sys,tempfile
from pathlib import Path
CODE=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--project-root',required=True);ap.add_argument('--output',required=True);ap.add_argument('--blend');args=ap.parse_args();project=Path(args.project_root).resolve();out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
required=['mechanical_fit/export_scene.js','mechanical_fit/profiles.json','integration/data/source_replacements.json','integration/data/source_library/chair_conflict_01_source_candidate.json']
for name in required:
 if not (project/name).is_file():raise SystemExit('Missing required public repository resource: '+name)
env=os.environ.copy();env['OPENBLAS_NUM_THREADS']='1';env['OMP_NUM_THREADS']='1'
script_names=['audit_combined.py','triangle_classifier.py','refine_interfaces.py','cap_geometry.py','geos_kernel.py','test_audit.py','finalize_report.py','reproduce.py','README_ZH.md']
for name in script_names:
 if CODE/name!=out/name:shutil.copyfile(CODE/name,out/name)
def run(cmd,log,cwd=None):
 with (out/log).open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True,env=env,cwd=cwd)
with tempfile.TemporaryDirectory(prefix='gtsd-combined-audit-') as tmp:
 staging=Path(tmp)
 run(['node',str(project/'mechanical_fit/export_scene.js'),str(staging)],'rebuild_exports.log')
 source=staging/'source.json.gz';combined=staging/'combined.json.gz';shared=['--source',str(source),'--combined',str(combined)]
 common=shared+['--profiles',str(project/'mechanical_fit/profiles.json'),'--replacements',str(project/'integration/data/source_replacements.json')]
 run([sys.executable,str(CODE/'audit_combined.py'),*common,'--output',str(out)],'audit.log')
 run([sys.executable,str(CODE/'audit_combined.py'),*common,'--float32','--output',str(out/'native_float32')],'native_float32.log')
 run([sys.executable,str(CODE/'refine_interfaces.py'),*shared,'--audit',str(out/'combined_interference_audit.json'),'--source-library',str(project/'integration/data/source_library'),'--output',str(out)],'refine.log')
 run([sys.executable,'-m','unittest','-v','test_audit'],'regression_tests.log',cwd=CODE)
 data=json.loads((out/'combined_interference_audit.json').read_text());native=json.loads((out/'native_float32/combined_interference_audit.json').read_text())
 assert data['counts']==native['counts'] and data['after_status_counts']==native['after_status_counts']
 (out/'reproduction_validation.json').write_text(json.dumps({'schema':'gtsd-public-repo-audit-reproduction-v1','completed':True,'public_exporter':'mechanical_fit/export_scene.js','public_profiles':'mechanical_fit/profiles.json','source_mesh_copies_retained_in_delivery':False,'temporary_exports_cleaned_on_exit':True,'blender_started':False,'simulation_started':False,'project_files_modified':False,'primary_counts':data['counts'],'precision_classifications_match':True,'analytic_regression_tests_passed':7},ensure_ascii=False,indent=2)+'\n')
 extra=['--blend',str(Path(args.blend).resolve())] if args.blend else []
 run([sys.executable,str(CODE/'finalize_report.py'),*shared,'--output',str(out),*extra],'finalize.log')
 print(json.dumps({'completed':True,'before_meshes':data['counts']['before_meshes'],'after_meshes':data['counts']['after_meshes'],'after_status_counts':data['after_status_counts'],'public_entrypoint':'reproduce.py --project-root <public-repository> --output <report-folder>'},ensure_ascii=False))
