"""Read-only source expansion routes; verify real controller is unchanged."""
import os
import json,time,threading,hashlib
from pathlib import Path
from http.client import HTTPConnection
from source_console import make_server,ROOT,PROJECT
out=Path(os.environ.get('GTSD_ACCEPTANCE_OUTPUT_DIR',str(ROOT/'evidence')));out.mkdir(parents=True,exist_ok=True);server,ctl,ctx=make_server(0,'cad_seat_research');ctl.start();thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();log=[]
def req(path,status=200,origin=None):
 h=HTTPConnection('127.0.0.1',server.server_port,timeout=25);h.request('GET',path,headers={'Origin':origin} if origin else{});r=h.getresponse();raw=r.read();h.close();log.append({'path':path,'status':r.status,'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()});assert r.status==status,(r.status,raw[:300]);return raw
try:
 for _ in range(200):
  s=json.loads(req('/api/assembly/runtime'))
  if s['phase']=='paused':break
  assert s['phase']!='error',s;time.sleep(.05)
 assert s['steps']==0;cp=req('/api/checkpoint');lease=ctl.last_client
 pairs={'/api/assembly/source-replacements':ROOT/'data/source_replacements.json','/api/source-dimensions/resource':ROOT/'data/source_dimensions.json','/api/source-drive/resource':PROJECT/'full_source_audit/semantic_triage/drive_comparison_geometry.json','/api/source-fixture/resource':PROJECT/'full_source_audit/semantic_triage/transformer_fixture_geometry/geometry.json'}
 pairs.update({'/api/mechanical-fit/rail-source':PROJECT/'mechanical_fit/components/track_support/rail_body_review/native_track_curve_overlay.json'})
 pairs.update({'/api/mechanical-fit/profiles':PROJECT/'mechanical_fit/profiles.json','/api/mechanical-fit/report':PROJECT/'mechanical_fit/report.json'})
 pairs.update({'/api/source-audit/index':ROOT/'data/source_audit_index.json','/api/source-library/catalog':ROOT/'data/source_library/catalog.json'})
 for item in json.loads((ROOT/'data/source_library/catalog.json').read_text())['items']:pairs[item['url']]=ROOT/'data/source_library'/(item['id']+'.json')
 for endpoint,path in pairs.items():assert req(endpoint)==path.read_bytes()
 for path in ['/mechanical-fit/rail','/mechanical-fit/rail/app.js','/mechanical-fit/rail/style.css','/assembly/mechanical-fit.js','/mechanical-fit/report','/mechanical-fit/style.css','/mechanical-fit/renders/whole_candidate.png','/assembly/source-replacements.js','/source-dimensions','/source-dimensions/app.js','/source-dimensions/style.css','/source-dimensions/native.blend','/source-dimensions/model.glb','/source-drive','/source-drive/app.js','/source-fixture','/source-fixture/app.js','/source-audit','/source-audit/app.js','/source-library','/source-library/app.js','/source-library/oblique.svg']:assert req(path)
 req('/api/mechanical-fit/unknown',404);req('/mechanical-fit/renders/../../private',404);req('/api/mechanical-fit/profiles',403,'http://foreign.invalid');req('/api/source-fixture/unknown',404);req('/source-fixture/../private',404);req('/api/source-drive/resource',403,'http://foreign.invalid')
 assert req('/api/checkpoint')==cp and json.loads(req('/api/assembly/runtime'))['steps']==0;assert ctl.last_client==lease
 assert json.loads(req('/api/pantograph-candidate/status'))['phase']=='disabled'
 result={'ok':True,'source_pages':5,'mechanical_fit_pages':2,'source_resources_byte_equal':len(pairs),'new_geometry_revision':json.loads(pairs['/api/assembly/source-replacements'].read_text())['display_revision_sha256'],'main_native_checkpoint_unchanged':True,'main_steps':0,'run_lease_unchanged':True,'pantograph_candidate_disabled':True,'unknown_and_cross_origin_rejected':True,'browser_visual_acceptance':False,'browser_download_acceptance':False}
 (out/'source_review_routes_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
 server.shutdown();server.server_close();thread.join();ctl.close();(out/'source_review_routes_http_log.json').write_text(json.dumps(log,indent=2))
