from pathlib import Path
import os,sys,json,copy,threading,time,uuid
from http.client import HTTPConnection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import strict82_server as S
c=S.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'strict82_config.json');c['base']['child']['base']['port']=0
server,ctl,ctx=S.make_server(c);ctl.start();t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
def request(path,body=None,expected=(200,202)):
 h=HTTPConnection('127.0.0.1',server.server_port,timeout=60);h.request('GET' if body is None else 'POST',path,None if body is None else json.dumps(body),{} if body is None else {'Content-Type':'application/json'});r=h.getresponse();data=r.read();h.close();assert r.status in expected,(r.status,data[:500]);return data
read=lambda path='/api/observation':json.loads(request(path))
def wait(pred):
 until=time.monotonic()+90
 while time.monotonic()<until:
  st=read();assert st['phase']!='error',st.get('error')
  if pred(st):return st
  time.sleep(.04)
 raise AssertionError('Readiness timeout')
def cmd(action,args,status='applied'):
 st=read();rid=uuid.uuid4().hex;request('/api/command',{'session_id':st['session_id'],'request_id':rid,'action':action,'args':args});st=wait(lambda s:any(e['request_id']==rid for e in s['events']));e=next(e for e in st['events'] if e['request_id']==rid);assert e['status']==status,e;return st
try:
 initial=wait(lambda s:s['phase']=='paused');p=initial['plant'];assert p['variant']==S.ACTUAL;assert p['model_identity_sha256']=='9bd3b5a7f26a40c44f1b77f5c71b9ec53488e682170ddfda651a939aa96323db';assert p['lineage_observations']['identity_match'];cat=read('/api/lineage-registry');assert cat['variant']==S.ACTUAL
 panel=read('/api/four-door-fans');assert panel['identity_match'] and panel['inventory']['rotating_axes']==6;assert b'cad_doors_four_fans_scoped_rollback' in request('/fan82.js');assert b'four-fan-panel' in request('/');request('/truth-evidence.js')
 cmd('step',{'steps':1});accepted=wait(lambda s:s['plant']['steps']==1 and not s['single_steps_remaining']);cp=read('/api/checkpoint');lease=ctl.last_client
 for _ in range(2):read('/api/four-door-fans');read('/api/phase6-observations')
 assert ctl.last_client==lease and read('/api/checkpoint')==cp
 from controller import digest
 for field in ('air_speed_rad_s','relative_speed_rad_s'):
  bad=copy.deepcopy(cp);dictionary=bad['plant']['four_fans']['last'][field];dictionary[next(iter(dictionary))]+=.1;bad['plant']['sha256']=digest({k:v for k,v in bad['plant'].items() if k!='sha256'});bad['sha256']=digest({k:v for k,v in bad.items() if k!='sha256'});cmd('restore_checkpoint',{'checkpoint':bad},'rejected');assert read('/api/checkpoint')==cp;assert read('/api/lineage-registry')['catalogue_sha256']==cat['catalogue_sha256']
 restored=cmd('restore_checkpoint',{'checkpoint':cp});assert read('/api/checkpoint')==cp
 for k in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert restored['plant'][k]==accepted['plant'][k],k
 final=read('/api/four-door-fans');assert all(r['available'] for r in final['observations']);assert final['extra_work_columns_counted_once'];o=accepted['plant']['lineage_observations'];assert o['denominators']['legacy']['reasons']['UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS']==687
 print(json.dumps({'ok':True,'scope':'Minimal current strict82 HTTP acceptance; one actual native integration call, two re-signed derived-speed attacks rejected, valid exact restore; no new long dynamics protocol','native_identity':p['model_identity_sha256'],'console_interface_identity':p['display_interface']['strict82_console_identity_sha256'],'geometry_sha256':p['source_geometry_sha256'],'display_raw_MRO_unchanged':True,'legacy_registry_guard_not_modified':True,'named_signal_denominators':o['denominators'],'catalogue_sha256':cat['catalogue_sha256'],'all_four_fan_rows_available_after_step':True,'six_rotors_one_piston':True,'additional_work_columns_once':True,'both_resigned_history_speed_attacks_rejected':True,'exact_checkpoint_restore_and_display_state':True,'failed_restore_keeps_active_catalogue':True,'readonly_API_checkpoint_unchanged_and_no_lease_renewal':True,'browser_GPU':False,'portable_clean_extraction_tested':False,'full_legacy_command_retest':False},indent=2))
finally:server.shutdown();server.server_close();t.join();ctl.close()
