"""Three accepted native calls incl repeated continuation; actual loopback HTTP."""
from pathlib import Path
import sys,os,copy,json,threading,time,uuid,hashlib
from http.client import HTTPConnection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import fan82_server as F

def run():
 c=F.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console82_config.json');assert c['variant']==F.VARIANT;c['child']['base']['port']=0;assert F.doctor(c)['ok']
 server,ctl,ctx=F.make_server(c);thread=threading.Thread(target=server.serve_forever,daemon=True);ctl.start();thread.start();timings=[]
 def request(path,data=None,headers=None,expected=(200,202)):
  start=time.perf_counter();h=HTTPConnection('127.0.0.1',server.server_port,timeout=90);h.request('GET' if data is None else 'POST',path,None if data is None else json.dumps(data),headers or ({} if data is None else {'Content-Type':'application/json'}));r=h.getresponse();b=r.read();hs=dict(r.getheaders());h.close();timings.append({'path':path,'status':r.status,'wall_s':time.perf_counter()-start});assert r.status in expected,(r.status,b[:1000]);return b,hs
 def read(path='/api/observation'):return json.loads(request(path)[0])
 def wait(pred):
  deadline=time.monotonic()+120
  while time.monotonic()<deadline:
   s=read();assert s['phase']!='error',s.get('error')
   if pred(s):return s
   time.sleep(.03)
  raise AssertionError('Readiness timeout')
 def command(action,args,status='applied'):
  s=read();rid=uuid.uuid4().hex;request('/api/command',{'session_id':s['session_id'],'request_id':rid,'action':action,'args':args});s=wait(lambda s:any(e['request_id']==rid for e in s['events']));event=next(e for e in s['events'] if e['request_id']==rid);assert event['status']==status,event;return s
 try:
  s=wait(lambda s:s['phase']=='paused');assert 'error' not in s['plant']['lineage_observations'],s['plant']['lineage_observations'];catalog=read('/api/lineage-registry');assert len(catalog['lineage_stages'])==3;assert catalog['lineage_stages'][-1]['proof_rows']==1962;assert len(catalog['lineage_stages'][-1]['rebound_names'])==3
  assert catalog['denominators']['retained_baseline_mesh_records']==1890;assert s['plant']['internal_systems']['geometry_identity_match'] is False
  initial=read('/api/four-door-fans');assert initial['inventory']=={'rotating_axes':6,'sliding_axes':1,'native_added_coordinates':7,'native_nv':82,'native_nq':94};assert len(initial['fans'])==4
  assert all(not r['available'] for r in initial['observations'] if r['requires_accepted_interval'])
  assert s['command_registry']['four_door_fan']['args']['key']['type']=='string';assert ctl.plant.command_registry['four_door_fan']['args']['key']['type']=='enum'
  html,headers=request('/');assert b'four-fan-panel' in html and b'/fan82.js' in html and b'lineage-panel' in html;assert "default-src 'none'" in headers['Content-Security-Policy']
  assert b'cad_doors_four_fans' in request('/truth-evidence.js')[0];request('/api/four-door-fans',headers={'Origin':'https://example.invalid'},expected=(403,))
  before=read('/api/checkpoint');lease=ctl.last_client;read('/api/four-door-fans');read('/api/phase6-observations');assert ctl.last_client==lease and read('/api/checkpoint')==before
  settings=dict(zip(initial['fans'],[(.2,False),(.4,False),(.6,False),(.8,True)]))
  for key,(v,failed) in settings.items():command('four_door_fan',{'key':key,'voltage_fraction':v,'power_failed':failed})
  for args in [{'key':'bogus','voltage_fraction':.5,'power_failed':False},{'key':'all','voltage_fraction':1.2,'power_failed':False},{'key':'all','voltage_fraction':True,'power_failed':False}]:
   s=read();request('/api/command',{'session_id':s['session_id'],'request_id':uuid.uuid4().hex,'action':'four_door_fan','args':args},expected=(400,))
  command('step',{'steps':1});accepted=wait(lambda s:s['plant']['steps']==1 and not s['single_steps_remaining']);cp=read('/api/checkpoint');panel=read('/api/four-door-fans');assert all(r['available'] for r in panel['observations']);assert panel['extra_work_columns_counted_once'] and len(panel['extra_work_columns_J'])==3
  currents={k:next(r['value'] for r in panel['observations'] if r['fan']==k and r['field']=='accepted_motor_current_A') for k in settings};assert list(currents.values())[:3]==sorted(list(currents.values())[:3]);assert list(currents.values())[-1]==0
  aux=read('/api/phase6-observations');assert len(aux['observations'])==14 and all(r['available'] for r in aux['observations']);assert len(aux['mount_sensor_records'])==6
  # Invalid plant identity remains invalid even after both checksum layers resigned.
  from controller import digest
  bad=copy.deepcopy(cp);bad['plant']['identity_sha256']='0'*64;bad['plant']['sha256']=digest({k:v for k,v in bad['plant'].items() if k!='sha256'});bad['sha256']=digest({k:v for k,v in bad.items() if k!='sha256'})
  command('restore_checkpoint',{'checkpoint':bad},'rejected');assert read('/api/checkpoint')==cp;assert read('/api/lineage-registry')['catalogue_sha256']==catalog['catalogue_sha256']
  # A valid79 external checkpoint cannot be restored into82 even when wrapped anew.
  raw79=F.phase7().door_factory(c['child'])();wrong=copy.deepcopy(cp);wrong['plant']=raw79.checkpoint();wrong['sha256']=digest({k:v for k,v in wrong.items() if k!='sha256'});command('restore_checkpoint',{'checkpoint':wrong},'rejected');assert read('/api/checkpoint')==cp
  for _ in range(2):read('/api/four-door-fans');read('/api/lineage-observation')
  assert read('/api/checkpoint')==cp
  command('step',{'steps':1});advanced=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining']);restored=command('restore_checkpoint',{'checkpoint':cp});assert read('/api/checkpoint')==cp
  for key in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert restored['plant'][key]==accepted['plant'][key],key
  command('step',{'steps':1});continued=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining'])
  for key in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert continued['plant'][key]==advanced['plant'][key],key
  final=read('/api/four-door-fans');binding=read('/api/binding');o=continued['plant']['lineage_observations'];assert o['identity_match'];assert o['denominators']['legacy']['reasons']['UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS']==687
  for name,value in [('current_state.json',continued),('current_binding.json',binding),('four_fans.json',final),('http_timings.json',timings)]: (ROOT/'evidence'/name).write_text(json.dumps(value,ensure_ascii=False,separators=(',',':')))
  return {'ok':True,'accepted_integrator_calls':3,'unique_final_steps':2,'short_scope_not_long_convergence':True,'model_identity_sha256':continued['plant']['model_identity_sha256'],'geometry_sha256':continued['plant']['source_geometry_sha256'],'catalogue_sha256':catalog['catalogue_sha256'],'lineage_stage_denominators':[{'retained':len(x['retained_names']),'removed':len(x['removed_names']),'added':len(x['added_names']),'rebound':len(x.get('rebound_names',[]))} for x in catalog['lineage_stages']],'signals':o['denominators'],'mesh_counts_not_hardware_coverage':catalog['denominators'],'inventory':final['inventory'],'first_step_four_currents_A':currents,'wrong_identity_and_valid79_checkpoint_rejected':True,'bad_restore_preserves_active_catalogue':True,'exact_checkpoint_and_continuation':True,'readonly_HTTP_and_native_snapshot_preserve_checkpoint':True,'schema_normalization_only_detached':True,'different_voltages_and_fault':True,'native_force_and_parent_reaction_available':True,'additional_work_columns_counted_once':True,'browser_GPU_tested':False,'known_restore_diagnostic_gap':'Re-signed last.air_speed_rad_s and last.relative_speed_rad_s not independently checked in frozen82; do not drive physics; exposed current speed recomputed from named native state'}
 finally:server.shutdown();server.server_close();thread.join();ctl.close()
if __name__=='__main__':print(json.dumps(run(),ensure_ascii=False,indent=2))
