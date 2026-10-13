"""Three actual steps including repeat continuation; no long physics protocol."""
from fixture import P,CONFIG,ROOT
import copy,json,threading,time,uuid,hashlib
from http.client import HTTPConnection

def run():
 cfg=copy.deepcopy(CONFIG);doors=cfg['variant']==P.DOORS_VARIANT
 if not doors:cfg['variant']='cad-body-mounted-research'
 cfg['base'].update(variant='cad-body-mounted-research',port=0);assert P.doctor(cfg)['ok']
 server,controller,context=P.make_server(cfg);thread=threading.Thread(target=server.serve_forever,daemon=True);controller.start();thread.start()
 def request(path,data=None,headers=None,expected=(200,202)):
  c=HTTPConnection('127.0.0.1',server.server_port,timeout=60);c.request('GET' if data is None else 'POST',path,None if data is None else json.dumps(data),headers or ({} if data is None else {'Content-Type':'application/json'}));r=c.getresponse();b=r.read();h=dict(r.getheaders());c.close();assert r.status in expected,(r.status,b[:500]);return b,h
 def state():return json.loads(request('/api/observation')[0])
 def wait(pred):
  deadline=time.monotonic()+90
  while time.monotonic()<deadline:
   s=state();assert s['phase']!='error',s.get('error')
   if pred(s):return s
   time.sleep(.03)
  raise AssertionError('Readiness timeout')
 def command(action,args,status="applied"):
  s=state();i=uuid.uuid4().hex;request('/api/command',{'session_id':s['session_id'],'request_id':i,'action':action,'args':args});s=wait(lambda s:any(e['request_id']==i for e in s['events']));assert next(e for e in s['events'] if e['request_id']==i)['status']==status;return s
 try:
  s=wait(lambda s:s['phase']=='paused');assert 'error' not in s['plant']['lineage_observations'],s['plant']['lineage_observations'];old=s['plant']['internal_systems'];assert old['geometry_identity_match'] is False
  catalog=json.loads(request('/api/lineage-registry')[0]);assert catalog['denominators']['retained_baseline_mesh_records']==(1890 if doors else 1940)
  if doors:
   assert len(catalog['lineage_stages'])==2 and catalog['lineage_stages'][1]['proof_rows']==1906
   assert s['plant']['variant']=='cad_doors_mounted' and s['plant']['telemetry']['cad_doors']['door_contact_enabled'] is False
   assert all('197 mm' not in x for x in s['plant']['cad_migration_display_limits'])
   aux=json.loads(request('/api/phase6-observations')[0]);assert aux['actual_variant']=='cad_doors_mounted' and all(x['available'] for x in aux['observations'])
  html,h=request('/');assert b'lineage-panel' in html and b'/lineage-client.js' in html;assert "default-src 'none'" in h['Content-Security-Policy']
  app=request('/app.js')[0];assert b'GTSDLineageView' in app and b'gtsd-lineage-selection' in app
  request('/api/lineage-registry',headers={'Origin':'https://example.invalid'},expected=(403,))
  command('step',{'steps':1});accepted=wait(lambda s:s['plant']['steps']==1 and not s['single_steps_remaining']);checkpoint=json.loads(request('/api/checkpoint')[0])
  bad=copy.deepcopy(checkpoint);bad['plant']['sha256']='0'*64
  from controller import digest
  bad['sha256']=digest({k:v for k,v in bad.items() if k!='sha256'})
  command('restore_checkpoint',{'checkpoint':bad},status='rejected')
  assert json.loads(request('/api/checkpoint')[0])==checkpoint
  assert json.loads(request('/api/lineage-registry')[0])['catalogue_sha256']==catalog['catalogue_sha256']
  lease=controller.last_client
  for _ in range(3):o=json.loads(request('/api/lineage-observation')[0])
  assert controller.last_client==lease and json.loads(request('/api/checkpoint')[0])==checkpoint
  assert o['identity_match'] and o['denominators']['legacy']['available']>850 and o['denominators']['legacy']['reasons']['UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS']==687
  command('step',{'steps':1});advanced=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining'])
  restored=command('restore_checkpoint',{'checkpoint':checkpoint});assert json.loads(request('/api/checkpoint')[0])==checkpoint
  for k in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert restored['plant'][k]==accepted['plant'][k],k
  command('step',{'steps':1});continued=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining'])
  for k in ('qpos','qvel','telemetry','solver_proxies','lineage_observations'):assert continued['plant'][k]==advanced['plant'][k],k
  binding=json.loads(request('/api/binding')[0]);(ROOT/('evidence/doors_state.json' if doors else 'evidence/current_state.json')).write_text(json.dumps(continued,ensure_ascii=False,separators=(',',':')));(ROOT/('evidence/doors_binding.json' if doors else 'evidence/current_binding.json')).write_text(json.dumps(binding,ensure_ascii=False,separators=(',',':')))
  return {'ok':True,'scope':('DOORS79' if doors else 'CAD79')+', 3 accepted integration calls including repeated continuation; not long convergence','legacy_denominator':o['denominators']['legacy'],'added_denominator':o['denominators']['added_named'],'mesh_denominators':catalog['denominators'],'old_registry_geometry_guard_unchanged':True,'new_lineage_identity_match':True,'exact_checkpoint_restore':True,'bad_candidate_restore_preserves_active_catalogue':True,'exact_continuation_with_lineage_observations':True,'readonly_requests_do_not_renew_lease':True,'cross_origin_rejected':True,'DOM_bridge_served':True,'catalogue_bytes':len(json.dumps(catalog).encode()),'catalogue_sha256':catalog['catalogue_sha256'],'model_identity_sha256':continued['plant']['model_identity_sha256'],'geometry_sha256':binding['source_geometry_sha256'],'browser_GPU_acceptance':False}
 finally:server.shutdown();server.server_close();thread.join();controller.close()
if __name__=='__main__':print(json.dumps(run(),ensure_ascii=False,indent=2))
