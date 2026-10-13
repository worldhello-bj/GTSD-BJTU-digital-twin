"""Minimal independently selected CAD-body HTTP/identity/restore check.

No long protocol, no frozen source writes, no geometry file duplication.
"""
from http.client import HTTPConnection
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import phase6_server as S


def run():
    config=S.load_config(ROOT/'phase6_config.json');config.update(variant=S.CAD_VARIANT,port=0)
    preflight=S.doctor(config);assert preflight['ok'],preflight
    server,controller=S.make_server(config);thread=threading.Thread(target=server.serve_forever,daemon=True);controller.start();thread.start()
    def request(path,payload=None):
        c=HTTPConnection('127.0.0.1',server.server_port,timeout=30)
        c.request('GET' if payload is None else 'POST',path,None if payload is None else json.dumps(payload),{} if payload is None else {'Content-Type':'application/json'})
        r=c.getresponse();body=r.read();status=r.status;c.close();assert status in (200,202),(status,body[:1000]);return body
    def state():
        s=json.loads(request('/api/observation'));assert s['phase']!='error',s.get('error');return s
    def wait(predicate):
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            s=state()
            if predicate(s):return s
            time.sleep(.02)
        raise AssertionError('CAD console readiness timeout')
    def command(action,args):
        s=state();p={'session_id':s['session_id'],'request_id':uuid.uuid4().hex,'action':action,'args':args};request('/api/command',p)
        s=wait(lambda s:any(e['request_id']==p['request_id'] for e in s['events']))
        e=next(e for e in s['events'] if e['request_id']==p['request_id']);assert e['status']=='applied',e;return s
    try:
        initial=wait(lambda s:s['phase']=='paused');assert initial['capabilities']['integrated_dof']==79
        assert initial['plant']['variant']=='cad_body_mounted'
        geometry=request('/geometry.json');geometry_hash=hashlib.sha256(geometry).hexdigest()
        binding=json.loads(request('/api/binding'))
        assert geometry_hash==initial['plant']['source_geometry_sha256']==binding['source_geometry_sha256']
        assert geometry_hash==preflight['cad_body_source_freeze']['composed_geometry_sha256']
        assert geometry_hash!='7d925267cbfb18d39e64041ea851fc13db2e3a13f6d403eab386235f741d8f43'
        truth=json.loads(request('/api/truth-evidence'));assert truth['geometry_identity_match'] is False
        assert truth['current_solver_context']['cad_binding_status']=='EXPLICIT_PARTIAL_CAD_MIGRATION_NOT_FULL_AS_BUILT'
        assert initial['plant']['internal_systems']['geometry_identity_match'] is False
        indexed=[r for r in initial['plant']['internal_systems']['observations'] if r.get('source_path_used')]
        assert not indexed,'Old geometry registry must not be relabeled to pass'
        assert any('197' in text for text in initial['plant']['cad_migration_display_limits'])
        command('step',{'steps':2});accepted=wait(lambda s:s['plant']['steps']==2 and not s['single_steps_remaining'])
        checkpoint=json.loads(request('/api/checkpoint'))
        command('step',{'steps':1});advanced=wait(lambda s:s['plant']['steps']==3 and not s['single_steps_remaining'])
        restored=command('restore_checkpoint',{'checkpoint':checkpoint});assert restored['phase']=='paused'
        assert json.loads(request('/api/checkpoint'))==checkpoint
        for key in ('qpos','qvel','telemetry','solver_proxies'):assert restored['plant'][key]==accepted['plant'][key],key
        command('step',{'steps':1});continued=wait(lambda s:s['plant']['steps']==3 and not s['single_steps_remaining'])
        for key in ('qpos','qvel','telemetry','solver_proxies'):assert continued['plant'][key]==advanced['plant'][key],key
        aux=json.loads(request('/api/phase6-observations'));assert all(r['available'] for r in aux['observations'])
        assert len(aux['world_endpoint_observations']['rows'])==3
        from cad_body.plant import CadBodyPlant
        from mounted_display import make_cad_body_display
        raw=CadBodyPlant();display=make_cad_body_display();assert raw.checkpoint()==display.checkpoint()
        assert type(display.visual_resource_bytes) is bytes
        (ROOT/'evidence/cad_body_accepted_state.json').write_text(json.dumps(continued,ensure_ascii=False,separators=(',',':')))
        (ROOT/'evidence/cad_body_accepted_binding.json').write_text(json.dumps(binding,ensure_ascii=False,separators=(',',':')))
        return {'ok':True,'scope':'two steps plus exact one-step continuation; four actual integration calls including repeated continuation; not long physical acceptance',
                'dof':79,'final_steps':3,'time_s':continued['plant']['time_s'],'model_identity_sha256':continued['plant']['model_identity_sha256'],
                'composed_geometry_sha256':geometry_hash,'geometry_byte_count':len(geometry),'geometry_saved_as_duplicate':False,
                'snapshot_binding_bytes_hashes_match':True,'old_truth_registry_hash_rewritten':False,'old_registry_signals_unavailable':True,
                'partial_installation_warning_retained':True,'full_checkpoint_exact':True,'one_step_continuation_exact':True,
                'raw_and_display_checkpoints_equal':True,'world_auxiliary_observations_available':True,'browser_gpu_downloads':'not tested',
                'source_freeze':preflight['cad_body_source_freeze']}
    finally:server.shutdown();server.server_close();thread.join();controller.close()


if __name__=='__main__':print(json.dumps(run(),ensure_ascii=False,indent=2))
