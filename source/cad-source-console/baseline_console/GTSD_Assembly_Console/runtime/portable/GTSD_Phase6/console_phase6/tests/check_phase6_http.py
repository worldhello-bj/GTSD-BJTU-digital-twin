"""Explicit short adapter check after auxiliary source freeze; only four steps.

Independent truth refresh uses a small private test copy of evidence. No frozen
source writes, browser file imports, or long numerical acceptance protocols.
"""
import copy
from http.client import HTTPConnection
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import phase6_server as S
import truth_overlay as T


def run():
    config=S.load_config(ROOT/'phase6_config.json')
    config.update(variant=S.VARIANT,port=0)
    report=S.doctor(config)
    assert report['ok'],report
    with tempfile.TemporaryDirectory(prefix='truth-http-',dir=ROOT/'evidence') as temp:
        for filename,_ in T.FILES.values():shutil.copyfile(Path(config['evidence_root'])/filename,Path(temp)/filename)
        config['evidence_root']=temp
        server,controller=S.make_server(config)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        controller.start();thread.start()
        def request(path,payload=None,headers=None):
            c=HTTPConnection('127.0.0.1',server.server_port,timeout=30)
            h={} if headers is None else headers
            if payload is not None:h['Content-Type']='application/json'
            c.request('GET' if payload is None else 'POST',path,None if payload is None else json.dumps(payload),h)
            r=c.getresponse();body=r.read();code=r.status;response_headers=dict(r.getheaders());c.close()
            if code not in (200,202):return code,body,response_headers
            return code,json.loads(body) if 'json' in response_headers.get('Content-Type','') else body,response_headers
        def state():
            code,value,_=request('/api/observation');assert code==200,value;assert value['phase']!='error',value.get('error');return value
        def wait(predicate):
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                value=state()
                if predicate(value):return value
                time.sleep(.015)
            raise AssertionError('short HTTP check timeout')
        def command(action,args):
            current=state();payload={'session_id':current['session_id'],'request_id':uuid.uuid4().hex,'action':action,'args':args}
            code,result,_=request('/api/command',payload);assert code==202,result
            accepted=wait(lambda s:any(e['request_id']==payload['request_id'] for e in s['events']))
            event=next(e for e in accepted['events'] if e['request_id']==payload['request_id']);assert event['status']=='applied',event
            return accepted,payload
        try:
            start=wait(lambda s:s['phase']=='paused')
            assert start['capabilities']['integrated_dof']==79
            assert start['capabilities']['hardware_control'] is False
            assert len(start['plant']['qpos'])==91 and len(start['plant']['qvel'])==79
            assert len(start['plant']['solver_proxies'])==21
            code,html,headers=request('/');assert code==200 and b'truth-evidence-panel' in html
            assert "connect-src 'self'" in headers['Content-Security-Policy']
            for asset in ('/truth-evidence.js','/truth-evidence-client.js','/truth-evidence.css'):
                assert request(asset)[0]==200
            assert request('/api/truth-evidence',headers={'Origin':'https://external.invalid'})[0]==403
            assert request('/api/phase6-observations',headers={'Sec-Fetch-Site':'cross-site'})[0]==403
            before=controller.export_checkpoint();last_client=controller.last_client
            code,truth1,_=request('/api/truth-evidence');assert code==200 and truth1['counts']['field_entries']==763
            assert truth1['geometry_identity_match'] is True
            code,aux,_=request('/api/phase6-observations');assert code==200 and all(o['available'] for o in aux['observations'])
            assert aux['mount_sensor_records']==[]
            f=Path(temp)/'independent_findings.json';d=json.loads(f.read_text());d['findings'].append({'id':'HTTP-REFRESH-TEST','scope':'evidence only'});f.write_text(json.dumps(d))
            code,truth2,_=request('/api/truth-evidence');assert code==200 and truth1['evidence_revision_sha256']!=truth2['evidence_revision_sha256']
            assert controller.last_client==last_client
            assert controller.export_checkpoint()==before
            command('fan',{'key':'bay_fan','voltage_fraction':.2,'power_failed':False})
            command('step',{'steps':3});accepted=wait(lambda s:s['plant']['steps']==3 and not s['single_steps_remaining'])
            checkpoint=controller.export_checkpoint()
            command('step',{'steps':1});advanced=wait(lambda s:s['plant']['steps']==4 and not s['single_steps_remaining'])
            restored,payload=command('restore_checkpoint',{'checkpoint':checkpoint})
            assert restored['phase']=='paused' and restored['plant']['steps']==3
            assert controller.export_checkpoint()==checkpoint
            for key in ('qpos','qvel','telemetry','solver_proxies'):assert restored['plant'][key]==accepted['plant'][key],key
            command('step',{'steps':1});continued=wait(lambda s:s['plant']['steps']==4 and not s['single_steps_remaining'])
            for key in ('qpos','qvel','telemetry','solver_proxies'):assert continued['plant'][key]==advanced['plant'][key],key
            code,aux,_=request('/api/phase6-observations');assert code==200
            assert len(aux['mount_sensor_records'])==3
            assert len(aux['world_endpoint_observations']['rows'])==3
            assert aux['world_endpoint_observations']['endpoint_time_s']==continued['plant']['time_s']
            for row in aux['world_endpoint_observations']['rows'].values():assert abs(row['world_minus_stator_axis_residual_rad_s'])<=1e-9
            assert all(o['coordinate_index_used'] is None for o in aux['observations'])
            assert any(abs(o['value'])>0 for o in aux['observations'] if o['available'])
            assert sum(continued['plant']['warning_counts'])==0
            # Read-only proxy wrapping must retain the exact physical checkpoint.
            from aux_mount.plant import MountedAuxiliaryPlant
            from mounted_display import make_mounted_display
            raw=MountedAuxiliaryPlant();display=make_mounted_display()
            assert raw.checkpoint()==display.checkpoint()
            (ROOT/'evidence/phase6_accepted_state.json').write_text(json.dumps(continued,ensure_ascii=False,separators=(',',':')))
            (ROOT/'evidence/phase6_accepted_binding.json').write_text(json.dumps(controller.binding,ensure_ascii=False,separators=(',',':')))
            display.snapshot()['solver_proxies'][0]['body_name']='caller mutation'
            assert raw.checkpoint()==display.checkpoint()
            return {'ok':True,'scope':'79DOF new adapter, three steps plus exact one-step continuation; five total step calls including repeated continuation, not a long physics protocol',
                    'dof':79,'nq':91,'dt_s':continued['plant']['dt_s'],'final_simulated_steps':4,'final_time_s':continued['plant']['time_s'],
                    'proxy_count':21,'named_auxiliary_observations':len(aux['observations']),'native_mount_sensor_records':len(aux['mount_sensor_records']),'world_endpoint_rotor_records':3,
                    'truth_field_entries':763,'evidence_refresh_changes_model_or_checkpoint':False,'evidence_requests_renew_lease':False,
                    'full_checkpoint_exact':True,'one_step_continuation_exact':True,'presentation_checkpoint_matches_raw_factory':True,
                    'cross_origin_rejected':True,'same_origin_csp_retained':True,'browser_gpu_downloads':'not tested',
                    'source_freeze':report['auxiliary_source_freeze'],'model_identity_sha256':continued['plant']['model_identity_sha256']}
        finally:
            server.shutdown();server.server_close();thread.join();controller.close()


if __name__=='__main__':print(json.dumps(run(),ensure_ascii=False,indent=2))
