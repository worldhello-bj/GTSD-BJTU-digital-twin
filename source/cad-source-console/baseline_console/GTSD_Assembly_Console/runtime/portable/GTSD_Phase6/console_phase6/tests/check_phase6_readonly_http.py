"""Read-only evidence/candidate HTTP route checks with a fake immutable owner.

No model construction or integration. Source handler/CSP/origin rules are real.
"""
import copy
from http.client import HTTPConnection
import json
from pathlib import Path
import sys
import threading

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import phase6_server as S
from truth_overlay import EvidenceStore
from candidate_review import EncoderReview, BodyReview, PantographReview


def run():
    config=S.load_config(ROOT/'phase6_config.json');base=S.source_modules(config)
    class Owner:
        def __init__(self):self.reads=[]
        def snapshot(self,*,heartbeat=True):
            self.reads.append(heartbeat)
            return {'plant':{'source_geometry_sha256':'7d925267cbfb18d39e64041ea851fc13db2e3a13f6d403eab386235f741d8f43'},'phase':'paused','session_id':'readonly-test','snapshot_age_s':0}
    owner=Owner();server=base.ConsoleServer(0,owner)
    server.RequestHandlerClass=S.build_handler(base,EvidenceStore(config['evidence_root'],ROOT/'design_candidates.json'),EncoderReview(config.get('encoder_candidate_root')),BodyReview(config.get('body_candidate_root')),PantographReview(config.get('pantograph_candidate_root')))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    def get(path,headers=None):
        c=HTTPConnection('127.0.0.1',server.server_port,timeout=10);c.request('GET',path,headers={} if headers is None else headers);r=c.getresponse();body=r.read();result=(r.status,body,dict(r.getheaders()));c.close();return result
    try:
        code,body,headers=get('/api/truth-evidence');assert code==200
        evidence=json.loads(body);assert len(evidence['independent_design_candidates'])==3
        assert evidence['encoder_independent_review']['available']
        assert evidence['body_independent_review']['available']
        assert evidence['pantograph_independent_review']['available']
        assert all(row['current_solver_binding'] is False for row in evidence['independent_design_candidates'])
        code,body,_=get('/api/candidate-encoder');assert code==200 and json.loads(body)['enabled_by_default'] is False
        code,body,_=get('/candidate-review/encoder');assert code==200 and b'/candidate-assets/encoder/outputs/render_comparison.png' in body
        code,body,headers=get('/candidate-assets/encoder/outputs/render_comparison.png');assert code==200 and body.startswith(b'\x89PNG\r\n\x1a\n')
        assert headers['Content-Type']=='image/png';assert "connect-src 'self'" in headers['Content-Security-Policy']
        assert get('/candidate-assets/encoder/../console/server.py')[0]==404
        assert get('/candidate-review/encoder',{'Origin':'https://external.invalid'})[0]==403
        assert get('/api/candidate-encoder',{'Sec-Fetch-Site':'cross-site'})[0]==403
        code,body,_=get('/api/candidate-body');assert code==200 and json.loads(body)['current_solver_binding'] is False
        code,body,_=get('/candidate-review/body');assert code==200 and b'/candidate-assets/body/outputs/body_CAD_front.png' in body
        assert get('/candidate-assets/body/../inputs/vectors_zw_3E1B.json')[0]==404
        assert get('/candidate-review/body',{'Origin':'https://external.invalid'})[0]==403
        code,body,_=get('/api/candidate-pantograph');assert code==200 and json.loads(body)['allow_dynamics'] is False
        code,body,_=get('/candidate-review/pantograph');assert code==200 and b'LOCAL_INSPECTION_ONLY' in body
        assert get('/candidate-assets/pantograph/../scripts/build.py')[0]==404
        assert owner.reads and not any(owner.reads)
        return {'ok':True,'scope':'read-only routes with fake owner; real source HTTP/origin/CSP handling, no model constructed',
                'evidence_fields':evidence['counts']['field_entries'],'design_candidates':3,'encoder_default_off':True,
                'body_review_default_off':True,'pantograph_local_review_no_dynamics':True,'current_model_substitution':False,'candidate_render_bytes_verified':True,'path_traversal_rejected':True,
                'cross_origin_rejected':True,'heartbeats_sent':0,'browser_and_gpu':'not tested'}
    finally:server.shutdown();server.server_close();thread.join()


if __name__=='__main__':print(json.dumps(run(),ensure_ascii=False,indent=2))
