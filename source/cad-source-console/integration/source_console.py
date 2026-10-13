"""Loopback-only source-rail variant and latest CAD assembly console.

Frozen numerical/controller source remains unchanged. New source rail geometry
has its own exact checkpoint identity. No hardware or network-device interface.
"""
from pathlib import Path
import argparse,copy,gzip,hashlib,importlib,json,sys
from urllib.parse import urlsplit
from plant import raw_type,BASE,ROOT,PROJECT,VARIANT,sha

def evidence(mode):
    if mode not in ('source_rails','cad_seat_research'):raise ValueError('Unknown explicit geometry mode')
    return ROOT/('evidence_seats'if mode=='cad_seat_research'else'evidence')

def prepare_contract(cfg,initial,mode='source_rails'):
    mapping=copy.deepcopy(cfg['mapping']);old=copy.deepcopy(mapping['contract']['accepted_identity'])
    proof=json.loads((evidence(mode)/'initial_validation.json').read_text())
    if not proof['ok'] or initial['model_identity_sha256']!=proof['model_identity_sha256'] or initial['variant']!=proof['variant']:
        raise ValueError('New source-rail identity has not passed initial admission')
    for name,ref in mapping['contract']['reference_poses'].items():
        if initial['body_poses'].get(name)!=ref:raise ValueError('Changed initial body pose; per-mesh re-registration needed '+name)
    accepted=copy.deepcopy(old)
    for k in ('model_identity_sha256','source_geometry_sha256','variant','time_s','steps'):accepted[k]=initial[k]
    accepted.update(source_state_path='integration/'+evidence(mode).name+'/initial_state.json',source_geometry_path='inherited frozen82 geometry; source rails specified separately',source_binding_path='old per-mesh equality evidence preserved; new identity admitted by exact reference-pose check')
    mapping['contract']['accepted_identity']=accepted
    mapping['contract']['source_rail_variant_admission']={'old_accepted_identity':old,'new_acceptance':proof,'old_checkpoint_reuse_allowed':False,'old_mapping_rows_unchanged':True,'new_geometry_collision_scope':'Two source-aligned rail proxies only; not entire CAD scene','near_spring_force_geometry_updated':mode=='cad_seat_research'}
    addon=json.loads((PROJECT/'frame_track/registration_addendum.json').read_text())
    if addon['baseline']['accepted_solver_identity']!=old:raise ValueError('Reviewed registration from different old baseline')
    addon['baseline']['previous_accepted_solver_identity']=old
    addon['baseline']['accepted_solver_identity']=accepted
    addon['source_rail_variant_admission']=copy.deepcopy(mapping['contract']['source_rail_variant_admission'])
    bogie=json.loads((PROJECT/'frame_track/bogie_registration_addendum.json').read_text())
    if bogie['baseline']['accepted_solver_identity']!=old:raise ValueError('Bogie source baseline mismatch')
    bogie['baseline']['previous_accepted_solver_identity']=old;bogie['baseline']['accepted_solver_identity']=accepted
    bogie['source_rail_variant_admission']=copy.deepcopy(mapping['contract']['source_rail_variant_admission'])
    bogie['view_admission_options']={'enableUncalibratedCarrierVisuals':True,'enableUncalibratedCadSeatFit':mode=='cad_seat_research','referenceSitePoses':initial.get('native_spring_sites',{}),'referenceBodyPoses':mapping['contract']['reference_poses'],'cadSeatContractSha256':initial.get('cad_seat_application',{}).get('contract_sha256')}
    return mapping,addon,bogie

def display_factory(T,mode='source_rails'):
    proof=json.loads((evidence(mode)/'initial_validation.json').read_text())
    from mounted_display import display_type
    class Delegate:
        def __init__(self,raw):self._raw=raw
        def __getattr__(self,key):return getattr(self._raw,key)
        def snapshot(self):return self._raw.snapshot()
        def checkpoint(self):return self._raw.checkpoint()
        def restore(self,value):return self._raw.restore(value)
        def command(self,name,args):return self._raw.command(name,args)
        def step(self,n):return self._raw.step(n)
        @property
        def visual_resource_bytes(self):return self._raw.visual_resource_bytes()
    class Display(display_type(Delegate)):
        def snapshot(self):
            s=super().snapshot()
            if s['variant']!=proof['variant'] or s['model_identity_sha256']!=proof['model_identity_sha256']:raise ValueError('Wrong or unvalidated source rail identity')
            key=s['command_registry']['four_door_fan']['args']['key']
            if key.get('type')!='enum' or set(key)!={'type','values'}:raise ValueError('Unknown native fan command schema')
            s['command_registry']['four_door_fan']['args']['key']={'type':'string','enum':list(key['values'])}
            s['title']='新CAD来源轨道 · 82DOF整车研究控制台'
            s['subtitle']='两轨几何已更新 · 车辆其余碰撞/质量仍为未标定继承参数 · 非实物控制'
            if mode=='cad_seat_research':s['title']='CAD近侧弹簧座 · 来源轨道研究控制台';s['subtitle']='近侧2弹簧端点/力臂更新 · Y/刚接/弹簧率和质量未实测 · 非1:1认证'
            return s
    return lambda:Display(T())

def make_server(port=8778,mode='source_rails'):
    evidence(mode)
    if mode=='cad_seat_research':
        from seat_plant import raw_type as chosen_type
    else:chosen_type=raw_type
    T,cfg=chosen_type()
    # Verify the new reference identity before admitting any old visual mapping.
    initial=json.loads((evidence(mode)/'initial_state.json').read_text())
    mapping,addon,bogie=prepare_contract(cfg,initial,mode)
    import strict82_server as S
    P=S.fan_console().phase7().phase6_module()
    source=P.source_modules(cfg['strict_config']['base']['child']['base'])
    ctl=source.Controller(factory=display_factory(T,mode),resume=None)
    server=source.ConsoleServer(port,ctl);parent=server.RequestHandlerClass
    class Handler(parent):
        def file(self,path,mime,encoding=None):
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(path.stat().st_size));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Security-Policy',"default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'none'")
            if encoding:self.send_header('Content-Encoding',encoding)
            self.end_headers()
            try:
                with path.open('rb')as f:
                    for b in iter(lambda:f.read(65536),b''):self.wfile.write(b)
            except (BrokenPipeError,ConnectionResetError):pass
        def _send(self,code,data,content_type='application/json; charset=utf-8'):
            if urlsplit(self.path).path=='/'and code==200 and isinstance(data,bytes):
                data=data.replace(b'</header>',b'</header><nav style="padding:12px"><a href="/assembly">Latest source-registered CAD assembly</a> | source-aligned rails, remaining vehicle is uncalibrated research.</nav>',1)
            return super()._send(code,data,content_type)
        def do_GET(self):
            path=urlsplit(self.path).path
            if not(path=='/assembly'or path.startswith('/assembly/')or path.startswith('/api/assembly/')or path=='/source-dimensions'or path.startswith('/source-dimensions/')or path=='/api/source-dimensions/resource'or path=='/source-drive'or path.startswith('/source-drive/')or path=='/api/source-drive/resource'or path=='/source-fixture'or path.startswith('/source-fixture/')or path=='/api/source-fixture/resource'or path=='/source-library'or path.startswith('/source-library/')or path.startswith('/api/source-library/')or path=='/source-audit'or path.startswith('/source-audit/')or path=='/api/source-audit/index'):return super().do_GET()
            if not self._authorized():return
            dimroutes={'/source-dimensions':(ROOT/'dimensions_web/index.html','text/html; charset=utf-8'),'/source-dimensions/app.js':(ROOT/'dimensions_web/app.js','text/javascript; charset=utf-8'),'/source-dimensions/style.css':(ROOT/'dimensions_web/style.css','text/css; charset=utf-8'),'/api/source-dimensions/resource':(ROOT/'data/source_dimensions.json','application/json; charset=utf-8'),'/source-dimensions/native.blend':(PROJECT/'full_source_audit/bearing_envelopes/bearing_dimension_envelopes.blend','application/octet-stream'),'/source-dimensions/model.glb':(PROJECT/'full_source_audit/bearing_envelopes/bearing_envelopes.glb','model/gltf-binary')}
            dimroutes.update({'/source-drive':(ROOT/'drive_web/index.html','text/html; charset=utf-8'),'/source-drive/app.js':(ROOT/'drive_web/app.js','text/javascript; charset=utf-8'),'/api/source-drive/resource':(PROJECT/'full_source_audit/semantic_triage/drive_comparison_geometry.json','application/json; charset=utf-8')})
            dimroutes.update({'/source-fixture':(ROOT/'fixture_web/index.html','text/html; charset=utf-8'),'/source-fixture/app.js':(ROOT/'fixture_web/app.js','text/javascript; charset=utf-8'),'/api/source-fixture/resource':(PROJECT/'full_source_audit/semantic_triage/transformer_fixture_geometry/geometry.json','application/json; charset=utf-8')})
            dimroutes.update({'/source-library':(ROOT/'library_web/index.html','text/html; charset=utf-8'),'/source-library/app.js':(ROOT/'library_web/app.js','text/javascript; charset=utf-8'),'/source-library/oblique.svg':(PROJECT/'full_source_audit/oblique_library_review/native_reference.svg','image/svg+xml'),'/api/source-library/catalog':(ROOT/'data/source_library/catalog.json','application/json; charset=utf-8')})
            dimroutes.update({'/source-audit':(ROOT/'audit_web/index.html','text/html; charset=utf-8'),'/source-audit/app.js':(ROOT/'audit_web/app.js','text/javascript; charset=utf-8'),'/api/source-audit/index':(ROOT/'data/source_audit_index.json','application/json; charset=utf-8')})
            library_catalog=json.loads((ROOT/'data/source_library/catalog.json').read_text())
            for item in library_catalog['items']:
                if not item['id'].replace('_','').isalnum():raise ValueError('Unsafe library resource ID')
                dimroutes['/api/source-library/resource/'+item['id']]=(ROOT/'data/source_library'/(item['id']+'.json'),'application/json; charset=utf-8')
            if path in dimroutes:return self.file(*dimroutes[path])
            routes={'/assembly':('index.html','text/html; charset=utf-8'),'/assembly/':('index.html','text/html; charset=utf-8'),'/assembly/app.js':('app.js','text/javascript; charset=utf-8'),'/assembly/core.js':('core.js','text/javascript; charset=utf-8'),'/assembly/model.js':('model.js','text/javascript; charset=utf-8'),'/assembly/lineage.js':('lineage.js','text/javascript; charset=utf-8'),'/assembly/style.css':('style.css','text/css; charset=utf-8')}
            if path in routes:
                name,mime=routes[path];local=ROOT/'web'/name
                return self.file(local if local.exists()else BASE/'app/web'/name,mime)
            if path=='/assembly/registration.js':return self.file(PROJECT/'frame_track/registration.js','text/javascript; charset=utf-8')
            if path=='/assembly/bogie-registration.js':return self.file(PROJECT/'frame_track/bogie_registration.js','text/javascript; charset=utf-8')
            if path=='/assembly/source-overlays.js':return self.file(ROOT/'web/source-overlays.js','text/javascript; charset=utf-8')
            if path=='/assembly/source-replacements.js':return self.file(ROOT/'web/source-replacements.js','text/javascript; charset=utf-8')
            if path=='/api/assembly/source-replacements':return self.file(ROOT/'data/source_replacements.json','application/json; charset=utf-8')
            if path=='/api/assembly/source-overlays':return self.file(ROOT/'data/source_overlays.json','application/json; charset=utf-8')
            if path=='/api/assembly/environment-references':return self.file(PROJECT/'final_review/environment_reference_registration.json','application/json; charset=utf-8')
            if path=='/assembly/model.glb':return self.file(Path(cfg['assembly_gzip']),'model/gltf-binary','gzip')
            if path=='/assembly/extras.json':return self.file(Path(cfg['extras']),'application/json; charset=utf-8','gzip')
            if path=='/api/assembly/catalog':return self._send(200,cfg['catalogue'])
            if path=='/api/assembly/mapping':return self._send(200,mapping)
            if path=='/api/assembly/registration':return self._send(200,addon)
            if path=='/api/assembly/bogie-registration':return self._send(200,bogie)
            if path=='/api/assembly/runtime':
                state=ctl.snapshot(heartbeat=False);p=state.get('plant')or{}
                return self._send(200,{'schema':'gtsd-assembly-runtime-v1',**{k:state.get(k)for k in ('phase','session_id','snapshot_age_s','events','error','single_steps_remaining')},**{k:p.get(k)for k in ('model_identity_sha256','source_geometry_sha256','variant','time_s','steps','body_poses','cad_seat_application','native_spring_sites')},'renews_run_lease':False,'simulation_speed':state.get('performance'),'assembly_geometry_used_for_solver_collision':'SOURCE_RAIL_PROXY_BOUNDS_ONLY','cad_seat_force_geometry_enabled':mode=='cad_seat_research','hardware_connected':False})
            return self._send(404,{'error':'Unknown fixed assembly route'})
    server.RequestHandlerClass=Handler
    if str(PROJECT/'pantograph')not in sys.path:sys.path.insert(0,str(PROJECT/'pantograph'))
    from candidate_server import install_candidate_routes
    candidate_session=install_candidate_routes(server)
    return server,ctl,{'config':cfg,'mapping':mapping,'registration':addon,'bogie_registration':bogie,'mode':mode,'pantograph_candidate_session':candidate_session}

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8778);p.add_argument('--cad-seat-research',action='store_true',help='Explicit uncalibrated near-side CAD seat fit');args=p.parse_args()
    server,ctl,_=make_server(args.port,'cad_seat_research'if args.cad_seat_research else'source_rails');ctl.start();print('Cloud virtual console http://127.0.0.1:'+str(server.server_port)+'/assembly',flush=True)
    try:server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:pass
    finally:server.server_close();ctl.close()
if __name__=='__main__':main()
