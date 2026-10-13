#!/usr/bin/env python3
"""Standalone Phase7 semantic-lineage layer; all earlier releases read-only."""
from __future__ import annotations
import argparse,copy,hashlib,importlib,json,math,os,sys,weakref
from pathlib import Path
from urllib.parse import urlsplit
from lineage_registry import LineageRegistry,verify_stage
ROOT=Path(__file__).resolve().parent
DOORS_VARIANT='cad-doors-mounted-research'

def strict(body):
    def obj(pairs):
        result={}
        for k,v in pairs:
            if k in result:raise ValueError('Duplicate JSON key')
            result[k]=v
        return result
    def invalid(v):raise ValueError('Invalid JSON constant: '+v)
    value=json.loads(body,object_pairs_hook=obj,parse_constant=invalid)
    def finite(x):
        if type(x) in (int,float) and not math.isfinite(x):raise ValueError('Nonfinite JSON number')
        if isinstance(x,dict):
            for v in x.values():finite(v)
        elif isinstance(x,list):
            for v in x:finite(v)
    finite(value);return value
def admitted(path,key):
    body=Path(path).read_bytes();expected=strict((ROOT/'accepted_dependencies.json').read_bytes()).get(key)
    if expected is None or hashlib.sha256(body).hexdigest()!=expected:raise ValueError('Unknown reviewed dependency: '+key)
    return body

def load_config(path=None):
    path=Path(path or os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'phase7_config.json').resolve()
    if path.stat().st_size>65536:raise ValueError('Phase7 configuration exceeds 64KiB')
    raw=strict(path.read_bytes());keys={'schema','phase6_console_root','phase6_config','variant','cad_doors_root','door_candidate_root'}
    if set(raw)!=keys or raw.get('schema')!='gtsd-phase7-console-config-v1':raise ValueError('Exact Phase7 schema/fields required')
    def resolve(v):
        if type(v)is not str or not v.strip():raise ValueError('Nonempty local path required')
        return (path.parent/Path(v).expanduser()).resolve()
    phase6=resolve(raw['phase6_console_root']);freeze=strict(admitted(phase6/'SOURCE_FREEZE_CONSOLE_PHASE6.json','phase6_source_freeze'))
    for relative,digest in freeze['source_sha256'].items():
        if hashlib.sha256((phase6/relative).read_bytes()).hexdigest()!=digest:raise ValueError('Frozen Phase6 code changed: '+relative)
    sys.dont_write_bytecode=True
    if str(phase6) not in sys.path:sys.path.insert(0,str(phase6))
    S=importlib.import_module('phase6_server')
    if Path(S.__file__).resolve().parent!=phase6:raise ValueError('Conflicting already-loaded Phase6 console')
    base=S.load_config(resolve(raw['phase6_config']))
    if raw['variant'] not in S.ALLOWED+(DOORS_VARIANT,):raise ValueError('Unknown explicit model variant')
    base['variant']=raw['variant'] if raw['variant']!=DOORS_VARIANT else S.CAD_VARIANT
    verify_phase5_semantics(base)
    admitted(Path(base['source_root'])/'internal_registry/registry.json','legacy_registry')
    admitted(Path(base['source_root'])/'sensors/observe.py','native_observer')
    if raw['variant'] in (S.CAD_VARIANT,DOORS_VARIANT):admitted(Path(base['cad_body_root'])/'evidence/retained_mesh_identity.json','cad_retained_map')
    return {'path':str(path),'phase6_root':str(phase6),'base':base,'variant':raw['variant'],'cad_doors_root':None if raw['cad_doors_root'] is None else str(resolve(raw['cad_doors_root'])),'door_candidate_root':None if raw['door_candidate_root'] is None else str(resolve(raw['door_candidate_root']))}

def phase6_module():return importlib.import_module('phase6_server')

def verify_baseline_bytes(freeze,source_root,body):
    data=strict(body);hashes=data['source_sha256'];digest=hashlib.sha256(json.dumps(hashes,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    if digest!=freeze['phase5_dependency_sha256']:raise ValueError('Unknown numerical dependency manifest identity')
    for name,expected in hashes.items():
        if hashlib.sha256((Path(source_root)/name).read_bytes()).hexdigest()!=expected:raise ValueError('Unknown numerical source: '+name)
    return len(hashes)

def verify_phase5_semantics(base):
    freeze=phase6_module().verify_auxiliary_freeze(base['auxiliary_root'])
    return verify_baseline_bytes(freeze,base['source_root'],(Path(base['auxiliary_root'])/'PHASE6_READONLY_BASELINE.json').read_bytes())

def verify_doors(config):
    if config.get('cad_doors_root') is None or config.get('door_candidate_root') is None:raise ValueError('Explicit door physics and geometry roots required')
    root=Path(config['cad_doors_root']);freeze=strict(admitted(root/'SOURCE_FREEZE.json','cad_doors_source_freeze'))
    expected={'cad_doors/'+n for n in ('__init__.py','bootstrap.py','geometry.py','moments.py','model.py','plant.py')}|{'READONLY_DEPENDENCIES.json','PHYSICAL_INTERFACE_CONTRACT.json'}
    if freeze.get('schema')!='gtsd-cad-doors-source-freeze-v1' or set(freeze.get('physics_source_sha256',{}))!=expected:raise ValueError('Unknown door source-freeze shape')
    for name,h in freeze['physics_source_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError('Frozen door source changed: '+name)
    admitted(root/'evidence/retained_mesh_identity.json','cad_doors_retained_map')
    return freeze

def door_factory(config):
    verify_doors(config)
    for key,path in {'GTSD_DOOR_BODY_ROOT':config['base']['cad_body_root'],'GTSD_DOOR_DOORS_ROOT':config['door_candidate_root']}.items():
        if key in os.environ and Path(os.environ[key]).resolve()!=Path(path).resolve():raise ValueError('Conflicting door dependency environment: '+key)
        os.environ[key]=path
    if config['cad_doors_root'] not in sys.path:sys.path.insert(0,config['cad_doors_root'])
    from cad_doors.plant import CadDoorsPlant
    from mounted_display import display_type
    class DoorDisplay(display_type(CadDoorsPlant)):
        @property
        def visual_resource_bytes(self):return CadDoorsPlant.visual_resource_bytes(self)
        def snapshot(self):
            value=super().snapshot()
            if value.get('variant')=='cad_doors_mounted':
                value['title']='CAD端框与四门原生研究控制台'
                value['subtitle']='显式79DOF · CAD开口/候选门安装 · 理想空间锁扣 · 仅1个独立门风机转子'
                value['cad_migration_display_limits']=['四门各1个风机外形，但仅1个原生独立转子；其余3个没有独立电机/转子动力学。','新端框、门、铰轴与锁扣为显式研究迁移；间隙、铰深、硬件安装及质量仍未实测标定。','门周边/端框局部接触默认关闭；不是全门板/风机罩/锁具碰撞覆盖。','叶侧世界XYZ锁扣约束力与框侧反力来自独立endpoint mj_forward；不得计作额外功输入。']
            return value
    return DoorDisplay


def build_registry(config,plant):
    S=phase6_module();base=config['base'];source=Path(base['source_root']);baseline=(source/'web/geometry.json').read_bytes();current=getattr(plant,'visual_resource_bytes',None)
    if callable(current):current=current()
    current=baseline if current is None else current
    if type(current)is not bytes:raise ValueError('Native visual resource must be exact bytes')
    stages=[]
    if base['variant']==S.CAD_VARIANT:
        from cad_body.geometry import visual_resource_bytes
        cad_bytes=visual_resource_bytes();proof=strict(admitted(Path(base['cad_body_root'])/'evidence/retained_mesh_identity.json','cad_retained_map'))
        overlay=json.loads(cad_bytes)['cad_body_overlay'];manifest=json.loads((Path(base['cad_body_root'])/'replacement_manifest.json').read_text())
        stages.append(verify_stage(baseline,cad_bytes,proof,manifest['replaced_source_mesh_names'],overlay['added_mesh_names']))
        if config['variant']==DOORS_VARIANT:
            proof=strict(admitted(Path(config['cad_doors_root'])/'evidence/retained_mesh_identity.json','cad_doors_retained_map'))
            overlay=json.loads(current)['cad_door_overlay']
            prior_names={m['name'] for m in json.loads(cad_bytes)['meshes']}
            absent=set(overlay['removed_mesh_names'])-prior_names
            if absent!=set(overlay['previously_removed_control_mesh_names']) or not absent<=set(stages[0]['removed_names']):raise ValueError('Unproven previously removed door-control source')
            stage=verify_stage(cad_bytes,current,proof,[n for n in overlay['removed_mesh_names'] if n in prior_names],overlay['added_mesh_names'])
            stage['previously_removed_control_names']=sorted(absent);stages.append(stage)
    snapshot=plant.snapshot();legacy=strict(admitted(source/'internal_registry/registry.json','legacy_registry'))
    return LineageRegistry(legacy,baseline,current,plant.binding,snapshot['model_identity_sha256'],snapshot['variant'],stages)

def doctor(config,check_bind=True):
    S=phase6_module();result=S.doctor(config['base'],check_bind=check_bind)
    result['schema']='gtsd-phase7-lineage-console-preflight-v1';result['explicit_variant']=config['variant'];result['legacy_expected_hash_modified']=False
    if config['variant']==DOORS_VARIANT:
        try:result['cad_doors_source_freeze']=verify_doors(config)
        except (ValueError,OSError) as exc:result['ok']=False;result['errors'].append({'code':'door_source_admission','message':str(exc)})
    return result

def bridge_app(body):
    """Reviewed in-memory extension; original file remains byte-identical."""
    text=body.decode();anchor='controls();poll();\n})();'
    if text.count(anchor)!=1:raise ValueError('Frozen app bridge anchor changed')
    bridge='''window.GTSDLineageView=Object.freeze({
select:ids=>choose(Array.isArray(ids)?ids:[]),
selection:()=>[...selectedIds],
visible:()=>view?.resource?.meshes.filter(m=>G.visibleMesh(m,view.view)).map(m=>m.id)||[],
pick:(x,y,w,h)=>window.GTSDLineage?.pick(view,x,y,w,h,G)||null
});
'''
    text=text.replace(anchor,bridge+anchor)
    choose_anchor="$('selection-status').textContent='已高亮 '+selectedIds.length+' 个已绑定源网格。视角/高亮不改变求解姿态。';}"
    if text.count(choose_anchor)!=1:raise ValueError('Frozen selection bridge anchor changed')
    text=text.replace(choose_anchor,choose_anchor[:-1]+"window.dispatchEvent(new CustomEvent('gtsd-lineage-selection',{detail:{ids:[...selectedIds]}}));}")
    return text.encode()

def door_auxiliary_observations(plant):
    from lineage_registry import get,valid_value
    rows=[]
    for name in ('compressor_rotor','compressor_fan_rotor','bay_fan_rotor'):
        for field,unit in (('angle_rad','rad'),('omega_rad_s','rad/s')):
            path=['telemetry','auxiliary_mount','native',field,name];found,value=get(plant,path);valid=found and valid_value(value,unit)
            rows.append({'id':name+'.'+field,'unit':unit,'value':value if valid else None,'available':valid,'availability':'NATIVE_NAMED_SOLVER_NOT_HARDWARE' if valid else 'MISSING_NAMED_PATH','source_path':path})
    for field,unit in (('piston_displacement_m','m'),('piston_velocity_m_s','m/s')):
        path=['telemetry','auxiliary_mount','native',field];found,value=get(plant,path);valid=found and valid_value(value,unit)
        rows.append({'id':'compressor.'+field,'unit':unit,'value':value if valid else None,'available':valid,'availability':'NATIVE_NAMED_SOLVER_NOT_HARDWARE' if valid else 'MISSING_NAMED_PATH','source_path':path})
    telemetry=plant.get('telemetry',{});aux=telemetry.get('auxiliary_mount',{})
    return {'schema':'gtsd-phase6-named-auxiliary-observations-v1','mounted_variant':True,'actual_variant':plant.get('variant'),'sample_time_s':plant.get('time_s'),'observations':rows,
            'mount_sensor_records':[{'mount':k,'record':v,'quality':'NATIVE_VIRTUAL_SENSOR_NOT_HARDWARE'} for k,v in (aux.get('last_sensor') or {}).items()] if plant.get('steps',0) else [],
            'world_endpoint_observations':telemetry.get('auxiliary_world_observations'), 'cad_migration_display_limits':plant.get('cad_migration_display_limits',[]),'renews_run_lease':False,'hardware_connected':False}

def build_handler(parent,context,config):
    class Handler(parent):
        def _send(self,code,data,content_type='application/json; charset=utf-8'):
            if urlsplit(self.path).path=='/api/truth-evidence' and type(data)is dict and config['variant']==DOORS_VARIANT:
                data=copy.deepcopy(data);data.setdefault('current_solver_context',{})['cad_binding_status']='EXPLICIT_CAD_BODY_AND_DOOR_RESEARCH_MIGRATION_NOT_AS_BUILT'
                for candidate in data.get('independent_design_candidates',[]):
                    if candidate.get('id')=='BODY-CAD-CROSS-SECTION-CANDIDATE':candidate.update(status='CAD_BODY_SOURCE_DERIVED_IN_CURRENT_DOOR_BRANCH',current_solver_derived_usage=True,current_composed_geometry_sha256=data.get('current_running_geometry_identity'))
            if urlsplit(self.path).path=='/' and code==200 and isinstance(data,bytes):
                html=data.decode();html=html.replace('</head>','<link rel="stylesheet" href="/lineage.css"></head>')
                html=html.replace('<section id="truth-evidence-panel"','<section id="lineage-panel" class="panel" aria-label="新几何谱系与当前测点"></section><section id="truth-evidence-panel"')
                html=html.replace('</body>','<script src="/lineage.js"></script><script src="/lineage-client.js"></script></body>');data=html.encode()
            return super()._send(code,data,content_type)
        def do_GET(self):
            path=urlsplit(self.path).path
            if path not in ('/api/lineage-registry','/api/lineage-observation','/app.js','/lineage.js','/lineage-client.js','/lineage.css','/truth-evidence.js','/api/phase6-observations'):return super().do_GET()
            if not self._authorized():return
            try:
                if path=='/truth-evidence.js':
                    body=(Path(config['phase6_root'])/'phase6_web/truth-evidence.js').read_text()
                    body=body.replace("value.current_solver_context?.variant==='cad_body_mounted'","['cad_body_mounted','cad_doors_mounted'].includes(value.current_solver_context?.variant)")
                    body=body.replace('门、辅机安装等仍待迁移','部分安装仍待核验，端门迁移边界以当前模型说明为准')
                    return self._send(200,body.encode(),'text/javascript; charset=utf-8')
                if path=='/api/phase6-observations':
                    if config['variant']!=DOORS_VARIANT:return super().do_GET()
                    state=self.server.controller.snapshot(heartbeat=False);plant=state.get('plant') or {}
                    if plant.get('variant')!='cad_doors_mounted':raise ValueError('Explicit CAD door plant is not ready')
                    return self._send(200,dict(door_auxiliary_observations(plant),phase=state.get('phase'),session_id=state.get('session_id'),snapshot_age_s=state.get('snapshot_age_s')))
                if path=='/app.js':return self._send(200,bridge_app((Path(config['base']['source_root'])/'console/web/app.js').read_bytes()),'text/javascript; charset=utf-8')
                if path.startswith('/lineage'):
                    mime='text/css; charset=utf-8' if path.endswith('.css') else 'text/javascript; charset=utf-8'
                    return self._send(200,(ROOT/'web'/path.lstrip('/')).read_bytes(),mime)
                state=self.server.controller.snapshot(heartbeat=False)
                if path=='/api/lineage-observation':
                    value=(state.get('plant') or {}).get('lineage_observations')
                    if value is None:raise ValueError('Current named lineage observations not ready')
                    return self._send(200,dict(value,phase=state.get('phase'),session_id=state.get('session_id'),snapshot_age_s=state.get('snapshot_age_s')))
                reg=context.get('registry')
                if reg is None or not reg.augmented:raise ValueError('Lineage catalogue not ready')
                return self._send(200,reg.catalogue())
            except (ValueError,OSError) as exc:self._send(409,{'error':str(exc),'solver_changed':False})
    return Handler

def make_server(config):
    S=phase6_module()
    server,original=S.make_server(config['base']);factory0=door_factory(config) if config['variant']==DOORS_VARIANT else original.factory;context={};candidates=weakref.WeakKeyDictionary()
    def factory():
        plant=factory0();candidates[plant]=build_registry(config,plant);return plant
    class Controller(type(original)):
        def _state(self,snapshot):
            result=super()._state(snapshot)
            if result.get('plant') is not None:
                try:
                    registry=candidates[self.plant]
                    result['plant']['lineage_observations']=registry.observe(result['plant'])
                    context['registry']=registry
                except Exception as exc:result['plant']['lineage_observations']={'schema':'gtsd-semantic-lineage-observations-v1','identity_match':False,'observations':[],'error':str(exc),'calibrated':False,'hardware_connected':False}
            return result
    controller=Controller(factory=factory,resume=original.resume);server.controller=controller
    server.RequestHandlerClass=build_handler(server.RequestHandlerClass,context,config)
    return server,controller,context

def main():
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['doctor','serve']);parser.add_argument('--config',default=None);args=parser.parse_args()
    try:
        config=load_config(args.config);result=doctor(config)
        if args.action=='doctor' or not result['ok']:print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['ok'] else 2
        server,controller,_=make_server(config);controller.start();print(f'GTSD lineage console http://127.0.0.1:{server.server_port}',flush=True)
        try:server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt:pass
        finally:server.server_close();controller.close()
        return 0
    except Exception as exc:print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False));return 2
if __name__=='__main__':raise SystemExit(main())
