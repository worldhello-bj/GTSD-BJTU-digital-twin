#!/usr/bin/env python3
"""Independent reviewed82 console increment; all prior releases stay read-only."""
from __future__ import annotations
import argparse,copy,hashlib,importlib,json,math,os,sys,weakref
from pathlib import Path
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parent
VARIANT='cad-doors-four-fans-research'
ACTUAL='cad_doors_four_fans'

def strict(body):
    def obj(pairs):
        v={}
        for k,x in pairs:
            if k in v:raise ValueError('Duplicate JSON key')
            v[k]=x
        return v
    def bad(x):raise ValueError('Nonfinite JSON')
    out=json.loads(body,object_pairs_hook=obj,parse_constant=bad)
    def check(v):
        if type(v)in(int,float) and not math.isfinite(v):raise ValueError('Nonfinite number')
        if isinstance(v,dict):
            for x in v.values():check(x)
        if isinstance(v,list):
            for x in v:check(x)
    check(out);return out

def admitted(path,key):
    data=Path(path).read_bytes();expected=strict((ROOT/'accepted_dependencies.json').read_bytes())['files'][key]
    if hashlib.sha256(data).hexdigest()!=expected:raise ValueError('Unknown reviewed dependency: '+key)
    return data

def phase7():return importlib.import_module('phase7_server')
def load_config(path=None):
    p=Path(path or os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console_config.json').resolve()
    if p.stat().st_size>65536:raise ValueError('Configuration exceeds64KiB')
    v=strict(p.read_bytes())
    if set(v)!={'schema','phase7_console_root','phase7_config','fan82_root','variant'} or v['schema']!='gtsd-console-fan82-config-v1':raise ValueError('Exact fan82 configuration schema required')
    def resolve(x):
        if type(x)is not str or not x.strip():raise ValueError('Nonempty local path required')
        return (p.parent/Path(x).expanduser()).resolve()
    root=resolve(v['phase7_console_root']);freeze=strict(admitted(root/'SOURCE_FREEZE_CONSOLE_PHASE7.json','phase7_source_freeze'))
    for name,digest in freeze['source_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:raise ValueError('Frozen Phase7 source changed: '+name)
    sys.dont_write_bytecode=True
    if str(root) not in sys.path:sys.path.insert(0,str(root))
    P=phase7()
    if Path(P.__file__).resolve().parent!=root:raise ValueError('Conflicting Phase7 module')
    c=P.load_config(resolve(v['phase7_config']));S=P.phase6_module()
    if v['variant'] not in S.ALLOWED+(P.DOORS_VARIANT,VARIANT):raise ValueError('Unknown explicit variant')
    c['variant']=P.DOORS_VARIANT if v['variant']==VARIANT else v['variant'];c['base']['variant']=S.CAD_VARIANT if c['variant']==P.DOORS_VARIANT else c['variant']
    out={'path':str(p),'phase7_root':str(root),'child':c,'fan82_root':str(resolve(v['fan82_root'])),'variant':v['variant']}
    if v['variant']==VARIANT:verify_fan82(out)
    return out

def verify_fan82(config):
    P=phase7();P.verify_doors(config['child']);root=Path(config['fan82_root'])
    if root!=Path(config['child']['cad_doors_root']):raise ValueError('Fan82 and frozen79 must share admitted physics root')
    f=strict(admitted(root/'FAN82_SOURCE_FREEZE.json','fan82_source_freeze'))
    expected={'fan82/'+n for n in ('__init__.py','bootstrap.py','model.py','geometry.py','runtime.py','plant.py')}|{'FAN82_INTERFACE_CONTRACT.json'}
    if f.get('schema')!='gtsd-fan82-source-freeze-v1' or set(f.get('physics_source_sha256',{}))!=expected:raise ValueError('Unknown82 freeze shape')
    if hashlib.sha256((root/'SOURCE_FREEZE.json').read_bytes()).hexdigest()!=f['frozen79_source_freeze_sha256']:raise ValueError('Unknown79 dependency')
    for name,h in f['physics_source_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=h:raise ValueError('Frozen82 source changed: '+name)
    for name,key in [('retained_mesh_identity.json','fan82_retained_map'),('native_ports.json','fan82_ports'),('fan_spec.json','fan82_spec')]:admitted(root/'evidence/fan82'/name,key)
    return f

def fan_factory(config):
    P=phase7();freeze=verify_fan82(config);P.phase6_module().source_modules(config['child']['base']);P.door_factory(config['child'])
    from fan82.plant import FourDoorFansPlant
    from mounted_display import display_type
    from fan82_view import normalize_command_registry,named_endpoint
    if Path(importlib.import_module('fan82.plant').__file__).resolve().parent.parent!=Path(config['fan82_root']):raise ValueError('Conflicting82 source module')
    class Display(display_type(FourDoorFansPlant)):
        @property
        def visual_resource_bytes(self):return FourDoorFansPlant.visual_resource_bytes(self)
        def snapshot(self):
            value=super().snapshot()
            # Base constructor can request a snapshot before the82 extension exists.
            if not hasattr(self,'extra_fans'):return value
            value=normalize_command_registry(value)
            if value['model_identity_sha256']!=freeze['validated_identity_sha256']['full'] or value['source_geometry_sha256']!=freeze['geometry_sha256'] or self.model.nq!=freeze['nq'] or self.model.nv!=freeze['nv']:raise ValueError('Unknown82 model/parameter/geometry identity')
            value['title']='四门独立风机82DOF研究控制台';value['subtitle']='显式82DOF · 6旋转轴+1活塞 · 四独立风机支路 · 参数与安装未标定'
            value['telemetry']['fan82_named_endpoint']=named_endpoint(self)
            value['cad_migration_display_limits']=['四门各有独立原生风机转轴；新增三台按同规格/平行理想供电假设。','端框/铰轴/理想空间锁扣及局部接触仍为未实测研究迁移。','锁扣为独立endpoint mj_forward观测；安装反力是原生虚拟传感器，不作额外能量输入。','辅机压力峰值/全状态收敛尚未完成；几何件数不表示硬件真值覆盖。']
            return value
    return Display

def verify_rebind(prior,current,proof,spec):
    from lineage_registry import mesh_map,sha
    old,_=mesh_map(strict(prior));new,_=mesh_map(strict(current))
    if proof['prior_geometry_sha256']!=sha(prior) or proof['geometry_sha256']!=sha(current) or set(old)!=set(new):raise ValueError('Rebind geometry identity/selector mismatch')
    expected={v['source_shroud_mesh']:v for v in spec['added_mounts'].values()}
    rows=proof['retained_exact_objects'];rebound=proof['rebound_only_objects']
    if len(rows)!=len({r['mesh_name'] for r in rows}) or len(rebound)!=len(expected) or {r['mesh_name'] for r in rebound}!=set(expected) or {r['mesh_name'] for r in rows}!=set(old)-set(expected):raise ValueError('Rebind proof denominator mismatch')
    digest=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    for row in rows+rebound:
        n=row['mesh_name'];a,b=old[n],new[n]
        if a['id']!=b['id'] or digest(a)!=row['prior_canonical_sha256'] or digest(b)!=row['new_canonical_sha256']:raise ValueError('Rebind record hash mismatch: '+n)
        if a.get('vertices')!=b.get('vertices') or a.get('indices')!=b.get('indices'):raise ValueError('Rebind changed shape')
        if n in expected:
            expect=expected[n];ac=copy.deepcopy(a);ac['physics_body_name']=expect['stator']
            if a.get('physics_body_name')!=expect['parent'] or b!=ac:raise ValueError('Unreviewed mesh change beyond stator binding')
        elif a!=b:raise ValueError('Retained record changed')
    return {'prior_geometry_sha256':sha(prior),'current_geometry_sha256':sha(current),'retained_names':sorted(set(old)-set(expected)),'rebound_names':sorted(expected),'removed_names':[],'added_names':[],'proof_rows':len(rows),'rebound_rows':len(rebound),'every_retained_mesh_checked':True,'all_vertices_indices_exact_unchanged':True,'canonical_format':'JSON sorted keys, compact separators, ensure_ascii=False, allow_nan=False; UTF-8'}

def build_registry(config,plant):
    P=phase7();from lineage_registry import LineageRegistry,verify_stage
    from fan82_view import fan_records
    from cad_body.geometry import visual_resource_bytes
    from cad_doors.geometry import resource
    c=config['child'];base=c['base'];source=Path(base['source_root']);baseline=(source/'web/geometry.json').read_bytes();cad=visual_resource_bytes();door=resource()[0];current=plant.visual_resource_bytes
    proof=P.strict(P.admitted(Path(base['cad_body_root'])/'evidence/retained_mesh_identity.json','cad_retained_map'));manifest=strict((Path(base['cad_body_root'])/'replacement_manifest.json').read_bytes())
    stages=[verify_stage(baseline,cad,proof,manifest['replaced_source_mesh_names'],strict(cad)['cad_body_overlay']['added_mesh_names'])]
    overlay=strict(door)['cad_door_overlay'];prior_names={m['name'] for m in strict(cad)['meshes']};absent=set(overlay['removed_mesh_names'])-prior_names
    if absent!=set(overlay['previously_removed_control_mesh_names']) or not absent<=set(stages[0]['removed_names']):raise ValueError('Unproven historical removal')
    stages.append(verify_stage(cad,door,P.strict(P.admitted(Path(c['cad_doors_root'])/'evidence/retained_mesh_identity.json','cad_doors_retained_map')),[n for n in overlay['removed_mesh_names'] if n in prior_names],overlay['added_mesh_names']))
    spec=strict(admitted(Path(config['fan82_root'])/'evidence/fan82/fan_spec.json','fan82_spec'));stages.append(verify_rebind(door,current,strict(admitted(Path(config['fan82_root'])/'evidence/fan82/retained_mesh_identity.json','fan82_retained_map')),spec))
    class Registry(LineageRegistry):
        def augment_named(self,snapshot):
            if self.augmented:return
            definitions,fields=fan_records(snapshot,spec)
            for row in fields:
                bodies=definitions[row['fan']]
                self.signals.append({'id':row['id'],'label':row['fan']+' '+row['field'],'unit':row['unit'],'shape':row['shape'],'path_tokens':[row['source_path']],'requires_accepted_interval':row['requires_accepted_interval'],'body_names':[bodies['rotor'],bodies['stator']],'geometry_mesh_ids':[],'definition_origin':'FAN82_NAMED_NATIVE_PORT','hardware_only':False,'evidence_level':'NATIVE_SOFTWARE_UNCALIBRATED','source_refs':[{'path':'.'.join(row['source_path']),'world_reaction_is_extra_work':False}]})
            super().augment_named(snapshot)
    snap=plant.snapshot();reg=Registry(P.strict(P.admitted(source/'internal_registry/registry.json','legacy_registry')),baseline,current,plant.binding,snap['model_identity_sha256'],snap['variant'],stages)
    for row in reg.meshes:
        if row['name'] in stages[-1]['rebound_names']:row.update(lineage='UNCHANGED_SHAPE_REBOUND_TO_NEW_NATIVE_STATOR',shape_prior_geometry_sha256=stages[-1]['prior_geometry_sha256'],mounting_status='SAME_SPEC_RESEARCH_NATIVE_STATOR_BINDING_NOT_AS_BUILT')
    return reg

def doctor(config,check_bind=True):
    out=phase7().doctor(config['child'],check_bind);out.update(schema='gtsd-console-fan82-preflight-v1',explicit_variant=config['variant'])
    if config['variant']==VARIANT:
        try:out['fan82_source_freeze']=verify_fan82(config)
        except (ValueError,OSError) as exc:out['ok']=False;out['errors'].append({'code':'fan82_admission','message':str(exc)})
    return out

def build_handler(parent,context,config):
    from fan82_view import panel_payload
    freeze=verify_fan82(config);spec=strict(admitted(Path(config['fan82_root'])/'evidence/fan82/fan_spec.json','fan82_spec'))
    class Handler(parent):
        def _send(self,code,data,content_type='application/json; charset=utf-8'):
            path=urlsplit(self.path).path
            if path=='/' and code==200 and isinstance(data,bytes):data=data.replace(b'</head>',b'<link rel="stylesheet" href="/fan82.css"></head>').replace(b'<section id="truth-evidence-panel"',b'<section id="four-fan-panel" class="panel"></section><section id="truth-evidence-panel"').replace(b'</body>',b'<script src="/fan82.js"></script></body>')
            return super()._send(code,data,content_type)
        def do_GET(self):
            path=urlsplit(self.path).path
            if path not in ('/api/four-door-fans','/api/phase6-observations','/fan82.js','/fan82.css','/truth-evidence.js'):return super().do_GET()
            if not self._authorized():return
            try:
                if path in ('/fan82.js','/fan82.css'):return self._send(200,(ROOT/'web'/path[1:]).read_bytes(),'text/javascript; charset=utf-8' if path.endswith('.js') else 'text/css; charset=utf-8')
                if path=='/truth-evidence.js':
                    body=(Path(config['child']['phase6_root'])/'phase6_web/truth-evidence.js').read_text().replace("value.current_solver_context?.variant==='cad_body_mounted'","['cad_body_mounted','cad_doors_mounted','cad_doors_four_fans'].includes(value.current_solver_context?.variant)").replace('门、辅机安装等仍待迁移','安装与参数仍待实测核验，边界以当前模型说明为准')
                    return self._send(200,body.encode(),'text/javascript; charset=utf-8')
                state=self.server.controller.snapshot(heartbeat=False);plant=state.get('plant') or {};out=panel_payload(plant,spec,freeze['validated_identity_sha256']['full'],freeze['geometry_sha256'])
                if not out['identity_match']:raise ValueError('Explicit82 native identity not ready/mismatched')
                if path=='/api/phase6-observations':
                    rows=[]
                    for name,c in plant['telemetry']['fan82_named_endpoint']['coordinates'].items():
                        for field in ('position','velocity'):rows.append({'id':name+'.'+field,'unit':c[field+'_unit'],'value':c[field],'available':True,'availability':'NATIVE_NAMED_SOLVER_NOT_HARDWARE','source_path':['telemetry','fan82_named_endpoint','coordinates',name,field]})
                    out.update(schema='gtsd-phase6-named-auxiliary-observations-v1',mounted_variant=True,actual_variant=plant['variant'],observations=rows,world_endpoint_observations=plant['telemetry']['fan82_named_endpoint'],mount_sensor_records=[{'mount':k,'record':v,'quality':'NATIVE_VIRTUAL_SENSOR_NOT_HARDWARE'} for group in ('auxiliary_mount','four_door_fans') for k,v in plant['telemetry'][group].get('last_sensor',{}).items()],cad_migration_display_limits=plant['cad_migration_display_limits'])
                return self._send(200,dict(out,phase=state.get('phase'),session_id=state.get('session_id'),snapshot_age_s=state.get('snapshot_age_s')))
            except (ValueError,OSError,KeyError) as exc:return self._send(409,{'error':str(exc),'solver_changed':False})
    return Handler

def make_server(config):
    P=phase7()
    if config['variant']!=VARIANT:return P.make_server(config['child'])
    S=P.phase6_module();server,original=S.make_server(config['child']['base']);basefactory=fan_factory(config);context={};candidates=weakref.WeakKeyDictionary()
    def factory():
        plant=basefactory();candidates[plant]=build_registry(config,plant);return plant
    class Controller(type(original)):
        def _state(self,snapshot):
            result=super()._state(snapshot)
            if result.get('plant') is not None:
                try:
                    registry=candidates[self.plant];result['plant']['lineage_observations']=registry.observe(result['plant']);context['registry']=registry
                except Exception as exc:result['plant']['lineage_observations']={'identity_match':False,'observations':[],'error':str(exc),'calibrated':False,'hardware_connected':False}
            return result
    controller=Controller(factory=factory,resume=original.resume);server.controller=controller
    server.RequestHandlerClass=build_handler(P.build_handler(server.RequestHandlerClass,context,config['child']),context,config)
    return server,controller,context

def main():
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['doctor','serve']);parser.add_argument('--config',default=None);a=parser.parse_args()
    try:
        config=load_config(a.config);result=doctor(config)
        if a.action=='doctor' or not result['ok']:print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['ok'] else 2
        server,controller,_=make_server(config);controller.start();print(f'GTSD explicit {config["variant"]} http://127.0.0.1:{server.server_port}',flush=True)
        try:server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt:pass
        finally:server.server_close();controller.close()
        return 0
    except Exception as exc:print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False));return 2
if __name__=='__main__':raise SystemExit(main())
