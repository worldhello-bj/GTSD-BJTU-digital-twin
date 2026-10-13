#!/usr/bin/env python3
"""Portable explicit strict82 console; composition preserves reviewed native MRO."""
from pathlib import Path
import argparse,copy,hashlib,importlib,json,math,os,sys,types,weakref
from urllib.parse import urlsplit
ROOT=Path(__file__).resolve().parent
VARIANT='cad-doors-four-fans-strict-rollback-research'
ACTUAL='cad_doors_four_fans_scoped_rollback'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def canonical(v):return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()
def strict(body):
 def pairs(rows):
  out={}
  for k,v in rows:
   if k in out:raise ValueError('Duplicate JSON key')
   out[k]=v
  return out
 def bad(x):raise ValueError('Nonfinite JSON')
 v=json.loads(body,object_pairs_hook=pairs,parse_constant=bad)
 def check(x):
  if type(x)in(int,float) and not math.isfinite(x):raise ValueError('Nonfinite number')
  if isinstance(x,dict):
   for q in x.values():check(q)
  if isinstance(x,list):
   for q in x:check(q)
 check(v);return v

def admitted(path,key):
 if sha(path)!=strict((ROOT/'accepted_dependencies.json').read_bytes())[key]:raise ValueError('Unknown reviewed dependency: '+key)
 return Path(path).read_bytes()
def fan_console():return importlib.import_module('fan82_server')
def load_config(path=None):
 p=Path(path or os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'console_config.json').resolve()
 if p.stat().st_size>65536:raise ValueError('Configuration exceeds64KiB')
 v=strict(p.read_bytes())
 if set(v)!={'schema','fan82_console_root','fan82_config','strict_root','trusted_root','variant'} or v['schema']!='gtsd-strict82-console-config-v1':raise ValueError('Exact strict82 console fields required')
 def resolve(x):
  if type(x)is not str or not x.strip():raise ValueError('Nonempty local path required')
  return (p.parent/Path(x).expanduser()).resolve()
 root=resolve(v['fan82_console_root']);freeze=strict(admitted(root/'SOURCE_FREEZE_CONSOLE_FAN82.json','fan82_console_freeze'))
 for name,h in freeze['source_sha256'].items():
  if sha(root/name)!=h:raise ValueError('Frozen82 console changed: '+name)
 sys.dont_write_bytecode=True
 if str(root)not in sys.path:sys.path.insert(0,str(root))
 F=fan_console()
 if Path(F.__file__).resolve().parent!=root:raise ValueError('Conflicting82 console module')
 base=F.load_config(resolve(v['fan82_config']))
 if v['variant'] not in ('internal-coupled-research',VARIANT):raise ValueError('Select explicit default75 or strict82 variant')
 if base['variant']!=('internal-coupled-research' if v['variant']=='internal-coupled-research' else F.VARIANT):raise ValueError('Explicit underlying configuration/variant mismatch')
 out={'path':str(p),'base':base,'fan82_console_root':str(root),'strict_root':str(resolve(v['strict_root'])),'trusted_root':str(resolve(v['trusted_root'])),'variant':v['variant']}
 if v['variant']==VARIANT:verify_strict(out)
 return out

def verify_strict(config):
 F=fan_console();basefreeze=F.verify_fan82(config['base']);root=Path(config['strict_root']);freeze=strict(admitted(root/'SOURCE_FREEZE.json','strict_physics_freeze'))
 if set(freeze['physics_source_sha256'])!={'bootstrap.py','strict_four_fan.py','optimized_four_fan.py'}:raise ValueError('Unknown strict82 numerical source set')
 for name,h in freeze['physics_source_sha256'].items():
  if sha(root/name)!=h:raise ValueError('Unknown strict82 source: '+name)
 if freeze['frozen82_source_freeze_sha256']!=sha(Path(config['base']['fan82_root'])/'FAN82_SOURCE_FREEZE.json'):raise ValueError('Strict82 baseline freeze mismatch')
 admitted(Path(config['trusted_root'])/'trusted_rollback.py','trusted_rollback')
 if freeze['trusted_rollback_source_sha256']!=sha(Path(config['trusted_root'])/'trusted_rollback.py'):raise ValueError('Rollback identity mismatch')
 if freeze['geometry_sha256']!=basefreeze['geometry_sha256']:raise ValueError('Unexpected strict82 geometry migration')
 return freeze

def portable_bootstrap(config):
 """Explicit replacement of admission/path bootstrap only, not numeric classes.

 Source bootstrap is pinned but not executed: its two cloud literals cannot
 resolve portably. This module exports the same reviewed hashes/sha_file/roots;
 its own implementation hash is declared separately from the native identity.
 """
 f=verify_strict(config);F=fan_console();P=F.phase7();P.phase6_module().source_modules(config['base']['child']['base']);P.door_factory(config['base']['child'])
 root=Path(config['strict_root']);base=Path(config['base']['fan82_root']);trusted=Path(config['trusted_root']);paths={'ROOT':str(root),'BASE':str(base),'TRUSTED':str(trusted)}
 for name,path in [('strict_four_fan',root/'strict_four_fan.py'),('optimized_four_fan',root/'optimized_four_fan.py'),('trusted_rollback',trusted/'trusted_rollback.py')]:
  loaded=sys.modules.get(name)
  if loaded is not None and Path(loaded.__file__).resolve()!=path.resolve():raise ValueError('Conflicting numerical module '+name)
 old=sys.modules.get('bootstrap')
 if old is not None and (getattr(old,'_strict82_paths',None)!=paths or getattr(old,'_strict82_adapter_sha',None)!=sha(__file__)):raise ValueError('Generic bootstrap module conflict; start a fresh explicit process')
 if old is None:
  mod=types.ModuleType('bootstrap');mod.__file__=__file__;mod.ROOT=root;mod.BASE=base;mod.TRUSTED=trusted;mod.BASE_FREEZE_SHA=f['frozen82_source_freeze_sha256'];mod.TRUSTED_SHA=f['trusted_rollback_source_sha256'];mod.sha_file=sha;mod._strict82_paths=paths;mod._strict82_adapter_sha=sha(__file__);sys.modules['bootstrap']=mod
 for path in (root,trusted):
  if str(path)not in sys.path:sys.path.insert(0,str(path))
 from optimized_four_fan import TrustedRollbackFourDoorFansPlant
 return TrustedRollbackFourDoorFansPlant

def interface_identity(config):
 return {'schema':'gtsd-strict82-console-interface-identity-v1','adapter_source_sha256':sha(__file__),'prior_console_source_freeze_sha256':sha(Path(config['fan82_console_root'])/'SOURCE_FREEZE_CONSOLE_FAN82.json'),'strict_physics_source_freeze_sha256':sha(Path(config['strict_root'])/'SOURCE_FREEZE.json'),'trusted_rollback_source_sha256':sha(Path(config['trusted_root'])/'trusted_rollback.py'),'bootstrap_policy':'reviewed path/admission shim; original frozen bootstrap pinned, not executed; no numerical source file edits','presentation_policy':'composition delegate; native type/MRO and raw checkpoint unchanged','command_protocol':'detached snapshot enum/values -> string/enum with exact value set preserved'}

def display_factory(config):
 raw_type=portable_bootstrap(config);freeze=verify_strict(config);identity=interface_identity(config)
 from mounted_display import display_type
 from fan82_view import named_endpoint
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
   value=super().snapshot()
   if value['variant']!=ACTUAL or value['model_identity_sha256']!=freeze['validated_optimized_identity_sha256']['full'] or value['source_geometry_sha256']!=freeze['geometry_sha256'] or self.model.nv!=freeze['nv'] or self.model.nq!=freeze['nq']:raise ValueError('Unknown strict82 native identity/MRO/parameter/layout')
   key=value['command_registry']['four_door_fan']['args']['key']
   if set(key)!={'type','values'} or key['type']!='enum' or type(key['values'])is not list or any(type(x)is not str for x in key['values']) or len(key['values'])!=len(set(key['values'])):raise ValueError('Unknown frozen four-fan schema')
   value['command_registry']['four_door_fan']['args']['key']={'type':'string','enum':list(key['values'])}
   value['telemetry']['fan82_named_endpoint']=named_endpoint(self._raw)
   value['display_interface']['strict82_console_identity']=copy.deepcopy(identity)
   value['display_interface']['strict82_console_identity_sha256']=hashlib.sha256(canonical(identity)).hexdigest()
   value['title']='严格恢复与局部回滚82DOF研究控制台';value['subtitle']='显式新数值身份 · 6转轴+1活塞 · 外部恢复全校验 · 未标定'
   value['cad_migration_display_limits']=['新严格82身份校验原生used relative/air速度与实际施加轴承/电机/空气力矩一致；不是把used速度等同于端点速度。','局部回滚令牌仅当前步、当前owner、同一对象、active scope有效；外部checkpoint没有令牌捷径。','四门独立理想电源与同规格参数仍是工程假设；未校准热网与全状态收敛限制保留。','展示使用组合代理，原始求解类MRO和checkpoint身份保持已审数值身份；路径/admission展示适配另有独立身份。']
   return value
 def factory():return Display(raw_type())
 return factory

def panel_payload(snapshot,config):
 from fan82_view import fan_records
 F=fan_console();freeze=verify_strict(config);spec=F.strict(F.admitted(Path(config['base']['fan82_root'])/'evidence/fan82/fan_spec.json','fan82_spec'));defs,fields=fan_records(snapshot,spec)
 okay=snapshot.get('variant')==ACTUAL and snapshot.get('model_identity_sha256')==freeze['validated_optimized_identity_sha256']['full'] and snapshot.get('source_geometry_sha256')==freeze['geometry_sha256']
 if not okay:
  for row in fields:row.update(available=False,value=None,availability='STRICT82_MODEL_OR_GEOMETRY_IDENTITY_MISMATCH')
 t=snapshot.get('telemetry',{});names=t.get('energy_work_columns',[]);values=t.get('energy_work_columns_J',[]);extra=t.get('four_door_fans',{}).get('work',{});work={n:values[names.index(n)] for n in extra if len(names)==len(values) and names.count(n)==1}
 return {'schema':'gtsd-four-fan-console-observations-v1','variant':snapshot.get('variant'),'model_identity_sha256':snapshot.get('model_identity_sha256'),'source_geometry_sha256':snapshot.get('source_geometry_sha256'),'identity_match':okay,'sample_time_s':snapshot.get('time_s'),'steps':snapshot.get('steps'),'fans':defs,'observations':fields,'inventory':t.get('fan82_named_endpoint',{}).get('inventory',{}),'extra_work_columns_J':work if okay else {},'extra_work_columns_counted_once':okay and len(work)==len(extra) and work==extra,'hardware_connected':False,'hardware_calibrated':False,'renews_run_lease':False,'limits':snapshot.get('cad_migration_display_limits',[])+['新增三台热量仍仅诊断，未接原机柜热网络；气压峰值/全状态收敛尚未完成。','原生安装反力不重复施力或记功；罩件是固定定子，没有推断真实旋转叶片。','0电压保持电阻连接；power_failed=true为断路。最后接受区间电流与当前指令分开。'],'console_interface_identity':snapshot.get('display_interface',{}).get('strict82_console_identity')}

def adapted_fan_js(config):
 body=(Path(config['fan82_console_root'])/'web/fan82.js').read_bytes();text=body.decode()
 if text.count('cad_doors_four_fans')!=2:raise ValueError('Unknown frozen fan UI variant anchors')
 return text.replace('cad_doors_four_fans',ACTUAL).encode()

def build_handler(parent,config):
 F=fan_console()
 class Handler(parent):
  def _send(self,code,data,content_type='application/json; charset=utf-8'):
   if urlsplit(self.path).path=='/' and code==200 and isinstance(data,bytes):data=data.replace(b'</head>',b'<link rel="stylesheet" href="/fan82.css"></head>').replace(b'<section id="truth-evidence-panel"',b'<section id="four-fan-panel" class="panel"></section><section id="truth-evidence-panel"').replace(b'</body>',b'<script src="/fan82.js"></script></body>')
   return super()._send(code,data,content_type)
  def do_GET(self):
   path=urlsplit(self.path).path
   if path not in ('/api/four-door-fans','/api/phase6-observations','/fan82.js','/fan82.css','/truth-evidence.js'):return super().do_GET()
   if not self._authorized():return
   try:
    if path=='/fan82.js':return self._send(200,adapted_fan_js(config),'text/javascript; charset=utf-8')
    if path=='/fan82.css':return self._send(200,(Path(config['fan82_console_root'])/'web/fan82.css').read_bytes(),'text/css; charset=utf-8')
    if path=='/truth-evidence.js':
     body=(Path(config['base']['child']['phase6_root'])/'phase6_web/truth-evidence.js').read_text().replace("value.current_solver_context?.variant==='cad_body_mounted'","['cad_body_mounted','cad_doors_mounted','cad_doors_four_fans','"+ACTUAL+"'].includes(value.current_solver_context?.variant)").replace('门、辅机安装等仍待迁移','安装与参数仍待实测核验，边界以当前模型说明为准')
     return self._send(200,body.encode(),'text/javascript; charset=utf-8')
    state=self.server.controller.snapshot(heartbeat=False);plant=state.get('plant')or{};out=panel_payload(plant,config)
    if not out['identity_match']:raise ValueError('Strict82 native identity not ready/mismatched')
    if path=='/api/phase6-observations':
     rows=[]
     for name,c in plant['telemetry']['fan82_named_endpoint']['coordinates'].items():
      for field in ('position','velocity'):rows.append({'id':name+'.'+field,'unit':c[field+'_unit'],'value':c[field],'available':True,'availability':'NATIVE_NAMED_SOLVER_NOT_HARDWARE','source_path':['telemetry','fan82_named_endpoint','coordinates',name,field]})
     out.update(schema='gtsd-phase6-named-auxiliary-observations-v1',mounted_variant=True,actual_variant=ACTUAL,observations=rows,world_endpoint_observations=plant['telemetry']['fan82_named_endpoint'],mount_sensor_records=[{'mount':k,'record':v,'quality':'NATIVE_VIRTUAL_SENSOR_NOT_HARDWARE'} for group in ('auxiliary_mount','four_door_fans') for k,v in plant['telemetry'][group].get('last_sensor',{}).items()],cad_migration_display_limits=plant['cad_migration_display_limits'])
    return self._send(200,dict(out,phase=state.get('phase'),session_id=state.get('session_id'),snapshot_age_s=state.get('snapshot_age_s')))
   except (ValueError,OSError,KeyError) as exc:return self._send(409,{'error':str(exc),'solver_changed':False})
 return Handler

def doctor(config,check_bind=True):
 F=fan_console();out=F.doctor(config['base'],check_bind);out.update(schema='gtsd-console-strict82-preflight-v1',explicit_variant=config['variant'])
 if config['variant']==VARIANT:
  try:out['strict_freeze']=verify_strict(config);out['console_interface_identity']=interface_identity(config)
  except (ValueError,OSError) as exc:out['ok']=False;out['errors'].append({'code':'strict82_admission','message':str(exc)})
 return out

def make_server(config):
 F=fan_console()
 if config['variant']!=VARIANT:return F.make_server(config['base'])
 P=F.phase7();server,original=P.phase6_module().make_server(config['base']['child']['base']);basefactory=display_factory(config);context={};candidates=weakref.WeakKeyDictionary()
 def factory():
  plant=basefactory();candidates[plant]=F.build_registry(config['base'],plant);return plant
 class Controller(type(original)):
  def _state(self,snapshot):
   result=super()._state(snapshot)
   if result.get('plant')is not None:
    try:r=candidates[self.plant];result['plant']['lineage_observations']=r.observe(result['plant']);context['registry']=r
    except Exception as exc:result['plant']['lineage_observations']={'identity_match':False,'observations':[],'error':str(exc),'hardware_connected':False}
   return result
 ctl=Controller(factory=factory,resume=original.resume);server.controller=ctl;server.RequestHandlerClass=build_handler(P.build_handler(server.RequestHandlerClass,context,config['base']['child']),config);return server,ctl,context

def main():
 parser=argparse.ArgumentParser();parser.add_argument('action',choices=['doctor','serve']);parser.add_argument('--config');a=parser.parse_args()
 try:
  c=load_config(a.config);r=doctor(c)
  if a.action=='doctor' or not r['ok']:print(json.dumps(r,ensure_ascii=False,indent=2));return 0 if r['ok'] else 2
  server,ctl,_=make_server(c);ctl.start();print('GTSD explicit '+c['variant']+' http://127.0.0.1:'+str(server.server_port),flush=True)
  try:server.serve_forever(poll_interval=.1)
  except KeyboardInterrupt:pass
  finally:server.server_close();ctl.close()
  return 0
 except Exception as exc:print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False));return 2
if __name__=='__main__':raise SystemExit(main())
