from pathlib import Path
import sys,os,json,hashlib,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import strict82_server as S
c=S.load_config(os.environ.get('GTSD_CONSOLE_CONFIG') or ROOT/'strict82_config.json');p=argparse.ArgumentParser();p.add_argument('--mode',choices=['source','display']);a=p.parse_args()
if a.mode=='source':
 F=S.fan_console();P=F.phase7();P.phase6_module().source_modules(c['base']['child']['base']);P.door_factory(c['base']['child'])
 sys.path.insert(0,c['strict_root']);from optimized_four_fan import TrustedRollbackFourDoorFansPlant
 plant=TrustedRollbackFourDoorFansPlant();raw=plant
 import bootstrap
 assert Path(bootstrap.__file__).resolve()==Path(c['strict_root'])/'bootstrap.py'
else:plant=S.display_factory(c)();raw=plant._raw
h=lambda v:hashlib.sha256(S.canonical(v)).hexdigest()
rows=[]
for key,v,f in [('end_neg_y_neg',.2,False),('end_neg_y_pos',.4,False),('end_pos_y_neg',.6,False),('end_pos_y_pos',.8,True)]:plant.command('four_door_fan',{'key':key,'voltage_fraction':v,'power_failed':f})
for n in range(3):
 cp=plant.checkpoint();s=plant.snapshot();plant.snapshot();assert cp==plant.checkpoint();assert cp==raw.checkpoint()
 rows.append({'steps':n,'checkpoint_sha256':h(cp),'raw_snapshot_sha256':h(raw.snapshot()),'qpos_qvel_sha256':h([s['qpos'],s['qvel']]),'native_identity':s['model_identity_sha256'],'native_mro':[x.__module__+'.'+x.__name__ for x in type(raw).__mro__]})
 if n<2:plant.step(1)
print(json.dumps({'mode':a.mode,'rows':rows,'snapshot_readonly':True,'raw_and_display_checkpoint_equal':True},indent=2))
