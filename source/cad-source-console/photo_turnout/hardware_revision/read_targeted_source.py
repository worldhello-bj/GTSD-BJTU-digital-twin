"""Read-only bounded station/circle/known-fastener-block witness extraction."""
from pathlib import Path
import sqlite3,zlib,json,math,gzip,hashlib,argparse
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parent
ap=argparse.ArgumentParser();ap.add_argument('--native-records',type=Path,required=True);DB=ap.parse_args().native_records
c=sqlite3.connect('file:'+str(DB)+'?mode=ro',uri=True)
shift=np.array([-39254.296868109456,-33707.31594700124]);roi=np.array([[-9475,100],[-8310,450]])
def parse(r):
 tags=json.loads(zlib.decompress(r[10]));d={k:v for k,v in tags};return {'handle':r[1],'record_id':r[0],'type':r[3],'block':r[4],'source_line_range':[r[7],r[8]],'metadata':json.loads(r[9]),'tags':tags},d
def inside(p,r=0):return bool(np.all(p+r>=roi[0])and np.all(p-r<=roi[1]))
def xy(d,x='10',y='20'):return np.array([float(d[x]),float(d[y])])
source=json.load(gzip.open(ROOT/'source_seed.json.gz','rt'));stations=[];handles=set()
for m in source['meshes']:
 if m['name'].startswith('CAD_TRACK_REV_long_sleeper_'):
  num=int(m['name'].split('sleeper_')[1].split('_')[0])
  if num<=5:
   d=m['metadata'];stations.append({'station':num,'source_sleeper':m['name'],'source_handles':d['source_handles'],'axis_world_mm':(np.array(d['source_centerline_world_endpoints_m'])*1000).tolist()});handles.update(d['source_handles'])
records=[]
for h in sorted(handles):
 r=c.execute('select * from records where handle=?',(h,)).fetchone();record,d=parse(r);record['world_points_mm']=[(xy(d)+shift).tolist(),(xy(d,'11','21')+shift).tolist()];records.append(record)
# Only model circles near these stations and instances of the already-known
# fastener block are examined. This is not a catalogue-wide part discovery.
circles=[];circle_count=0
for r in c.execute("select * from records where definition_block='*Model_Space' and entity_type='CIRCLE'"):
 circle_count+=1;record,d=parse(r);p=xy(d)+shift;radius=float(d['40'])
 if inside(p,radius):record.update(center_world_mm=p.tolist(),radius_mm=radius);circles.append(record)
block='A$C6E4F4F8B';detail=[];vertices=[]
for r in c.execute('select * from records where definition_block=? and entity_type in (\'LINE\',\'ARC\',\'CIRCLE\',\'LWPOLYLINE\')',(block,)):
 record,d=parse(r);detail.append(record)
 if '10'in d and'20'in d:vertices.append(xy(d))
 if '11'in d and'21'in d:vertices.append(xy(d,'11','21'))
placements=[];scanned=0;near=[]
for r in c.execute("select * from records where definition_block='*Model_Space' and entity_type='INSERT'"):
 record,d=parse(r)
 if d.get('2')!=block:continue
 scanned+=1;a=math.radians(float(d.get('50',0)));R=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]);sc=np.array([float(d.get('41',1)),float(d.get('42',1))]);p=xy(d);vv=np.asarray(vertices)*sc@R.T+p+shift;lo,hi=vv.min(0),vv.max(0)
 if np.all(hi>=roi[0])and np.all(lo<=roi[1]):record.update(bounds_world_mm=[lo.tolist(),hi.tolist()]);placements.append(record)
 # Keep nearest other-rail instance as a separate reference, never silently
 # reuse its bore pattern at a frog station.
 if record['handle']=='4EDF8':near.append(record)
out={'schema':'gtsd-bounded-station-fastener-source-review-v1','source_native_dxf_sha256':'73f9eeb145cfbda5f01d1302d3e6e09d2d81e707ff7776187d899100f654f143','native_database_opened_read_only':True,'roi_world_mm':roi.tolist(),'stations':sorted(stations,key=lambda r:r['station']),'station_axis_records':records,'direct_model_circle_records_examined':circle_count,'circle_hits':circles,'known_fastener_block':block,'known_block_placement_records_examined':scanned,'known_block_placements_intersecting_target_roi':placements,'separate_reference_insert':near,'known_fastener_detail_records':detail,'scope_limit':'Circle bounds and known-block geometry bounds only; no assertion that every possible hidden hole or unknown block was found. Exact station axis records have no bolt diameter/Z/fastener section.','conclusion':'No verified through-foot or mounting-hole identity may be inferred from source axes or a proxy mesh.'}
(H/'targeted_source_evidence.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'stations':len(stations),'circle_hits':len(circles),'targeted_known_block_placements':len(placements),'known_block_detail_records':len(detail)}))
