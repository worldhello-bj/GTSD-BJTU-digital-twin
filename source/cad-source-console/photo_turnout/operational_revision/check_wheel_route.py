"""Exact source rail-top triangle coverage for a straight guided-wheel route."""
import datetime,gzip,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
Y=.2589166666667;WIDTH=.012;RADIUS=.045;LO=-9.62;HI=-8.4
meshes=json.load(gzip.open(H/'regional_meshes.json.gz','rt'))['meshes']
def merge(spans):
 out=[]
 for a,b in sorted(spans):
  if out and a<=out[-1][1]+1e-10:out[-1][1]=max(out[-1][1],b)
  else:out.append([float(a),float(b)])
 return out
rows=[]
for m in meshes:
 if not m['name'].startswith(('PHOTO_FROG','PHOTO_WING','PHOTO_RETAIN')):continue
 v=np.array(m['vertices']).reshape(-1,3);ts=v[np.array(m['indices']).reshape(-1,3)];norm=np.cross(ts[:,1]-ts[:,0],ts[:,2]-ts[:,0]);ts=ts[(np.max(abs(ts[:,:,2]),axis=1)<1e-7)&(norm[:,2]>1e-14)];center=[];tread=[]
 for tri in ts:
  xx=[]
  for p,q in zip(tri,np.roll(tri,-1,axis=0)):
   if abs(q[1]-p[1])<1e-15:
    if abs(Y-p[1])<1e-10:xx.extend([p[0],q[0]])
   elif min(p[1],q[1])-1e-12<=Y<=max(p[1],q[1])+1e-12:xx.append(p[0]+(q[0]-p[0])*(Y-p[1])/(q[1]-p[1]))
  if len(xx)>1:center.append([min(xx),max(xx)])
  poly=list(tri)
  for limit,sgn in [(Y-WIDTH/2,1),(Y+WIDTH/2,-1)]:
   new=[]
   for p,q in zip(poly,poly[1:]+poly[:1]):
    a,b=(p[1]-limit)*sgn,(q[1]-limit)*sgn
    if a>=0:new.append(p)
    if (a>=0)!=(b>=0):new.append(p+(q-p)*a/(a-b))
   poly=new
  if poly:tread.append([min(p[0]for p in poly),max(p[0]for p in poly)])
 rows.append({'name':m['name'],'centerline_top_x_intervals_m':merge(center),'tread_width_top_x_intervals_m':merge(tread)})
center=merge([s for r in rows for s in r['centerline_top_x_intervals_m']]);tread=merge([s for r in rows for s in r['tread_width_top_x_intervals_m']]);gaps=lambda spans:[{'start_x_m':a[1],'end_x_m':b[0],'gap_mm':(b[0]-a[1])*1000}for a,b in zip(spans,spans[1:])if LO<a[1]<HI]
samples=[]
for x in np.linspace(LO,HI,1221):samples.append({'x_m':float(x),'centerline_owners':[r['name']for r in rows if any(a-1e-12<=x<=b+1e-12 for a,b in r['centerline_top_x_intervals_m'])],'tread_width_owners':[r['name']for r in rows if any(a-1e-12<=x<=b+1e-12 for a,b in r['tread_width_top_x_intervals_m'])]})
now=datetime.datetime.now(datetime.timezone.utc);out={'schema':'gtsd-source-top-route-coverage-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'route_y_m':Y,'wheel_width_m':WIDTH,'wheel_radius_m':RADIUS,'x_range_m':[LO,HI],'sample_pitch_mm':1,'sample_count':len(samples),'centerline_supported_samples':sum(bool(s['centerline_owners'])for s in samples),'tread_width_projected_supported_samples':sum(bool(s['tread_width_owners'])for s in samples),'centerline_intervals_m':center,'tread_width_intervals_m':tread,'centerline_interior_gaps':gaps(center),'tread_width_interior_gaps':gaps(tread),'source_meshes':rows,'coverage_method':'Exact world XY top triangle / horizontal line or wheel-width strip intersection; sample occupancy evaluated from merged exact intervals.','interpretation':'Wheel-width projection gives potential support at any lateral point. Actual wheel Z/contact response is determined by the native solver. Generated approach meshes belong to the solver contract and are not part of this source-only check.'}
(H/'wheel_route_geometry.json').write_text(json.dumps(out,indent=2)+'\n');(H/'wheel_route_samples.csv').write_text('x_m,centerline_owners,tread_width_owners\n'+'\n'.join(f"{s['x_m']:.9f},{'|'.join(s['centerline_owners'])},{'|'.join(s['tread_width_owners'])}"for s in samples)+'\n');print(json.dumps(out,indent=2))
