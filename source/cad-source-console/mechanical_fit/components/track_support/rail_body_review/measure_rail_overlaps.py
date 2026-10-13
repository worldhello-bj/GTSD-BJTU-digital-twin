#!/usr/bin/env python3
"""Independent read-only follow-on. Does not change or rebuild support profiles."""
from pathlib import Path
import argparse,hashlib,json,sys
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent));import portable_rebuild as p

def slices(v,f,z):
 tri=v[f];out=[]
 for t in tri:
  points=[]
  for a,b in zip(t,np.roll(t,-1,axis=0)):
   if (a[2]<z<b[2])or(b[2]<z<a[2]):points.append(a[:2]+(b[:2]-a[:2])*(z-a[2])/(b[2]-a[2]))
  if len(points)==2 and np.linalg.norm(points[0]-points[1])>1e-12:out.append(points)
 return np.asarray(out)

def intervals(segs,x):
 if not len(segs):return []
 a=segs[:,0];b=segs[:,1];ok=(np.minimum(a[:,0],b[:,0])<=x)&(np.maximum(a[:,0],b[:,0])>x);a=a[ok];b=b[ok];ys=a[:,1]+(b[:,1]-a[:,1])*(x-a[:,0])/(b[:,0]-a[:,0]);ys=np.unique(np.round(ys,12));assert len(ys)%2==0,(x,ys);return list(zip(ys[::2],ys[1::2]))

def ray_exit(v,f,point,d):
 tri=v[f];e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0];h=np.cross(np.broadcast_to(d,e2.shape),e2);det=(e1*h).sum(1);s=point-tri[:,0];q=np.cross(s,e1);ok=abs(det)>1e-16;u=np.zeros(len(f));w=u.copy();tt=u.copy();u[ok]=(s[ok]*h[ok]).sum(1)/det[ok];w[ok]=(q[ok]*d).sum(1)/det[ok];tt[ok]=(e2[ok]*q[ok]).sum(1)/det[ok];ok&=(u>=-1e-10)&(w>=-1e-10)&(u+w<=1+1e-10)&(tt>1e-10);assert ok.any();return float(tt[ok].min())

def study(ga,gb):
 va,fa=ga;vb,fb=gb;lo=max(va[:,0].min(),vb[:,0].min());hi=min(va[:,0].max(),vb[:,0].max());rows=[]
 for label,z in [('head',-.0065),('upper_transition',-.016),('web',-.035),('base_transition',-.055),('base',-.0615)]:
  sa=slices(*ga,z);sb=slices(*gb,z);found=[]
  for x in np.linspace(lo+1e-6,hi-1e-6,2001):
   for a,b in intervals(sa,x):
    for c,d in intervals(sb,x):
     y0=max(a,c);y1=min(b,d)
     if y1-y0>1e-8:found.append((float(y1-y0),float(x),float((y0+y1)/2)))
  row={'height_layer':label,'z_world_mm':z*1000,'sampled_x_count':2001,'positive_overlap_found':bool(found),'absence_not_global_proof':not bool(found)}
  if found:
   width,x,y=max(found);pt=np.array([x,y,z]);chords={}
   for i,key in enumerate('XYZ'):
    axis=np.zeros(3);axis[i]=1;neg=min(ray_exit(*ga,pt,-axis),ray_exit(*gb,pt,-axis));pos=min(ray_exit(*ga,pt,axis),ray_exit(*gb,pt,axis));chords[key]={'negative_mm':neg*1000,'positive_mm':pos*1000,'shared_material_chord_mm':(neg+pos)*1000}
   row.update({'witness_world_m':pt.tolist(),'maximum_sampled_world_Y_overlap_chord_mm':width*1000,'positive_x_sample_span_m':[min(r[1]for r in found),max(r[1]for r in found)],'positive_x_sample_count':len(set(r[1]for r in found)),'actual_surface_exit_chords':chords})
  rows.append(row)
 return rows

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--project-root',type=Path,required=True);a=ap.parse_args();base=a.project_root/'baseline_console/GTSD_Assembly_Console';cat=p.read(base/'app/data/catalog.json');scene=p.load_glb(base/'assets/whole.glb.gz',cat);rep=p.read(a.project_root/'integration/data/source_replacements.json')
 for r in rep['replacements']:del scene[r['old_name']]
 for r in rep['meshes']:scene[r['name']]=(np.array(r['vertices']).reshape(-1,3),np.array(r['indices']).reshape(-1,3))
 rows=[]
 for pair in [('DETAIL_BENCH_turnout_inner_left_segment','ENV_main_rail_1'),('DETAIL_BENCH_guard_diverging','CAD_TRACK_REV_outer_branch_native_xy')]:rows.append({'pair':pair,'height_layer_results':study(scene[pair[0]],scene[pair[1]])})
 out={'schema':'gtsd-inherited-rail-body-overlap-layers-v1','actual_source_default_mesh_count':len(scene),'source_replacements_sha256':p.sha(a.project_root/'integration/data/source_replacements.json'),'pairs':rows,'measurement_limit':'Actual triangle-slice intervals and actual ray exits; maxima and X spans are sampled lower-bound observations, not exact intersection volume or minimum separating translation. No source axes or support profiles changed.'};p.write(HERE/'rail_layer_overlap.json',out);print(json.dumps(out,indent=2))
if __name__=='__main__':main()
