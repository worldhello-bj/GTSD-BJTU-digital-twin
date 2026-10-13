#!/usr/bin/env python3
"""Deterministic, material-preserving convex partitions of pinned GTSD meshes.

All calculations use a translated mm frame. Source coordinates are never snapped.
Two-layer side irregularities are preserved as exact tetrahedral wedges, then
merged with adjacent core cells only if the convex hull adds no material.
Run using system Python (NumPy, SciPy and system GEOS are preinstalled).
"""
from __future__ import annotations
import argparse, hashlib, json, math, pathlib, time
import numpy as np
from scipy.spatial import ConvexHull
import geometry as G
import geos_kernel as K
HERE=pathlib.Path(__file__).resolve().parent
K.bind('GEOSBoundary',[K.P],K.P)
SOURCE_FILES=['mechanical_fit/profiles.json','mechanical_fit/components/clamps/published_clamp_meshes.json']

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def canonical(obj):return json.dumps(obj,sort_keys=True,separators=(',',':')).encode()
def geo_sha(m):return hashlib.sha256(canonical({'name':m['name'],'vertices':m['vertices'],'indices':m['indices']})).hexdigest()
def destroy(*ps):
 for p in ps:K.G.GEOSGeom_destroy(p)
def area2(p):return sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(p,p[1:]+p[:1]))
def normalize(p):
 p=[tuple(map(float,q)) for q in p]
 if p[0]==p[-1]:p=p[:-1]
 if area2(p)<0:p.reverse()
 i=min(range(len(p)),key=lambda i:p[i]);return p[i:]+p[:i]
def convex(p):
 return all((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])>=-1e-13 for a,b,c in zip(p,p[1:]+p[:1],p[2:]+p[:2]))
def merge_polygon(a,b):
 edges={ (x,y) for p in (a,b) for x,y in zip(p,p[1:]+p[:1]) }
 shared=[(x,y) for x,y in edges if (y,x) in edges]
 if not shared:return None
 edges.difference_update(shared);nxt={a:b for a,b in edges}
 if len(nxt)!=len(edges):return None
 p=[min(nxt)];cur=p[0]
 while True:
  cur=nxt.get(cur)
  if cur==p[0]:break
  if cur is None or cur in p:return None
  p.append(cur)
 if len(p)!=len(edges) or len(p)>32 or not convex(p):return None
 return normalize(p)
def merge_triangles(triangles):
 cells={i:normalize(p) for i,p in enumerate(triangles)}
 while True:
  edge={};pairs=set()
  for i,p in cells.items():
   for a,b in zip(p,p[1:]+p[:1]):
    if (b,a) in edge:pairs.add(tuple(sorted((i,edge[(b,a)]))))
    edge[(a,b)]=i
  changes=0
  for i,j in sorted(pairs):
   if i not in cells or j not in cells:continue
   p=merge_polygon(cells[i],cells[j])
   if p is not None:cells[i]=p;del cells[j];changes+=1
  if not changes:break
 return [cells[i] for i in sorted(cells)]
def hull_piece(vertices):
 v=np.unique(vertices,axis=0);h=ConvexHull(v)
 used=np.sort(h.vertices);v=v[used];h=ConvexHull(v)
 f=h.simplices.copy()
 for i,t in enumerate(f):
  if np.dot(np.cross(v[t[1]]-v[t[0]],v[t[2]]-v[t[0]]),h.equations[i,:3])<0:f[i]=t[::-1]
 return {'v':v,'f':f,'volume_mm3':float(h.volume),'eq':h.equations}
def prism(p,axis,lo,hi):
 uv=[i for i in range(3)if i!=axis];v=np.zeros((len(p)*2,3));v[:len(p),axis]=lo;v[len(p):,axis]=hi;v[:len(p),uv]=p;v[len(p):,uv]=p
 return hull_piece(v)
def merge_wedge(cells,wedge):
 # Exact convex-union test. Each pair has disjoint interiors by construction.
 # A combined convex hull with the same volume is the same regular closed solid.
 candidates=[]
 for i,p in enumerate(cells):
  if not np.all(p['v'].max(0)+1e-10>=wedge['v'].min(0)) or not np.all(wedge['v'].max(0)+1e-10>=p['v'].min(0)):continue
  h=hull_piece(np.r_[p['v'],wedge['v']]);error=h['volume_mm3']-p['volume_mm3']-wedge['volume_mm3']
  tol=max(1e-11,1e-12*h['volume_mm3'])
  if abs(error)<=tol:candidates.append((abs(error),i,h,error,tol))
 if not candidates:
  cells.append(wedge)
  return {'merged_into_cell':None,'standalone_shell_inertia_mesh':True,'wedge_volume_mm3':wedge['volume_mm3'],'reason':'Retain non-prismatic float32 boundary exactly; shell asset inertia avoids MuJoCo minimum-volume rejection, while geom mass=0 and explicit body inertia exclude asset inertia from dynamics.'}
 _,i,h,error,tol=min(candidates,key=lambda q:(q[0],q[1]));cells[i]=h
 return {'merged_into_cell':i,'convex_union_volume_error_mm3':error,'comparison_tolerance_mm3':tol,'wedge_volume_mm3':wedge['volume_mm3']}
def poly_union(polygons):
 if not polygons:return K.read('POLYGON EMPTY')
 w='GEOMETRYCOLLECTION('+','.join('POLYGON(('+K.coords(list(p)+[p[0]])+'))'for p in polygons)+')'
 src=K.read(w);out=K.G.GEOSUnaryUnion(src);destroy(src);return out

def partition(m,axis):
 v=G.verts(m);origin=(v.min(0)+v.max(0))/2;local=(v-origin)*1000;f=G.faces(m);uv=[i for i in range(3)if i!=axis]
 levels=np.unique(local[:,axis]);assert len(levels)==2,(m['name'],len(levels));lo,hi=levels
 p=G.cap(m,axis,origin);q=G.cap(m,axis,origin,'max')
 exceptional=[];side_count=0;side_projection=[];regular_segments=[]
 for fid,t in enumerate(local[f]):
  if np.ptp(t[:,axis])==0:continue
  side_count+=1
  if len(np.unique(t[:,uv],axis=0))==3:
   a=abs(area2([tuple(x)for x in t[:,uv]]))/2
   if a>0:exceptional.append((fid,t,a));side_projection.append(normalize(t[:,uv]))
  else:regular_segments.append(np.unique(t[:,uv],axis=0).tolist())
 bp=K.G.GEOSBoundary(p);bq=K.G.GEOSBoundary(q);both_boundary=K.G.GEOSUnion(bp,bq);rl=K.linework(regular_segments);unmatched=K.G.GEOSDifference(rl,both_boundary);length=K.D();K.G.GEOSLength(unmatched,K.C.byref(length));assert length.value<1e-8,('Side boundary mismatch',length.value)
 side_boundary_unmatched_mm=length.value;destroy(bp,bq,both_boundary,rl,unmatched)
 s=poly_union(side_projection)
 core=K.G.GEOSDifference(p,s);core_hi=K.G.GEOSDifference(q,s)
 cap_core_equivalence=K.equivalence(core,core_hi,1e-9)
 triangles=[normalize(t[0]) for t in K.triangulate(core)]
 tri_union=poly_union(triangles);tri_equiv=K.equivalence(core,tri_union,1e-9)
 polygons=merge_triangles(triangles);poly=poly_union(polygons);poly_equiv=K.equivalence(core,poly,1e-9)
 assert abs(sum(abs(area2(t))/2 for t in polygons)-K.area(core))<1e-8
 cells=[prism(t,axis,lo,hi)for t in polygons];wedge_proofs=[]
 for fid,t,a in exceptional:
  normal=np.cross(t[1]-t[0],t[2]-t[0]);end=hi if normal[axis]<0 else lo
  vt=t.copy();vt[:,axis]=end;wedge=hull_piece(np.r_[t,vt]);proof=merge_wedge(cells,wedge);proof.update({'source_triangle':fid,'projected_area_mm2':a,'source_vertices_local_mm':t.tolist()});wedge_proofs.append(proof)
 source_volume=G.signed_volume(m)*1e9;volume=sum(c['volume_mm3']for c in cells)
 assert abs(volume-source_volume)<=max(1e-7,1e-11*source_volume),(volume,source_volume)
 # Side regularity proof: every source side triangle projects to a cap edge or
 # one explicitly retained exceptional triangle; all vertices are on two caps.
 # In this 2-layer case these boundary patches plus both caps describe the whole
 # closed solid, not only selected cross sections.
 boundary=G.topology(m);assert boundary['closed'] and boundary['positive_volume']
 source_cap_delta=K.G.GEOSDifference(p,q);source_cap_delta2=K.G.GEOSDifference(q,p)
 hd=K.D();K.G.GEOSHausdorffDistance(p,q,K.C.byref(hd))
 slices=[]
 # Source triangle sections use the supplied 1e-9 mm snap, reported explicitly.
 for frac in [.125,.375,.5,.625,.875]:
  at=origin[axis]+(lo+(hi-lo)*frac)/1000;src=G.actual_section(m,axis,at,origin)
  sections=[]
  for c in cells:
   mm={'name':'cell','vertices':(c['v']/1000+origin).ravel().tolist(),'indices':c['f'].ravel().tolist()}
   cp=G.actual_section(mm,axis,at,origin)
   sections.extend(K.polys(cp));destroy(cp)
  w='GEOMETRYCOLLECTION('+','.join('POLYGON('+','.join('('+K.coords(r)+')'for r in loops)+')'for loops in sections)+')'
  raw=K.read(w);out=K.G.GEOSUnaryUnion(raw);destroy(raw)
  src_check=K.G.GEOSGeom_setPrecision(src,1e-7,0); out_check=K.G.GEOSGeom_setPrecision(out,1e-7,0)
  e=K.equivalence(src_check,out_check,3e-7);e['depth_fraction']=frac;e['validation_only_precision_mm']=1e-7;slices.append(e);destroy(src,out,src_check,out_check)
 holes=[]
 for loops in K.polys(p):
  for ring in loops[1:]:
   rp=K.polygon(ring);pt=K.G.GEOSPointOnSurface(rp);coord=K.gj(pt)['coordinates'];localpt=np.zeros(3);localpt[axis]=(lo+hi)/2;localpt[uv]=coord
   membership=[bool(np.all(c['eq'][:,:3]@localpt+c['eq'][:,3]<=1e-10))for c in cells];assert not any(membership)
   holes.append({'center_world_m':(localpt/1000+origin).tolist(),'ring_world_uv_m':(np.array(ring)/1000+origin[uv]).tolist(),'point_in_any_collision_cell':False});destroy(rp,pt)
 proof={'source_topology':boundary,'source_two_layer_axis':axis,'source_layers_m':(levels/1000+origin[axis]).tolist(),'source_side_triangles':side_count,'uniform_side_triangles':side_count-len(exceptional),'regular_side_projection_unmatched_boundary_length_mm':side_boundary_unmatched_mm,'exceptional_source_wedges_preserved':wedge_proofs,'source_cap_hausdorff_mm':hd.value,'source_cap_symdiff_mm2':K.area(source_cap_delta)+K.area(source_cap_delta2),'core_min_max_equivalence':cap_core_equivalence,'cap_triangulation_equivalence':tri_equiv,'convex_merge_equivalence':poly_equiv,'source_volume_mm3':source_volume,'collision_union_volume_mm3':volume,'volume_error_mm3':volume-source_volume,'interior_overlap_volume_mm3':0.0,'disjointness_basis':'Disjoint GEOS constrained cap triangles, exact edge-adjacent convex unions, and disjoint source side-wedge projections outside the core.','sliced_source_collision_equivalence':slices,'section_coordinate_snap_mm':1e-9,'hole_negative_witnesses':holes,'initial_triangle_prism_count':len(triangles),'final_convex_piece_count':len(cells)}
 out=[]
 for i,c in enumerate(cells):
  out.append({'id':f'cell_{i:04d}','vertices_local_m':(c['v']/1000).ravel().tolist(),'triangles':c['f'].ravel().tolist(),'volume_m3':c['volume_mm3']*1e-9,'mass':0.0,'density':0.0,'mesh_inertia':'shell'})
 destroy(p,q,s,core,core_hi,tri_union,poly,source_cap_delta,source_cap_delta2)
 return {'source_name':m['name'],'source_geometry_sha256':geo_sha(m),'body_key':m['name'],'body_reference_world_m':origin.tolist(),'axis':axis,'pieces':out,'proof':proof}

def read_sources(source):
 profiles=json.loads((source/SOURCE_FILES[0]).read_text());combined=next(p for p in profiles['profiles']if p['id']=='combined');candidate=[m for g in combined['groups']for m in g['add_meshes']];retained=json.loads((source/SOURCE_FILES[1]).read_text())['meshes'];return candidate,retained

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,default=HERE/'generated');args=ap.parse_args();source=args.source.resolve();out=args.output.resolve();assert not out.is_relative_to(source);out.mkdir(parents=True,exist_ok=True)
 hashes={f:sha(source/f)for f in SOURCE_FILES};candidate,retained=read_sources(source)
 selected=[(m,1)for m in candidate if 'B535' in m['name']and'HOLE25'in m['name']]+[(m,0)for m in retained if m['name'].endswith('F643_profile_0')]
 parts=[]
 for m,axis in selected:
  t=time.monotonic();result=partition(m,axis);parts.append(result);print(m['name'],len(result['pieces']),'pieces',round(time.monotonic()-t,2),'s',flush=True)
 manifest={'schema':'gtsd-exact-contact-partition-v1','source_files_sha256':hashes,'builder_sha256':sha(pathlib.Path(__file__)),'units':'m','status':'CORE_GEOMETRY_PARTITION_BUILT_NATIVE_TEST_RESULT_SEPARATE','mass_policy':'Each collision geom has mass=0 and density=0; caller supplies explicit body inertial from source solids exactly once.','piece_count':sum(len(p['pieces'])for p in parts),'parts':parts,'remaining_nonconvex_interfaces':[m['name'] for m in retained if any(s in m['name']for s in ['upper_tube','jaw_','ring_','source-plan6.5'])]}
 assert hashes=={f:sha(source/f)for f in SOURCE_FILES};assert manifest['piece_count']<1200
 (out/'collision_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('TOTAL',manifest['piece_count'],flush=True)
if __name__=='__main__':main()
