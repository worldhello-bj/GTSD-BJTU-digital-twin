"""Exact cap/section integration on published triangulations; local coordinates mm."""
import copy,hashlib,itertools,json,math
import numpy as np
import geos_kernel as K
K.bind('GEOSDistance',[K.P,K.P,K.C.POINTER(K.D)],K.I)
def verts(m):return np.asarray(m['vertices'],float).reshape(-1,3)
def faces(m):return np.asarray(m['indices'],int).reshape(-1,3)
def poly_distance(a,b):
 d=K.D();assert K.G.GEOSDistance(a,b,K.C.byref(d))==1;return d.value
def cap(m,axis,origin,side='min'):
 v=(verts(m)-origin)*1000;f=faces(m);p=getattr(v[:,axis],side)();uv=[k for k in range(3)if k!=axis];f=f[np.max(abs(v[f,axis]-p),axis=1)<1e-6]
 if not len(f):raise ValueError('No flat cap '+m['name'])
 return K.G.GEOSUnaryUnion(K.read('MULTIPOLYGON('+','.join('(('+K.coords(v[t][:,uv].tolist()+[v[t[0],uv].tolist()])+'))' for t in f)+')'))
def section(m,axis,pos,origin):
 v=(verts(m)-origin)*1000;f=faces(m);val=(pos-origin[axis])*1000;uv=[k for k in range(3)if k!=axis];ts=v[f];ids=np.where((ts[:,:,axis].min(1)<val)&(ts[:,:,axis].max(1)>val))[0];lines=[]
 for t in ts[ids]:
  pp=[]
  for a,b in zip(t,np.roll(t,-1,axis=0)):
   if (a[axis]-val)*(b[axis]-val)<0:
    q=a+(b-a)*(val-a[axis])/(b[axis]-a[axis]);pp.append(q[uv].tolist())
  if len(pp)==2:lines.append(pp)
 if not lines:return K.read('POLYGON EMPTY')
 # All closed shells in these sections are oriented. Nest polygonized faces by
 # winding against the 3D source mesh to avoid filling interior bores.
 node=K.G.GEOSUnaryUnion(K.linework(lines));a=(K.P*1)(node);raw=K.G.GEOSPolygonize(a,1)
 return K.G.GEOSUnaryUnion(raw)
def line_length(poly,x,ymin=-1000000,ymax=1000000):
 line=K.linework([[[x,ymin],[x,ymax]]]);inter=K.G.GEOSIntersection(poly,line);d=K.D();K.G.GEOSLength(inter,K.C.byref(d));K.G.GEOSGeom_destroy(line);K.G.GEOSGeom_destroy(inter);return d.value

def cap_parallel_overlap(a,b,axis):
 o=(verts(a).min(0)+verts(a).max(0))/2;ap=cap(a,axis,o);bp=cap(b,axis,o);av=verts(a);bv=verts(b);depth=max(0,min(av[:,axis].max(),bv[:,axis].max())-max(av[:,axis].min(),bv[:,axis].min()))*1000
 area=K.area(K.G.GEOSIntersection(ap,bp));return {'method':'intersection of actual triangulated end caps times common extrusion depth','area_mm2':area,'depth_mm':depth,'volume_mm3':area*depth,'cap_min_clearance_mm':poly_distance(ap,bp)}
def cross_extrusion_overlap(a,axis_a,b,axis_b):
 """Product of exact polygon slice lengths; valid for selected uniform slabs.
 Source changes through a slab (square slot / ear bore) must be excluded or split.
 """
 assert axis_a!=axis_b
 av,bv=verts(a),verts(b);o=(av.min(0)+av.max(0))/2
 common=next(k for k in range(3)if k not in [axis_a,axis_b]);ap=cap(a,axis_a,o);bp=cap(b,axis_b,o)
 def normalize(poly,axis):
  uv=[k for k in range(3)if k!=axis]; ps=K.polys(poly);out=[]
  for loops in ps:
   new=[]
   for ring in loops:
    new.append([[q[uv.index(common)],q[1-uv.index(common)]]for q in ring])
   out.append(K.polygon(new[0],new[1:]))
  if len(out)==1:return out[0]
  return K.G.GEOSUnaryUnion(K.read('GEOMETRYCOLLECTION('+','.join('POLYGON('+','.join('('+K.coords(r)+')'for r in loops)+')'for loops in K.polys(poly))+')'))
 ap,bp=normalize(ap,axis_a),normalize(bp,axis_b)
 # Clip each shape by the finite extrusion span of the other part.
 la,ha=(av[:,axis_a].min()-o[axis_a])*1000,(av[:,axis_a].max()-o[axis_a])*1000
 lb,hb=(bv[:,axis_b].min()-o[axis_b])*1000,(bv[:,axis_b].max()-o[axis_b])*1000
 ap=K.G.GEOSIntersection(ap,K.polygon([[-1000000,lb],[1000000,lb],[1000000,hb],[-1000000,hb]]))
 bp=K.G.GEOSIntersection(bp,K.polygon([[-1000000,la],[1000000,la],[1000000,ha],[-1000000,ha]]))
 lo=max(av[:,common].min(),bv[:,common].min());hi=min(av[:,common].max(),bv[:,common].max())
 if lo>=hi:return {'method':'exact cap integration; separating projection','volume_mm3':0.0}
 lo,hi=(lo-o[common])*1000,(hi-o[common])*1000
 xs=sorted(set([lo,hi]+[float(p[0])for poly in [ap,bp]for loops in K.polys(poly)for rr in loops for p in rr if lo<p[0]<hi]))
 gx,gw=np.polynomial.legendre.leggauss(3);volume=0
 for x0,x1 in zip(xs,xs[1:]):
  for g,w in zip(gx,gw):
   x=(x0+x1)/2+(x1-x0)*g/2;volume+=w*(x1-x0)/2*line_length(ap,x)*line_length(bp,x)
 return {'method':'piecewise Gauss integration of actual triangle-cap cross sections','volume_mm3':volume,'breakpoint_count':len(xs)}
def signed_volume(m):
 v=verts(m);f=faces(m);t=v[f]-v.mean(0);return float(np.einsum('ij,ij->',t[:,0],np.cross(t[:,1],t[:,2]))/6)
def cylinder(name,axis,center,lo,hi,radius,n=192):
 uv=[k for k in range(3)if k!=axis];v=[]
 for h in [lo,hi]:
  for a in np.arange(n)*2*math.pi/n:
   p=list(center);p[axis]=h;p[uv[0]]+=radius*math.cos(a);p[uv[1]]+=radius*math.sin(a);v.append(p)
 for h in [lo,hi]:p=list(center);p[axis]=h;v.append(p)
 f=[]
 for i in range(n):
  j=(i+1)%n;f.extend([[i,j,n+j],[i,n+j,n+i],[2*n,j,i],[2*n+1,n+i,n+j]])
 m={'name':name,'vertices':np.array(v).ravel().tolist(),'indices':np.array(f).ravel().tolist()}
 if signed_volume(m)<0:m['indices']=np.array(f)[:,::-1].ravel().tolist()
 return m
def extrude_poly(name,p,axis,origin,lo,hi):
 uv=[k for k in range(3)if k!=axis];vv=[];ff=[];lookup={}
 def point(q,h):
  a=np.array(origin);a[uv]=a[uv]+np.array(q)/1000;a[axis]=h;key=tuple(a)
  if key not in lookup:lookup[key]=len(vv);vv.append(a.tolist())
  return lookup[key]
 for loops in K.polys(p):
  poly=K.polygon(loops[0],loops[1:])
  for t in K.triangulate(poly):
   q=np.array(t[0][:-1]);area=np.cross(q[1]-q[0],q[2]-q[0]);
   if area<0:q=q[::-1]
   ff.append([point(x,lo)for x in q[::-1]]);ff.append([point(x,hi)for x in q])
  for ri,ring in enumerate(loops):
   q=np.array(ring[:-1]);sgn=np.sum(q[:,0]*np.roll(q[:,1],-1)-q[:,1]*np.roll(q[:,0],-1))
   if (ri==0 and sgn<0)or(ri>0 and sgn>0):q=q[::-1]
   for a,b in zip(q,np.roll(q,-1,axis=0)):
    i,j,k,l=point(a,lo),point(b,lo),point(b,hi),point(a,hi);ff.extend([[i,j,k],[i,k,l]])
 m={'name':name,'vertices':np.array(vv).ravel().tolist(),'indices':np.array(ff).ravel().tolist()}
 if signed_volume(m)<0:m['indices']=np.array(ff)[:,::-1].ravel().tolist()
 return m

def topology(m):
 v=verts(m);f=faces(m);uv,inv=np.unique(v,axis=0,return_inverse=True);f=inv[f];edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1);ue,counts=np.unique(edges,axis=0,return_counts=True);t=uv[f];ar=np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1)/2
 return {'welded_vertices':len(uv),'triangles':len(f),'boundary_edges':int(np.sum(counts==1)),'nonmanifold_edges':int(np.sum(counts!=2)),'min_area_mm2':float(ar.min()*1e6),'volume_mm3':signed_volume(m)*1e9,'closed':bool(np.all(counts==2)),'positive_volume':signed_volume(m)>0}
def witness(m,p):
 t=verts(m)[faces(m)]-p;a,b,c=t[:,0],t[:,1],t[:,2];na=np.linalg.norm(a,axis=1);nb=np.linalg.norm(b,axis=1);nc=np.linalg.norm(c,axis=1);n=np.einsum('ij,ij->i',a,np.cross(b,c));d=na*nb*nc+np.einsum('ij,ij->i',a,b)*nc+np.einsum('ij,ij->i',b,c)*na+np.einsum('ij,ij->i',c,a)*nb
 return float(np.sum(2*np.arctan2(n,d))/(4*np.pi))
def translate(m,delta,name=None):
 out=copy.deepcopy(m);out['vertices']=(verts(m)+delta).ravel().tolist();out['name']=name or m['name'];out['id']=out['name'];return out

def clip_triangles_slab(m,axis,lo,hi):
 t=verts(m)[faces(m)];out=[]
 for tri in t[(t[:,:,axis].max(1)>=lo)&(t[:,:,axis].min(1)<=hi)]:
  for p in tri:
   if lo<=p[axis]<=hi:out.append(p)
  for p,q in zip(tri,np.roll(tri,-1,axis=0)):
   if q[axis]==p[axis]:continue
   for h in [lo,hi]:
    u=(h-p[axis])/(q[axis]-p[axis])
    if 0<=u<=1:out.append(p+u*(q-p))
 return np.asarray(out)
def tube_ring_clearance(tube,ring):
 from scipy.spatial import ConvexHull
 rv=verts(ring);o=(rv.min(0)+rv.max(0))/2;p=clip_triangles_slab(tube,1,rv[:,1].min(),rv[:,1].max());uv=(p[:,[0,2]]-o[[0,2]])*1000;h=ConvexHull(uv);hull=K.polygon(uv[h.vertices].tolist());r=cap(ring,1,o);area=K.area(K.G.GEOSIntersection(hull,r));gap=poly_distance(hull,r)
 return {'method':'conservative convex projection of actual tube triangles clipped to ring Y slab against actual ring cap material','projection_overlap_area_mm2':area,'guaranteed_min_clearance_mm':gap,'positive_volume_overlap_mm3':0.0 if area<1e-12 else None,'tube_clipped_points':len(p)}

K.bind('GEOSPointOnSurface',[K.P],K.P)
def actual_section(m,axis,pos_m,origin):
 """Cross-section from triangle edge/plane intersections, 1e-9 mm snap.
 Nested planar regions are selected by the original 3D solid-angle predicate.
 """
 v=(verts(m)-origin)*1000;f=faces(m);value=(pos_m-origin[axis])*1000;uv=[k for k in range(3)if k!=axis];ts=v[f]
 ids=np.where((ts[:,:,axis].min(1)<value)&(ts[:,:,axis].max(1)>value))[0];lines=[]
 for t in ts[ids]:
  hit=[]
  for a,b in zip(t,np.roll(t,-1,axis=0)):
   if (a[axis]-value)*(b[axis]-value)<0:
    q=a+(b-a)*(value-a[axis])/(b[axis]-a[axis]);hit.append(np.round(q[uv],9).tolist())
  if len(hit)==2 and hit[0]!=hit[1]:lines.append(hit)
 if not lines:return K.read('POLYGON EMPTY')
 node=K.G.GEOSUnaryUnion(K.linework(lines));a=(K.P*1)(node);raw=K.G.GEOSPolygonize(a,1);result=K.read('POLYGON EMPTY')
 for loops in K.polys(raw):
  polygon=K.polygon(loops[0],loops[1:]);pp=K.G.GEOSPointOnSurface(polygon);coord=K.gj(pp)['coordinates'];q=np.array(origin);q[axis]=pos_m;q[uv]=q[uv]+np.array(coord)/1000
  if abs(witness(m,q))>.5:
   new=K.G.GEOSUnion(result,polygon);K.G.GEOSGeom_destroy(result);result=new
  K.G.GEOSGeom_destroy(polygon);K.G.GEOSGeom_destroy(pp)
 K.G.GEOSGeom_destroy(node);K.G.GEOSGeom_destroy(raw)
 return result

def sliced_overlap(a,b,axis=2,epsabs=1e-7):
 from scipy.integrate import quad
 av,bv=verts(a),verts(b);lo=max(av[:,axis].min(),bv[:,axis].min());hi=min(av[:,axis].max(),bv[:,axis].max())
 if hi<=lo:return {'method':'actual triangle slice integration','volume_mm3':0.,'estimated_error_mm3':0.}
 o=(np.maximum(av.min(0),bv.min(0))+np.minimum(av.max(0),bv.max(0)))/2
 xs=sorted(set([lo,hi]+[float(v)for v in np.r_[av[:,axis],bv[:,axis]]if lo<v<hi]));volume=err=0.;count=0;proof=None
 def area(zmm):
  nonlocal count,proof
  pos=o[axis]+zmm/1000;p=actual_section(a,axis,pos,o);q=actual_section(b,axis,pos,o);inter=K.G.GEOSIntersection(p,q);val=K.area(inter);count+=1
  if val>1e-12 and proof is None:
   pp=K.G.GEOSPointOnSurface(inter);coord=K.gj(pp)['coordinates'];point=o.copy();uv=[k for k in range(3)if k!=axis];point[axis]=pos;point[uv]+=np.array(coord)/1000;proof={'point_world_m':point.tolist(),'winding_numbers':[witness(a,point),witness(b,point)]};K.G.GEOSGeom_destroy(pp)
  for obj in [p,q,inter]:K.G.GEOSGeom_destroy(obj)
  return val
 for x0,x1 in zip(xs,xs[1:]):
  val,e=quad(area,(x0-o[axis])*1000,(x1-o[axis])*1000,epsabs=epsabs/max(1,len(xs)),epsrel=1e-7,limit=120);volume+=val;err+=e
 return {'method':'adaptive integration of actual triangle sections, solid-angle region selection; coordinates snapped at 1e-9 mm','volume_mm3':volume,'estimated_error_mm3':err,'slice_evaluations':count,'positive_volume_witness':proof}

def projection_separation(a,b,axes):
 from scipy.spatial import ConvexHull
 av,bv=verts(a),verts(b);o=(av.min(0)+av.max(0))/2
 polygons=[]
 for v in [av,bv]:
  q=(v[:,axes]-o[axes])*1000;h=ConvexHull(q);polygons.append(K.polygon(q[h.vertices].tolist()))
 return {'method':'separating convex projection of actual triangle vertices','axes':axes,'guaranteed_clearance_mm':poly_distance(*polygons),'projected_overlap_area_mm2':K.area(K.G.GEOSIntersection(*polygons))}
