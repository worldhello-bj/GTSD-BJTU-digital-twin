"""Use the preinstalled GEOS C library; no third-party package installation."""
import ctypes as C,json,math
G=C.CDLL('libgeos_c.so.1')
def bind(n,args,ret):
 f=getattr(G,n);f.argtypes=args;f.restype=ret;return f
P=C.c_void_p;D=C.c_double;I=C.c_int;U=C.c_uint
bind('initGEOS',[P,P],None)(None,None)
for n,args,ret in [('GEOSWKTReader_create',[],P),('GEOSWKTReader_read',[P,C.c_char_p],P),('GEOSGeoJSONWriter_create',[],P),('GEOSGeoJSONWriter_writeGeometry',[P,P,I],P),('GEOSFree',[P],None),('GEOSGeom_destroy',[P],None),('GEOSIntersection',[P,P],P),('GEOSUnion',[P,P],P),('GEOSDifference',[P,P],P),('GEOSUnaryUnion',[P],P),('GEOSPolygonize',[C.POINTER(P),U],P),('GEOSGeom_setPrecision',[P,D,I],P),('GEOSConstrainedDelaunayTriangulation',[P],P),('GEOSisValid',[P],C.c_char),('GEOSArea',[P,C.POINTER(D)],I)]:bind(n,args,ret)
reader=G.GEOSWKTReader_create();writer=G.GEOSGeoJSONWriter_create()
def read(w):
 p=G.GEOSWKTReader_read(reader,w.encode());assert p,w[:100];return p
def gj(p):
 s=G.GEOSGeoJSONWriter_writeGeometry(writer,p,-1);assert s
 q=json.loads(C.string_at(s));G.GEOSFree(s);return q
def coords(p):return ','.join('%.17g %.17g'%(a,b) for a,b in p)
def polygon(p,holes=[]):
 return read('POLYGON('+','.join('('+coords(x if x[0]==x[-1] else x+[x[0]])+')' for x in [p]+holes)+')')
def linework(lines):
 return read('MULTILINESTRING('+','.join('('+coords(x)+')' for x in lines if len(x)>1)+')')
def polygonize(lines,grid=.001):
 src=linework(lines);snap=G.GEOSGeom_setPrecision(src,grid,1);node=G.GEOSUnaryUnion(snap);a=(P*1)(node);faces=G.GEOSPolygonize(a,1);return G.GEOSUnaryUnion(faces)
def area(p):
 a=D();G.GEOSArea(p,C.byref(a));return a.value
def polys(p):
 def rec(x):
  if x['type']=='Polygon':return [x['coordinates']]
  if x['type']=='MultiPolygon':return x['coordinates']
  if x['type']=='GeometryCollection':return sum((rec(z) for z in x['geometries']),[])
  return []
 return rec(gj(p))
def triangulate(p):
 result=G.GEOSConstrainedDelaunayTriangulation(p)
 try:return polys(result)
 finally:G.GEOSGeom_destroy(result)

# Geometry-equivalence tolerance is expressed in millimetres, then converted to
# a perimeter-based area bound; it is not an arbitrary area-only relaxation.
bind('GEOSLength',[P,C.POINTER(D)],I)
bind('GEOSHausdorffDistance',[P,P,C.POINTER(D)],I)
def equivalence(a,b,xy_tolerance_mm=1e-8):
 h=D();assert G.GEOSHausdorffDistance(a,b,C.byref(h))==1
 la=D();lb=D();assert G.GEOSLength(a,C.byref(la))==1;assert G.GEOSLength(b,C.byref(lb))==1
 x=G.GEOSDifference(a,b);y=G.GEOSDifference(b,a)
 try: delta=area(x)+area(y)
 finally:G.GEOSGeom_destroy(x);G.GEOSGeom_destroy(y)
 bound=(la.value+lb.value)*xy_tolerance_mm+math.pi*xy_tolerance_mm**2+1e-8
 assert h.value<=xy_tolerance_mm,(h.value,xy_tolerance_mm)
 assert delta<=bound,(delta,bound,h.value)
 return {'hausdorff_mm':h.value,'symdiff_mm2':delta,'xy_tolerance_mm':xy_tolerance_mm,'perimeter_area_bound_mm2':bound}
