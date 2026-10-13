"""Read-only mesh loading and triangle-level fit tests. World units are metres.
No CAD semantics are inferred by these tests. A separated result is a numerical
statement about the supplied closed triangulations, not hardware certification.
"""
import gzip, hashlib, itertools, json, struct, sys
from pathlib import Path
import numpy as np

TOL = 1e-8  # 0.00001 mm; accommodates float32 support datum serialization.

def triangle_hash(v, f):
    a = np.asarray(v, dtype='<f4')[f].copy(); a[a == 0] = 0
    order = np.lexsort((a[:, :, 2], a[:, :, 1], a[:, :, 0]), axis=1)
    a = np.take_along_axis(a, order[:, :, None], axis=1).reshape(-1, 9)
    return hashlib.sha256(a[np.lexsort(a.T[::-1])].tobytes()).hexdigest()

def load_base_geometry(root):
    """Hash-check and decode the actual world-space baseline GLB, not a proxy."""
    base=Path(root)/'baseline_console/GTSD_Assembly_Console'
    cat=json.loads((base/'app/data/catalog.json').read_text())
    zipped=(base/'assets/whole.glb.gz').read_bytes()
    assert hashlib.sha256(zipped).hexdigest()==cat['base_glb']['gzip_sha256']
    raw=gzip.decompress(zipped);assert hashlib.sha256(raw).hexdigest()==cat['base_glb']['sha256']
    assert struct.unpack_from('<III',raw)==(0x46546c67,2,len(raw))
    off=12;gl=None;binary=None
    while off<len(raw):
        size,kind=struct.unpack_from('<II',raw,off);off+=8
        if kind==0x4e4f534a:gl=json.loads(raw[off:off+size])
        elif kind==0x004e4942:binary=memoryview(raw)[off:off+size]
        else:raise ValueError('Unexpected GLB chunk')
        off+=size
    assert gl and binary is not None and off==len(raw)
    assert not gl.get('animations') and not gl.get('skins')
    def accessor(index):
        a=gl['accessors'][index];bv=gl['bufferViews'][a['bufferView']]
        assert bv['buffer']==0 and not a.get('sparse')
        width={'VEC3':3,'SCALAR':1}[a['type']]
        dt=np.dtype({5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']])
        offset=bv.get('byteOffset',0)+a.get('byteOffset',0);stride=bv.get('byteStride',dt.itemsize*width)
        assert offset+(a['count']-1)*stride+dt.itemsize*width<=bv.get('byteOffset',0)+bv['byteLength']
        return np.ndarray((a['count'],width),dt,buffer=binary,offset=offset,strides=(stride,dt.itemsize)).copy()
    result={};byname={m['name']:m for m in cat['meshes']}
    for idx in gl['scenes'][gl.get('scene',0)]['nodes']:
        node=gl['nodes'][idx];name=node['name']
        assert name not in result and not any(k in node for k in ['children','matrix','translation','rotation','scale'])
        for key in ['group','assembly_layer','source_metadata','source_refs']:
            if key in byname[name]['source_metadata']:assert node.get('extras',{}).get(key)==byname[name]['source_metadata'][key]
        prims=gl['meshes'][node['mesh']]['primitives'];assert len(prims)==1 and prims[0].get('mode',4)==4
        prim=prims[0];v=accessor(prim['attributes']['POSITION']);f=accessor(prim['indices']).reshape(-1,3)
        assert np.isfinite(v).all() and f.max()<len(v)
        result[name]=(np.stack([v[:,0],-v[:,2],v[:,1]],axis=1).astype(float),f)
    assert set(result)==set(cat['scenes'][cat['default_scene']]['mesh_names'])
    return result,cat

def load_geometry(root):
    geo,cat=load_base_geometry(root)
    rev=Path(root)/'full_source_audit/census/track237_revision'
    for file in [rev/'track_curve_revision.mesh.json',rev/'sleeper_layout_revision.mesh.json',rev/'assembly_fit/assembly_chair_relocation.mesh.json']:
        for r in json.loads(file.read_text())['meshes']:
            geo[r['replaces_mesh_name']]=(np.array(r['world_vertices_m']),np.array(r['triangle_indices'],dtype=int))
    return geo, {m['name']:m for m in cat['meshes']}

def load_effective_scenes(root):
    """Return every exact scene after active source replacements, with hash evidence."""
    root=Path(root);cache,cat=load_base_geometry(root);base=root/'baseline_console/GTSD_Assembly_Console'
    eb=(base/'app/data/scene_extras.json.gz').read_bytes();er=gzip.decompress(eb)
    assert hashlib.sha256(eb).hexdigest()==cat['extras']['gzip_sha256'] and hashlib.sha256(er).hexdigest()==cat['extras']['sha256']
    extras=json.loads(er)
    for m in extras['meshes']:
        assert m['name'] not in cache
        cache[m['name']]=(np.array(m['vertices'],float).reshape(-1,3),np.array(m['indices'],int).reshape(-1,3))
    assert len(cache)==cat['mesh_union_count']
    rb=(root/'integration/data/source_replacements.json').read_bytes();rep=json.loads(rb)
    assert rep['base_glb_sha256']==cat['base_glb']['sha256']
    new={m['name']:(np.array(m['vertices'],float).reshape(-1,3),np.array(m['indices'],int).reshape(-1,3)) for m in rep['meshes']}
    scenes={};counts={}
    for name,selection in cat['scenes'].items():
        geo={n:cache[n] for n in selection['mesh_names']};active=[r for r in rep['replacements'] if r['old_name'] in geo]
        for r in active:
            del geo[r['old_name']]
            for n in r['add_names']:
                if n in geo:raise ValueError('Source replacement duplicates a name')
                geo[n]=new[n]
        scenes[name]=geo;counts[name]={'base':selection['mesh_count'],'source_removed':len(active),'source_added':sum(len(r['add_names']) for r in active),'effective':len(geo)}
    identity={'catalog_sha256':hashlib.sha256((base/'app/data/catalog.json').read_bytes()).hexdigest(),'extras_gzip_sha256':hashlib.sha256(eb).hexdigest(),'source_replacements_sha256':hashlib.sha256(rb).hexdigest(),'base_glb_sha256':cat['base_glb']['sha256'],'source_display_revision':rep['display_revision_sha256'],'default_scene':cat['default_scene'],'scene_counts':counts}
    return scenes,identity

def apply_profiles(geo,profiles):
    """Pure atomic profile application; returns geometry and new-name->old-name map."""
    out=dict(geo);identities={};seen_old=set()
    for profile in profiles:
        removed=profile['remove_names'];added=profile['add_meshes']
        if len(removed)!=len(set(removed)):raise ValueError('Duplicate remove name')
        if not set(removed)<=set(out):raise ValueError('Profile is incomplete or was applied twice')
        if set(removed)&seen_old:raise ValueError('Profiles overlap the same source objects')
        if set(profile.get('additional_mutually_exclusive_names',[]))&set(out):raise ValueError('Both old and source pending variants are present')
        for n in removed:del out[n]
        for m in added:
            n=m['name'];old=m.get('metadata',{}).get('replaces_mesh_name')
            if n in out:raise ValueError('Duplicate added name')
            if old not in removed:raise ValueError('Every added mesh needs an explicit removed predecessor for comparative validation')
            v=np.array(m['vertices'],float).reshape(-1,3);f=np.array(m['indices'],int).reshape(-1,3)
            if not np.isfinite(v).all() or np.any(f<0) or f.max()>=len(v):raise ValueError('Invalid triangle resource')
            out[n]=(v,f);identities[n]=old
        seen_old.update(removed)
    return out,identities

def validate_changed_meshes(before,after,identities):
    """Read-only cross-profile narrowphase. Caller supplies exact effective selection.
    `identities` maps every changed/new mesh name to its original predecessor.
    All changed-vs-all pairs are considered, including pairs across different profiles.
    A previously unresolved before pair does not excuse a new positive after witness.
    """
    rows=[];seen=set()
    for n in identities:
        if n not in after or identities[n] not in before:raise ValueError('Broken predecessor identity')
        for other in after:
            if n==other:continue
            pair=tuple(sorted([n,other]))
            if pair in seen:continue
            seen.add(pair);a,b=after[n][0],after[other][0]
            if not np.all(np.minimum(a.max(0),b.max(0))-np.maximum(a.min(0),b.min(0))>TOL):continue
            ar=classify(after[n],after[other]);old_a=identities.get(n,n);old_b=identities.get(other,other)
            if old_b not in before:raise ValueError('Unknown counterpart predecessor')
            br=classify(before[old_a],before[old_b]) if old_a!=old_b else {'status':'NO_DISTINCT_BEFORE_PAIR'}
            clear=ar['status'] in ['NO_POSITIVE_VOLUME_AABB','TRIANGULATED_MESHES_SEPARATED']
            bad=not clear and br['status']!='POSITIVE_MATERIAL_OVERLAP'
            rows.append({'pair':'|'.join(pair),'before_pair':'|'.join(sorted([old_a,old_b])),'before':br,'after':ar,'new_positive_or_uncleared_interference':bad})
    return {'passes_no_new_interference_gate':not any(r['new_positive_or_uncleared_interference'] for r in rows),'after_positive_aabb_pairs':rows,'new_positive_or_uncleared_interference_pairs':[r for r in rows if r['new_positive_or_uncleared_interference']],'remaining_after_positive_or_uncleared_pairs':[r for r in rows if r['after']['status'] not in ['NO_POSITIVE_VOLUME_AABB','TRIANGULATED_MESHES_SEPARATED']],'tolerance_m':TOL,'claim':'Supplied triangles only; inherited overlaps are reported and not certified away.'}

def load_candidate_geometry(project_root,seed):
    """Portable source-aware builder input using only restored public main assets."""
    root=Path(project_root);scenes,identity=load_effective_scenes(root)
    geo=dict(scenes[identity['default_scene']]);rep=json.loads((root/'integration/data/source_replacements.json').read_text())
    # Preserve one-to-one predecessor keys for the existing geometric contracts.
    # Multi-part replacements retain their real names; they are still checked.
    for r in rep['replacements']:
        if len(r['add_names'])==1 and r['add_names'][0] in geo:
            geo[r['old_name']]=geo.pop(r['add_names'][0])
    for r in seed['chair_source_transforms']:
        n=r['replaces_mesh_name'];v,f=geo[n]
        if triangle_hash(v,f)!=r['old_triangle_sha256']:raise ValueError('Source chair input identity changed: '+n)
        mat=np.array(r['old_baked_world_to_new_baked_world_matrix'])
        nv=v@mat[:3,:3].T+mat[:3,3]
        if triangle_hash(nv,f)!=r['triangle_sha256']:raise ValueError('Native station transform no longer reproduces source candidate: '+n)
        geo[n]=(nv,f)
    cat=json.loads((root/'baseline_console/GTSD_Assembly_Console/app/data/catalog.json').read_text())
    return geo,{m['name']:m for m in cat['meshes']},identity

def inside(v,f,p):
    """Vertical triangle-ray parity with coincident surface hits deduplicated."""
    tri=v[f];a=tri[:,0,:2];b=tri[:,1,:2]-a;c=tri[:,2,:2]-a;d=p[:2]-a
    det=b[:,0]*c[:,1]-b[:,1]*c[:,0];ok=abs(det)>1e-16
    u=np.zeros(len(f));w=u.copy();u[ok]=(d[ok,0]*c[ok,1]-d[ok,1]*c[ok,0])/det[ok]
    w[ok]=(b[ok,0]*d[ok,1]-b[ok,1]*d[ok,0])/det[ok]
    ok &= (u>=-1e-10)&(w>=-1e-10)&(u+w<=1+1e-10)
    z=tri[:,0,2]+u*(tri[:,1,2]-tri[:,0,2])+w*(tri[:,2,2]-tri[:,0,2])
    z=np.unique(np.round(z[ok&(z>p[2]+1e-10)],10))
    return len(z)%2==1

def ray_exit(v,f,p,d):
    t=v[f];e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0]
    h=np.cross(np.broadcast_to(d,e2.shape),e2);det=(e1*h).sum(1);s=p-t[:,0];q=np.cross(s,e1)
    ok=abs(det)>1e-16;u=np.zeros(len(f));w=u.copy();tt=u.copy()
    u[ok]=(s[ok]*h[ok]).sum(1)/det[ok];w[ok]=(q[ok]*d).sum(1)/det[ok];tt[ok]=(e2[ok]*q[ok]).sum(1)/det[ok]
    ok&=(u>=-1e-10)&(w>=-1e-10)&(u+w<=1+1e-10)&(tt>1e-10)
    if not ok.any():raise ValueError('No triangle ray exit from internal witness')
    return float(tt[ok].min())

def overlap_chords(a,b,p):
    p=np.asarray(p,float)
    if not inside(*a,p) or not inside(*b,p):raise ValueError('Witness must be inside both triangle meshes')
    out=[]
    for k in range(3):
        d=np.eye(3)[k];pos=min(ray_exit(*g,p,d) for g in [a,b]);neg=min(ray_exit(*g,p,-d) for g in [a,b])
        out.append({'axis':'XYZ'[k], 'negative_mm':neg*1000,'positive_mm':pos*1000,'shared_material_chord_mm':(pos+neg)*1000})
    return out

def closed(v,f):
    _,inv=np.unique(v,axis=0,return_inverse=True); ff=inv[f]
    e=np.sort(np.concatenate([ff[:,[0,1]],ff[:,[1,2]],ff[:,[2,0]]]),axis=1)
    _,n=np.unique(e,axis=0,return_counts=True)
    return bool(np.all(n==2))

def edge_hit(p,q,t):
    d=q-p;e1=t[1]-t[0];e2=t[2]-t[0];h=np.cross(d,e2);det=np.dot(e1,h)
    if abs(det)<1e-18:return None
    s=p-t[0];u=np.dot(s,h)/det
    if u<-1e-9 or u>1+1e-9:return None
    qq=np.cross(s,e1);v=np.dot(d,qq)/det;tt=np.dot(e2,qq)/det
    if v<-1e-9 or u+v>1+1e-9 or tt<-1e-9 or tt>1+1e-9:return None
    return p+tt*d

def coplanar_touch(a,b):
    n=np.cross(a[1]-a[0],a[2]-a[0]);nn=np.linalg.norm(n)
    if nn<1e-20:return False
    if max(abs((b-a[0])@n))/nn>TOL:return False
    drop=np.argmax(abs(n));a=np.delete(a,drop,axis=1);b=np.delete(b,drop,axis=1)
    for poly in [a,b]:
        for d in np.roll(poly,-1,axis=0)-poly:
            ax=np.array([-d[1],d[0]]);aa=a@ax;bb=b@ax
            if aa.max()<bb.min()-1e-14 or bb.max()<aa.min()-1e-14:return False
    return True

def classify(a,b, find_witness=True):
    v,f=a;w,g=b;lo=np.maximum(v.min(0),w.min(0));hi=np.minimum(v.max(0),w.max(0));dep=hi-lo
    r={'aabb_signed_overlap_mm':(dep*1000).tolist()}
    if np.any(dep<=TOL):
        r.update(status='NO_POSITIVE_VOLUME_AABB',numerical_tolerance_m=TOL,possible_contact_within_tolerance=bool(np.all(dep>=-TOL)));return r
    if find_witness:
        for t in itertools.product([.02,.1,.5,.9,.98],repeat=3):
            p=lo+np.array(t)*dep
            if inside(v,f,p) and inside(w,g,p):
                
                try: chords=overlap_chords(a,b,p)
                except ValueError: continue  # Surface coincidence is not an internal-volume witness.
                r.update(status='POSITIVE_MATERIAL_OVERLAP',witness_world_m=p.tolist(),shared_material_chords=chords);return r
    aa=v[f];bb=w[g];blo=bb.min(1);bhi=bb.max(1);hits=[];cop=0;tested=0
    for t in aa:
        ids=np.where(np.all(bhi>=t.min(0)-TOL,axis=1)&np.all(blo<=t.max(0)+TOL,axis=1))[0]
        for j in ids:
            u=bb[j];tested+=1;got=[]
            for p,q in zip(t,np.roll(t,-1,axis=0)):
                h=edge_hit(p,q,u)
                if h is not None:got.append(h)
            for p,q in zip(u,np.roll(u,-1,axis=0)):
                h=edge_hit(p,q,t)
                if h is not None:got.append(h)
            if got:hits.extend(got)
            elif coplanar_touch(t,u):cop+=1
    containment=[]
    if not hits and not cop:
        for av,af,bv,bf,label in [(v,f,w,g,'A_vertex_inside_B'),(w,g,v,f,'B_vertex_inside_A')]:
            uv=np.unique(av,axis=0);uv=uv[np.all(uv>bv.min(0)+TOL,axis=1)&np.all(uv<bv.max(0)-TOL,axis=1)]
            for p in uv:
                if inside(bv,bf,p):containment.append({'kind':label,'point':p.tolist()});break
    r.update(triangle_aabb_candidates_tested=tested,surface_crossing_count=len(hits),coplanar_touch_count=cop,containment_witnesses=containment,closed_edge_checks=[closed(v,f),closed(w,g)])
    r['status']='TRIANGULATED_MESHES_SEPARATED' if not hits and not cop and not containment and all(r['closed_edge_checks']) else 'SURFACE_OR_CONTAINMENT_REQUIRES_REVIEW'
    return r

def slab_y_bounds(v,f,xmin,xmax):
    """Exact extrema of piecewise planar triangles clipped to an X slab."""
    t=v[f];ids=np.where((t.max(1)[:,0]>=xmin-TOL)&(t.min(1)[:,0]<=xmax+TOL))[0]; pts=[]
    for tri in t[ids]:
        for p in tri:
            if xmin-TOL<=p[0]<=xmax+TOL:pts.append(p)
        for p,q in zip(tri,np.roll(tri,-1,axis=0)):
            if abs(q[0]-p[0])<=TOL:continue
            for x in [xmin,xmax]:
                s=(x-p[0])/(q[0]-p[0])
                if 0<=s<=1:pts.append(p+s*(q-p))
    if not pts:raise ValueError('Rail has no finite triangle geometry in the selected bay')
    a=np.array(pts);return [float(a[:,1].min()),float(a[:,1].max())]
