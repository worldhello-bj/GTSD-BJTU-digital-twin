"""Bounded-memory reader for the public scene export. Writes only the chosen output."""
from pathlib import Path
import argparse,gzip,hashlib,json
import numpy as np

def meshes(path):
    decoder=json.JSONDecoder()
    with gzip.open(path,'rt') as f:
        buf=''
        while '"meshes":[' not in buf:
            part=f.read(65536)
            if not part: raise ValueError('No meshes array')
            buf+=part
        prefix,buf=buf.split('"meshes":[',1)
        while True:
            buf=buf.lstrip(' \n\r\t,')
            if buf.startswith(']'): return
            try: row,n=decoder.raw_decode(buf)
            except json.JSONDecodeError:
                part=f.read(1024*1024)
                if not part: raise
                buf+=part;continue
            yield row;buf=buf[n:]

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        while chunk:=f.read(1024*1024):h.update(chunk)
    return h.hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--combined',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=True,parents=True)
    combined=json.load(gzip.open(a.combined,'rt'));refs={r['name']:r['geometry_witness_sha256']for r in combined['mesh_refs']};adds={r['name']:r for r in combined['meshes']};inventory=[];selected=[]
    low=np.array([-9.49,-.50,-.25]);high=np.array([-8.20,1.02,.03])
    for m in meshes(a.source):
        name=m['name']
        if name not in refs:continue
        v=np.asarray(m['vertices']).reshape(-1,3);lo=v.min(0);hi=v.max(0)
        inventory.append(dict(name=name,lo_m=lo.tolist(),hi_m=hi.tolist(),triangles=len(m['indices'])//3,geometry_witness_sha256=m['geometry_witness_sha256']))
        if np.all(hi>=low)and np.all(lo<=high):selected.append(m)
    for m in adds.values():
        v=np.asarray(m['vertices']).reshape(-1,3);lo=v.min(0);hi=v.max(0)
        inventory.append(dict(name=m['name'],lo_m=lo.tolist(),hi_m=hi.tolist(),triangles=len(m['indices'])//3,geometry_witness_sha256=m['geometry_witness_sha256']))
        if np.all(hi>=low)and np.all(lo<=high):selected.append(m)
    assert len(inventory)==len(refs)and len(set(r['name']for r in inventory))==len(refs)
    payload=dict(schema='gtsd-photo-turnout-public-scene-seed-v1',source_sha256=sha(a.source),combined_sha256=sha(a.combined),scene_mesh_count=len(refs),roi_m=[low.tolist(),high.tolist()],full_inventory=inventory,meshes=selected)
    with open(a.output/'source_seed.json.gz','wb')as f:f.write(gzip.compress(json.dumps(payload,separators=(',',':')).encode(),mtime=0))
    print(json.dumps(dict(total=len(refs),selected=len(selected),meshes=[dict(name=r['name'],lo_m=np.asarray(r['vertices']).reshape(-1,3).min(0).tolist(),hi_m=np.asarray(r['vertices']).reshape(-1,3).max(0).tolist(),triangles=len(r['indices'])//3)for r in selected]),indent=2))
if __name__=='__main__':main()
