"""Export compact public research records without dropping raw provenance hashes."""
import argparse,gzip,hashlib,json
from pathlib import Path
from datetime import datetime,timezone,timedelta


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);index=[]
    for scope in ('mount','contact','gravity'):
        src=a.input/scope;dest=a.output/scope;dest.mkdir(exist_ok=True)
        (dest/'report.json').write_bytes((src/'report.json').read_bytes())
        for path in sorted(src.glob('*.json.gz')):
            raw=json.loads(gzip.decompress(path.read_bytes()))
            if 'trace' not in raw:
                packed=path.read_bytes();method='identity'
            else:
                compact=dict(raw)
                compact['trace']=[]
                for row in raw['trace']:
                    record={k:v for k,v in row.items() if k!='state'}
                    record['native_state']={k:row['state'][k] for k in ('steps','time_s','qpos','qvel','energy','ledger','energy_balance_residual_J','native_contact_count','minimum_contact_gap_m')}
                    compact['trace'].append(record)
                compact['full_diagnostic_source_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
                compact['compact_trace_policy']='Per-step loads, qpos/qvel, energies/ledgers, native contact wrenches retained. Redundant per-step world poses and mount forces omitted; exact sampled poses and original complete diagnostic SHA retained.'
                packed=gzip.compress(json.dumps(compact,separators=(',',':'),allow_nan=False).encode(),mtime=0);method='deduplicated_derived_state'
            target=dest/path.name;target.write_bytes(packed)
            index.append({'path':target.relative_to(a.output).as_posix(),'bytes':len(packed),'sha256':hashlib.sha256(packed).hexdigest(),'full_raw_bytes':path.stat().st_size,'full_raw_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'method':method})
    for name in ('partition_rebuild.json','test_end.json'):
        if (a.input/name).exists():(a.output/name).write_bytes((a.input/name).read_bytes())
    now=datetime.now(timezone.utc)
    (a.output/'RECORD_INDEX.json').write_text(json.dumps({'schema':'gtsd-research-records-v1','completed_utc':now.isoformat(),'completed_beijing':now.astimezone(timezone(timedelta(hours=8))).isoformat(),'records':index,'raw_originals_retained_separately':True},indent=2)+'\n')
    print(json.dumps({'records':len(index),'public_bytes':sum(x['bytes'] for x in index),'raw_bytes':sum(x['full_raw_bytes'] for x in index)}))
if __name__=='__main__':main()
