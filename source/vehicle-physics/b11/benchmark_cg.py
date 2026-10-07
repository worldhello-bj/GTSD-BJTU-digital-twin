"""Diagnostic CG trial with the same geometry/forces; not a full acceptance case."""
import json
import simulate

original=simulate.model
def model(*args,**kwargs):
    xml,panto,doors=original(*args,**kwargs)
    return xml.replace('solver="Newton"','solver="CG"').replace('iterations="100"','iterations="500"'),panto,doors
simulate.model=model
metric=simulate.run('profile_cg',dict(torque=False,brake=False,panto=False,settle_s=.1),duration=.02,replay=False)
print(json.dumps({k:metric[k] for k in ('runtime_s','warning_counts','final_speed_mps')},indent=2))
