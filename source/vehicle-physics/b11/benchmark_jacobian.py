"""Numerical profiling only: compare identical mechanics with dense/sparse Jacobians."""
import json
import simulate

original = simulate.model
rows=[]
for mode in ('dense','sparse'):
    def model(*args, **kwargs):
        xml,panto,doors=original(*args,**kwargs)
        return xml.replace('<option ',f'<option jacobian="{mode}" ',1),panto,doors
    simulate.model=model
    m=simulate.run('profile_'+mode,dict(torque=False,brake=False,panto=False,settle_s=.1),duration=.02,replay=False)
    rows.append({k:m[k] for k in ('case','runtime_s','warning_counts','final_speed_mps')})
(simulate.ROOT/'jacobian_benchmark.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print(json.dumps(rows,indent=2))
