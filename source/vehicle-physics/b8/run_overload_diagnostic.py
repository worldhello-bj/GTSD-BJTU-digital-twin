"""Reproduce the HISTORICAL native-71 OUTSIDE_VALIDATED_DOMAIN stress case.
This deliberately loads frozen source; it is not a current HC77 operational case.
"""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent
FROZEN=ROOT/'contact_diagnostics/frozen_project/vehicle-physics/b8'
sys.path.insert(0,str(FROZEN))
import simulate_full as historical
if __name__=='__main__':
 historical.OUT=ROOT/'historical_overload_rerun';historical.OUT.mkdir(exist_ok=True)
 print('Historical native-71 diagnostic only; forcefully loading frozen source.')
 m=historical.run('overload_800N',dict(torque=True,brake=True,lateral=True,lateral_force_N=800.,yaw_moment_Nm=80.))
 m['validation_status']='OUTSIDE_VALIDATED_DOMAIN'
 m['failure_reason']='Overturn/derailment and excessive work error; no ground collision model. This is not a current HC77 validation.'
 (historical.OUT/'overload_800N_metrics.json').write_text(json.dumps(m,indent=2))
