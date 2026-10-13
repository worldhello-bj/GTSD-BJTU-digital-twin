"""Independent strict-restore identity; frozen82 equations untouched.

Frozen82 accepts re-signed mutations of two used-speed diagnostic dictionaries.
This wrapper refuses those mutations atomically by linking the stored diagnostics
to the actually applied bearing/motor/air component efforts. Used fixed-point
speeds are not falsely equated to endpoint qvel or accepted displacement/dt.
"""
import math,copy
from pathlib import Path
import numpy as np
import bootstrap
from fan82.plant import FourDoorFansPlant
from fan82.model import KEYS
from fan82.runtime import COLUMNS

def validate_derived_speeds(plant):
 r=plant.extra_fans
 if not r.steps:return
 last=r.last;p=r.parameters;finite=lambda x:isinstance(x,(int,float)) and not isinstance(x,(bool,np.bool_)) and math.isfinite(x)
 close=lambda a,b:abs(a-b)<=5e-12*max(1.,abs(a),abs(b))
 for field in ('relative_speed_rad_s','air_speed_rad_s'):
  v=last[field]
  if type(v) is not dict or set(v)!=set(KEYS) or any(not finite(x) for x in v.values()):raise ValueError('Strict82 derived speed schema: '+field)
 bearing=np.asarray(last['component_generalized_forces'][COLUMNS[1]],dtype=float)
 for k in KEYS:
  used_rel=last['relative_speed_rad_s'][k]
  expected_rel=-float(bearing[r.dof[k]])/p.bearing_drag_Nm_s
  if not close(used_rel,expected_rel):raise ValueError('Strict82 relative-speed/applied-bearing mismatch: '+k)
  controls=last['controls'][k]
  expected_motor=0. if controls['power_failed'] else p.torque_back_emf_constant*(p.voltage_V*controls['voltage_fraction']-p.torque_back_emf_constant*used_rel)/p.resistance_ohm
  if not close(last['motor_torque_Nm'][k],expected_motor):raise ValueError('Strict82 relative-speed/applied-motor mismatch: '+k)
  air_tau=last['air_torque_Nm'][k]
  expected_air=-math.copysign(math.sqrt(abs(air_tau)/p.aerodynamic_drag_Nm_s2),air_tau)
  if not close(last['air_speed_rad_s'][k],expected_air):raise ValueError('Strict82 air-speed/applied-drag mismatch: '+k)
 # Frozen82 already reconstructs full projection from native interval-start
 # pose, checks applied effort laws vs accepted displacement/dt within 2e-9,
 # and checks motor work vs accepted displacement within 1e-10 J.

class StrictFourDoorFansPlant(FourDoorFansPlant):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.identity['extensions']['strict_four_fan_restore']={'schema':'gtsd-four-fan-strict-derived-restore-v1','source_sha256':bootstrap.sha_file(Path(__file__)),'bootstrap_sha256':bootstrap.sha_file(bootstrap.ROOT/'bootstrap.py'),'frozen82_source_freeze_sha256':bootstrap.BASE_FREEZE_SHA,'scope':'Extra-fan applied relative/air speed diagnostics validated, existing public guards retained','physical_equations_changed':False}
  self.model_version='cad-four-door-fans82-strict-restore-research';self.binding['variant']='cad_doors_four_fans_strict_restore'
 def _validate_extra_fans(self):
  super()._validate_extra_fans();validate_derived_speeds(self)
 def snapshot(self):
  value=super().snapshot();value['variant']='cad_doors_four_fans_strict_restore';return value
