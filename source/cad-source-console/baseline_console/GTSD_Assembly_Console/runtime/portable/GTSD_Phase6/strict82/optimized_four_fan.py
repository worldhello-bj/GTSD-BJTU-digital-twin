"""Audited trusted rollback + strict82 via MRO, without copying solver code."""
from pathlib import Path
import bootstrap
from strict_four_fan import StrictFourDoorFansPlant
from trusted_rollback import TrustedRollbackMountedPlant

class TrustedRollbackFourDoorFansPlant(TrustedRollbackMountedPlant,StrictFourDoorFansPlant):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.identity['extensions']['four_fan_rollback_composition']={'schema':'gtsd-four-fan-scoped-rollback-composition-v1','adapter_source_sha256':bootstrap.sha_file(Path(__file__)),'trusted_source_sha256':bootstrap.TRUSTED_SHA,'strict_source_sha256':bootstrap.sha_file(bootstrap.ROOT/'strict_four_fan.py'),'mro':[c.__module__+'.'+c.__name__ for c in type(self).__mro__],'public_restore':'Full frozen82 semantic validation plus strict used-speed consistency; never token bypass','internal_restore':'Exact current local token; one public validation each interval; owner thread and active scope; revoked finally','physics_equations_changed':False}
  self.model_version='cad-four-door-fans82-strict-scoped-rollback-research';self.binding['variant']='cad_doors_four_fans_scoped_rollback'
 def snapshot(self):
  value=super().snapshot();value['variant']='cad_doors_four_fans_scoped_rollback';return value
