"""Internal rollback optimization study, separate identity; no equation changes.

External checkpoint restore remains the frozen validated implementation. The
private token below can only be the exact locally captured beginning of the
currently executing accepted-step transaction. It is never exposed as an API.
"""
import copy,hashlib,threading
from pathlib import Path
from aux_mount.plant import MountedAuxiliaryPlant,canonical

class TrustedRollbackMountedPlant(MountedAuxiliaryPlant):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.identity['extensions']['internal_rollback_study']={
          'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'schema':'gtsd-local-rollback-token-study-v1',
          'external_restore_validation':'unchanged',
          'physics_equations':'unchanged',
          'single_owner_only':True}
        self.model_version='phase7-internal-rollback-study-native79'

    def _trial_interval(self,state,aux0):
        aux=self.aux
        if 'restore' in vars(aux):
            raise RuntimeError('Unexpected pre-existing restore override')
        # One full public validation each accepted interval. Do not admit an
        # arbitrary external checkpoint as an internal token.
        if canonical(aux0)!=canonical(aux.checkpoint_state()):
            raise ValueError('Internal token is not the exact current state')
        aux.restore(aux0)
        pristine=copy.deepcopy(aux._state())
        owner=threading.get_ident()
        active=True
        def local_restore(candidate):
            if not active or candidate is not aux0 or threading.get_ident()!=owner:
                raise ValueError('Only the owner-local rollback token is admitted')
            # _load_state assigns nested dictionaries, so each use must get a
            # new deep copy; never alias the private pristine token into gas.
            aux._load_state(copy.deepcopy(pristine))
        aux.restore=local_restore
        try:
            return super()._trial_interval(state,aux0)
        finally:
            active=False  # Revoke even a callable retained by the owner.
            # The caller's exception rollback and every external import must
            # again use the unchanged class method, including all validators.
            del aux.restore
