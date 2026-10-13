"""Single new-vehicle solve with native brake pistons/discs and cable supports.

Braking and gas state remain solely owned by IntegratedBrakePlant. Source
support collision geometry adds no mass/DOF. Audit runtime never applies force.
"""
import copy
import hashlib
from pathlib import Path
import mujoco
import numpy as np
from brake_integrated.plant import IntegratedBrakePlant, STATE_KIND, sha, canonical
from .model import augment_xml, Parameters
from .runtime import SupportContactRuntime
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent

class CoupledInternalPlant(IntegratedBrakePlant):
    def __init__(self,dt=.0002,*,include_cables=True,brake_parameters=None,support_parameters=None):
        if include_cables is not True:raise ValueError('combined internal model requires cables')
        self.support_parameters=Parameters(**support_parameters) if isinstance(support_parameters,dict) else support_parameters or Parameters()
        super().__init__(dt,include_cables=True,brake_parameters=brake_parameters)
        self.support=SupportContactRuntime(self.model,self.data,self.support_spec)
        self.command_registry.update(self.support.command_registry)
        self.model_version='phase5-coupled-native-brake-cable-support-new-geometry'
        self.identity['extensions']['support_spec']=copy.deepcopy(self.support_spec)
        self.identity['extensions']['support_runtime_identity']=copy.deepcopy(self.support.identity)
        self.identity['extensions']['source_sha256'].update({str(p.relative_to(SOURCE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'model.py',ROOT/'runtime.py',ROOT/'combined.py')})
        self.capabilities.update(cable_support_native_contact=True,cable_support_vehicle_reactions=True)
        self.binding['support_augmentation']=copy.deepcopy(self.support_spec)
        self.mass_ledger['cable_support_incremental_mass_kg']=0.
        self.mass_ledger['cable_support_boundary']=self.support_spec['mass_boundary']

    def augment_extra_xml(self,xml,metadata):
        xml,self.support_spec=augment_xml(xml,metadata['cables'],self.support_parameters)
        metadata['cables']['limits']=[x for x in metadata['cables']['limits'] if not x.startswith('Numerical ground plane')]+['No numerical world floor. Selected bay route contacts move with the existing payload body through conservative source support proxies.']
        return xml

    def command(self,name,args):
        if name=='support_contact':
            if self.failed:raise ValueError('plant stopped after rejected interval')
            return self.support.command(name,args)
        return super().command(name,args)

    def _step(self):
        state=self._mechanical_state();aux=copy.deepcopy(self.aux.checkpoint_state())
        dynamic=copy.deepcopy(self._dynamic());motors=copy.deepcopy(self.motors)
        heat=self.brake.brake_heat_J;cp=self.support.checkpoint()
        warnings=[(w.number,w.lastinfo) for w in self.data.warning]
        try:
            super()._step()
            # Match this host's discrete native constraint work quadrature.
            virtual_velocity=np.asarray(self.last_native_brake['generalized_displacement'])/self.dt
            self.support.capture_interval(state,self.data,virtual_velocity,expected_constraint=self.last_constraint_generalized)
        except Exception:
            mujoco.mj_setState(self.model,self.data,state,STATE_KIND)
            self.aux.restore(aux);self.motors=motors;self.brake.brake_heat_J=heat
            for name,value in dynamic.items():setattr(self,name,value)
            self.support.restore(cp)
            warm=self.data.qacc_warmstart.copy();mujoco.mj_forward(self.model,self.data);self.data.qacc_warmstart[:]=warm
            for w,(number,lastinfo) in zip(self.data.warning,warnings):w.number,w.lastinfo=number,lastinfo
            raise

    def snapshot(self):
        value=super().snapshot()
        if hasattr(self,'support'):
            value['telemetry']['cable_support']=self.support.snapshot()
            value['telemetry']['cable_support']['virtual_velocity_convention']='DISCRETE_GENERALIZED_DISPLACEMENT_OVER_DT; point Jacobians from accepted solve'
            value['telemetry']['cable_support']['work_boundary']='Subset of the existing signed_solver_constraint column; do not add it a second time. Normal and tangential support work are signed numerical contact work, not asserted heat.'
            value['variant']='coupled_internal';value['binding_subsystem']='integrated'
        return value

    def checkpoint(self):
        payload={'schema':'gtsd-phase5-coupled-internal-checkpoint-v1','plant':super().checkpoint(),'support':self.support.checkpoint()}
        return payload|{'sha256':sha(payload)}

    def restore(self,value):
        if type(value) is not dict or set(value)!={'schema','plant','support','sha256'} or value['schema']!='gtsd-phase5-coupled-internal-checkpoint-v1':raise ValueError('combined checkpoint schema mismatch')
        canonical(value)
        if value['sha256']!=sha({k:v for k,v in value.items() if k!='sha256'}):raise ValueError('combined checkpoint checksum mismatch')
        candidate=type(self)(self.dt,include_cables=True,brake_parameters=self.brake_parameters,support_parameters=self.support_parameters)
        candidate._restore_validated_candidate(value['plant'])
        candidate.support.restore(value['support'])
        if candidate.steps!=candidate.support.steps:raise ValueError('combined host/support audit count mismatch')
        native=np.asarray(value['plant']['mujoco_integration_state'],dtype=float)
        mujoco.mj_forward(candidate.model,candidate.data)
        mujoco.mj_setState(candidate.model,candidate.data,native,STATE_KIND)
        if not np.array_equal(candidate._mechanical_state(),native):raise ValueError('combined exact native restore failed')
        self.__dict__.clear();self.__dict__.update(candidate.__dict__)
