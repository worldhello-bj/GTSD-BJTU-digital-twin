"""Explicit CAD-body variant; defaults and frozen Phase6 source are untouched."""
import copy,hashlib,json
from pathlib import Path
from . import bootstrap
from aux_mount.plant import MountedAuxiliaryPlant
from .model import augment_xml
from .geometry import patch_binding,resource,visual_resource_bytes

class CadBodyPlant(MountedAuxiliaryPlant):
    def augment_extra_xml(self,xml,metadata):
        xml=super().augment_extra_xml(xml,metadata)
        xml,self.cad_body_spec,self.support_spec=augment_xml(xml,self.support_spec)
        return xml

    def __init__(self,dt=.0002,*,include_cables=True,brake_parameters=None,support_parameters=None,auxiliary_parameters=None):
        super().__init__(dt,include_cables=include_cables,brake_parameters=brake_parameters,support_parameters=support_parameters,auxiliary_parameters=auxiliary_parameters)
        data,geometry_sha,overlay,_=resource()
        self.model_version='cad-body-79dof-mounted-research'
        self.binding=patch_binding(self.binding,self.model,self.data)
        self.binding['cad_body_augmentation']=copy.deepcopy(self.cad_body_spec)
        self.identity['schema']='gtsd-cad-body-mounted-model-identity-v1'
        self.identity['source_geometry_sha256']=geometry_sha
        self.identity['extensions'].update(cad_body=copy.deepcopy(self.cad_body_spec),geometry_overlay=copy.deepcopy(overlay),readonly_dependencies=copy.deepcopy(bootstrap.MANIFEST))
        self.identity['extensions']['source_sha256'].update({'cad_body/'+p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__).parent/n for n in ('__init__.py','bootstrap.py','geometry.py','model.py','plant.py')]})
        for name in ('READONLY_DEPENDENCIES.json','replacement_manifest.json'):
            self.identity['extensions']['source_sha256'][name]=hashlib.sha256((bootstrap.ROOT/name).read_bytes()).hexdigest()
        self.mass_ledger.update(cad_body_reallocation=copy.deepcopy(self.cad_body_spec),boundary=self.cad_body_spec['mass_boundary'])
        # Historical Phase5/6 subtraction ledgers are retained as provenance, not final COM/inertia truth.
        self.mass_ledger['historical_allocation_note']='door_allocation and auxiliary_allocation explain prior mass debits; their original spatial moments are superseded by cad_body_reallocation.'
        self.mass_ledger['reference_com_world_m']=self.cad_body_spec['reference_spatial_after']['com_world_m']
        self.mass_ledger['reference_inertia_about_com_world_kgm2']=self.cad_body_spec['reference_spatial_after']['inertia_world_com_kgm2']
        self.limits.append('CAD截面车体已重算完整惯量并接入底板支承；门、辅机安装及轮轨滚动角色仍为旧候选，未由该截面证实。')
        self.capabilities.update(cad_body_inertia=True,cad_body_visual_geometry=True,cad_floor_native_support=True,cad_all_mounts_certified=False)

    def visual_resource_bytes(self):return visual_resource_bytes()

    def snapshot(self):
        value=super().snapshot()
        value['variant']='cad_body_mounted';value['source_geometry_sha256']=resource()[1]
        if hasattr(self,'cad_body_spec'):
            value['telemetry']['cad_body']={'residual_mass_kg':self.cad_body_spec['residual_mass_kg'],
              'residual_com_body_local_m':self.cad_body_spec['residual_com_body_local_m'],
              'residual_inertia_com_body_local_kgm2':self.cad_body_spec['residual_inertia_com_body_local_kgm2'],
              'cad_width_m':self.cad_body_spec['cad_width_m'],'rail_contour_center_distance_m':self.cad_body_spec['rail_contour_center_distance_m'],
              'mounting_status':'PARTIAL_CAD_MIGRATION_REMAINING_INSTALLATIONS_UNVERIFIED','material_density_calibrated':False}
        return value
