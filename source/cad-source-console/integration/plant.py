"""Source-aligned rail geometry, independent identity above frozen strict82.

The native vehicle and its uncalibrated parameters are explicitly inherited.
Only the two rail proxy bounds are changed by this variant. New CAD meshes are
never inserted as collision shapes without a separate reviewed contract.
"""
import copy, hashlib, json, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent
BASE=PROJECT/'baseline_console/GTSD_Assembly_Console'
VARIANT='cad_assembly_1013_source_rails_strict82'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def raw_type():
    sys.dont_write_bytecode=True
    path=BASE/'app'
    if str(path)not in sys.path:sys.path.insert(0,str(path))
    import assembly_server as A
    cfg=A.load_config()
    import strict82_server as S
    Parent=S.portable_bootstrap(cfg['strict_config'])
    if str(PROJECT)not in sys.path:sys.path.insert(0,str(PROJECT))
    from frame_track.rail_proxy import augment_xml
    class SourceRailsPlant(Parent):
        def augment_extra_xml(self,xml,metadata):
            xml=super().augment_extra_xml(xml,metadata)
            xml,self.source_rail_spec=augment_xml(xml)
            return xml
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.identity['extensions']['cad_assembly_1013_source_rails']={
                'schema':'gtsd-source-rail-new-physics-identity-v1',
                'adapter_source_sha256':sha(__file__),
                'application':copy.deepcopy(self.source_rail_spec),
                'latest_visual_glb_sha256':cfg['catalogue']['base_glb']['sha256'],
                'old_native_vehicle_geometry_sha256':self.identity['source_geometry_sha256'],
                'boundary':'Only rail extent/center/railhead width follow new source. Vehicle physics, masses and remaining collision proxies are inherited uncalibrated; latest entire CAD assembly is not the solver collision geometry.',
                'old_checkpoints_compatible':False,'hardware_connected':False}
            self.binding['variant']=VARIANT
            self.model_version='cad-assembly-source-rail-geometry-research-1013'
            self.limits += ['1013来源轨道变体：两轨X范围/横向顶面按最新完整GLB核实，代理厚度40mm仍假设；整车其余物理尚未由全部新CAD重建。']
        def snapshot(self):
            s=super().snapshot();s['variant']=VARIANT
            s['source_rail_application']=copy.deepcopy(self.source_rail_spec)
            return s
    return SourceRailsPlant,cfg
