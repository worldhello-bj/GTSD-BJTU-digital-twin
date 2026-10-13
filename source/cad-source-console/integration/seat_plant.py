"""Explicit research variant: source rails + two near-side CAD spring seat sites.

The CAD determines side-view seat locations, not actual spring rate, preload,
installation Y or joint ownership. This mode keeps those prior assumptions
visible and retains the inherited initial axial preload only.
"""
from pathlib import Path
import copy,hashlib
from plant import raw_type as rail_type
VARIANT='cad_assembly_1013_source_rails_near_seats_research'
def raw_type():
    Parent,cfg=rail_type()
    from frame_track.spring_seat import augment_xml,native_site_snapshot
    class CadSeatResearchPlant(Parent):
        def augment_extra_xml(self,xml,metadata):
            xml=super().augment_extra_xml(xml,metadata)
            xml,self.cad_seat_spec=augment_xml(xml,enable_uncalibrated_cad_seat_fit=True)
            return xml
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.identity['extensions']['cad_near_spring_seats']={'adapter_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'application':copy.deepcopy(self.cad_seat_spec),'mass_inertia_stiffness_not_measured':True,'scope':'Only 2 near-side seat sites/restlengths. Far-side unchanged; no new mass, DOF or collision shape.'}
            self.binding['variant']=VARIANT
            self.model_version='cad-source-rails-near-spring-seat-research-1013'
            self.limits+=['近侧CAD弹簧座研究模式：D41/3A44/3A31端点入求解器，保留初始轴向预载；Y安装、运动归属和弹簧参数未实测，远侧未套用。']
        def snapshot(self):
            s=super().snapshot();s['variant']=VARIANT
            s['cad_seat_application']=copy.deepcopy(self.cad_seat_spec)
            s['native_spring_sites']=native_site_snapshot(self.model,self.data)
            return s
    return CadSeatResearchPlant,cfg
