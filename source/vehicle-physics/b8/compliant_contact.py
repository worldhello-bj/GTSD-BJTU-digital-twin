"""Experimental, explicitly compliant B8 wheel/rail contact (SI units).

Original primitive geometry and mechanical DOFs are preserved. Only the sixteen
native tread/flange rail pairs are removed from a candidate XML copy. Forces act
on the actual wheel body using its point Jacobian, so wheel spin contributes to
friction and the equal wheel torque is retained. Brakes/pantograph remain native.

This is a numerical-compliance study, not a calibrated steel contact model.
Drake references: https://drake.mit.edu/doxygen_cxx/group__compliant__contact.html
and https://drake.mit.edu/doxygen_cxx/group__contact__defaults.html . The linear
point stiffness is an effective compliance; k=1e6 is Drake's stated default.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import sys
from xml.etree import ElementTree as ET
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'vendor'))
import mujoco

@dataclass(frozen=True)
class ContactParameters:
    stiffness_Npm: float = 2e6
    dissipation_spm: float = 40.
    friction_regularization_mps: float = .005
    friction_coefficient: float = .015
    distance_tolerance_m: float = 1e-12
    def __post_init__(self):
        vals=(self.stiffness_Npm,self.dissipation_spm,self.friction_regularization_mps,self.friction_coefficient,self.distance_tolerance_m)
        if not all(np.isfinite(x) for x in vals) or self.stiffness_Npm <= 0 or self.dissipation_spm < 0 or self.friction_regularization_mps <= 0 or self.friction_coefficient < 0 or self.distance_tolerance_m <= 0:
            raise ValueError('finite positive stiffness/regularization and nonnegative dissipation/friction required')

def point_law(gap_m, separation_speed_mps, tangent_velocity_mps, p):
    """N >= 0, zero at gap >= 0. Positive normal speed means separating.

    N=k*delta*(1-alpha*vn)_+; Ft=-mu*N*vt/sqrt(vt.vt+vreg²).
    Dissipation power includes the cutoff branch and is always nonpositive.
    The normal force itself can return spring energy while separating; the
    combined mechanical-plus-spring energy remains dissipative.
    """
    delta=max(0.,-float(gap_m)); elastic=p.stiffness_Npm*delta
    normal=elastic*max(0.,1.-p.dissipation_spm*separation_speed_mps)
    tangent=np.asarray(tangent_velocity_mps,dtype=float)
    friction=-p.friction_coefficient*normal*tangent/np.sqrt(float(tangent@tangent)+p.friction_regularization_mps**2)
    return normal,friction,.5*p.stiffness_Npm*delta**2,(normal-elastic)*separation_speed_mps,float(friction@tangent)

def candidate_xml(original_xml):
    """Remove only explicit original wheel/flange rail pairs; change no geoms."""
    root=ET.fromstring(original_xml); contact=root.find('contact');removed=[]
    for pair in list(contact):
        if pair.tag != 'pair':continue
        geoms=(pair.get('geom1',''),pair.get('geom2',''))
        if any(x.startswith('rail_') for x in geoms) and any('_wheel_' in x or '_flange_' in x for x in geoms):
            contact.remove(pair);removed.append(geoms)
    if len(removed)!=16:raise ValueError(f'Expected 16 native tread/flange pairs, found {len(removed)}')
    return ET.tostring(root,encoding='unicode'),removed

class CompliantWheelRail:
    """One closest-point interaction per original wheel/rail primitive pair.

    No force callback or state override is installed. Call evaluate() once at
    each known position/velocity level before mj_step and copy qforce into the
    caller-owned qfrc_applied. mj_geomDistance is evaluated on unchanged geoms.
    All rails in this specific model are static world-body objects.
    """
    def __init__(self,model,parameters=ContactParameters()):
        self.model=model;self.parameters=parameters;self.pairs=[]
        for car in ('A','B'):
            for axle in ('front','rear'):
                for kind in ('wheel','flange'):
                    for side in ('L','R'):
                        name=f'{car}_{axle}_{kind}_{side}'
                        gid=int(model.geom(name).id);rid=int(model.geom('rail_'+side).id)
                        body=int(model.geom_bodyid[gid])
                        if int(model.geom_bodyid[rid])!=0:raise ValueError('Candidate requires static world rails')
                        if model.geom_contype[gid] or model.geom_conaffinity[gid]:raise ValueError('Automatic native wheel contacts must be disabled')
                        if any(gid in (model.pair_geom1[i],model.pair_geom2[i]) for i in range(model.npair)):raise ValueError('Native contact pair remains for '+name)
                        self.pairs.append((name,gid,rid,body,kind))
        self.jac=np.zeros((3,model.nv));self.fromto=np.zeros(6)
    def evaluate(self,data,details=False):
        # Tighten only this external geometric query; native brake/pantograph
        # collision calculations retain their original tolerance. Do not raise
        # ccd_iterations here: 3.3.7 distance calls with 1000 can crash.
        previous=float(self.model.opt.ccd_tolerance)
        self.model.opt.ccd_tolerance=self.parameters.distance_tolerance_m
        try:
            return self._evaluate_at_tolerance(data,details)
        finally:
            self.model.opt.ccd_tolerance=previous
    def _evaluate_at_tolerance(self,data,details=False):
        qforce=np.zeros(self.model.nv); normals=np.zeros(16);gaps=np.zeros(16)
        storage=normal_diss=friction_work_rate=normal_power=0.;records=[]
        total_force=np.zeros(3);wheel_force=np.zeros(3);flange_force=np.zeros(3)
        kind_friction={"wheel":0.,"flange":0.}
        for i,(name,gid,rid,body,kind) in enumerate(self.pairs):
            # A zero query bound requests penetration only. Positive distmax
            # is explicitly documented as inaccurate for general convex pairs
            # in this MuJoCo version. Separated gaps are reported as zero; no
            # artificial contact margin or penetration clipping is applied.
            gap=float(mujoco.mj_geomDistance(self.model,data,gid,rid,0.,self.fromto));gaps[i]=gap
            if gap>=0:continue
            vec=self.fromto[:3]-self.fromto[3:];norm=float(np.linalg.norm(vec))
            if norm==0:raise RuntimeError('Penetrating pair returned coincident distance witnesses: '+name)
            # Witness order reverses under penetration; signed division is essential.
            n=vec*(-1./norm)
            point=(self.fromto[:3]+self.fromto[3:])*.5
            mujoco.mj_jac(self.model,data,self.jac,None,point,body)
            velocity=self.jac@data.qvel;vn=float(velocity@n);vt=velocity-vn*n
            N,Ft,U,D,F=point_law(gap,vn,vt,self.parameters)
            force=N*n+Ft;qforce+=self.jac.T@force;normals[i]=N
            total_force+=force
            if kind=='wheel':wheel_force+=force
            else:flange_force+=force
            storage+=U;normal_diss+=D;friction_work_rate+=F;normal_power+=N*vn;kind_friction[kind]+=F
            if details:records.append(dict(name=name,kind=kind,gap_m=gap,point_m=point.tolist(),normal_on_wheel=n.tolist(),force_on_wheel_N=force.tolist(),normal_force_N=N,separation_speed_mps=vn,tangent_speed_mps=float(np.linalg.norm(vt)),friction_power_W=F,normal_dissipation_power_W=D,spring_storage_J=U))
        return dict(total_force_world_N=total_force,wheel_force_world_N=wheel_force,flange_force_world_N=flange_force,qforce=qforce,normals_N=normals,gaps_m=gaps,spring_storage_J=storage,normal_dissipation_power_W=normal_diss,friction_power_W=friction_work_rate,normal_power_W=normal_power,wheel_friction_power_W=kind_friction['wheel'],flange_friction_power_W=kind_friction['flange'],details=records)
