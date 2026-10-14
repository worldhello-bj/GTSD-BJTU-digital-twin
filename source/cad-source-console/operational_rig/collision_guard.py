"""Certified axial separation culling for the two source pin/cell interfaces.

Only a strictly positive compiled-geometry projection gap may reject a pair.
No overlap is suppressed. All remaining pairs use MuJoCo's native contact law.
"""
import weakref
import numpy as np
import mujoco

_STATE=getattr(mujoco,'_gtsd_pin_contact_guard_state',None)
if _STATE is None:
    _STATE={'guards':{},'fallback':None,'dispatcher':None}
    mujoco._gtsd_pin_contact_guard_state=_STATE
_GUARDS=_STATE['guards']

def _dispatch(model,data,g1,g2):
    reference=_GUARDS.get(id(model));guard=reference() if reference else None
    if guard is None:
        if _STATE['fallback'] is not None:return _STATE['fallback'](model,data,g1,g2)
        return int(not ((int(model.geom_contype[g1]) & int(model.geom_conaffinity[g2])) or (int(model.geom_contype[g2]) & int(model.geom_conaffinity[g1]))))
    if not ((guard.contype[g1] & guard.affinity[g2]) or (guard.contype[g2] & guard.affinity[g1])):return 1
    return guard.reject(data,int(g1),int(g2))

if _STATE['dispatcher'] is None:_STATE['dispatcher']=_dispatch

class PinAxialSeparation:
    def __init__(self,model,absolute_margin_m=1e-12):
        self.model=model;self.absolute_margin_m=absolute_margin_m;self.contype=tuple(map(int,model.geom_contype));self.affinity=tuple(map(int,model.geom_conaffinity));self.vertices={};self.bounds={};self.geom_bodies={};self.pins={};self.candidates={}
        for g in range(model.ngeom):
            name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g)
            if name in ('pin_50DF4','pin_51409'):
                self.pins[g]=int(model.geom_bodyid[g]);self.candidates[g]=set()
        for pin in self.pins:
            station=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,pin).split('_')[1]
            self.candidates[pin]={g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').startswith(('OP_CAD_'+station+'_F643','OP_FIT_'+station+'_B535'))}
        for g in set(self.pins)|set().union(*self.candidates.values()):
            mid=int(model.geom_dataid[g]);start=int(model.mesh_vertadr[mid]);count=int(model.mesh_vertnum[mid]);self.vertices[g]=model.mesh_vert[start:start+count].astype(np.float64).copy()
            rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,model.geom_quat[g])
            local_y=self.vertices[g]@rotation.reshape(3,3)[1,:]+model.geom_pos[g,1]
            self.bounds[g]=(float(local_y.min()),float(local_y.max()));self.geom_bodies[g]=int(model.geom_bodyid[g])
        for pin,cells in self.candidates.items():
            pb=self.geom_bodies[pin]
            for cell in cells:
                cb=self.geom_bodies[cell]
                if model.body_parentid[pb]!=model.body_parentid[cb]:raise ValueError('Pin guard requires a shared rigid parent')
                for body in (pb,cb):
                    start=int(model.body_jntadr[body]);count=int(model.body_jntnum[body])
                    if any(model.jnt_type[j]!=mujoco.mjtJoint.mjJNT_SLIDE for j in range(start,start+count)):raise ValueError('Pin guard requires translation-only relative motion')
                    if not np.array_equal(model.body_quat[body],[1,0,0,0]):raise ValueError('Pin guard requires common body orientation')

    def certificate(self,data,pin,cell):
        pb=self.geom_bodies[pin];cb=self.geom_bodies[cell]
        axis=np.asarray(data.xmat[pb])[1::3]
        pp=np.asarray(data.xpos[pb]);cp=np.asarray(data.xpos[cb])
        delta=float((cp-pp)@axis)
        pmin,pmax=self.bounds[pin];cmin,cmax=self.bounds[cell];cmin+=delta;cmax+=delta
        gap=float(max(cmin-pmax,pmin-cmax))
        # Precomputed body-frame intervals are exact for the verified common
        # orientation / translation-only bodies. Include world subtraction,
        # quaternion transforms and projection in this conservative error bound.
        magnitude=max(1.,float(np.abs(pp).sum()+np.abs(cp).sum()+abs(pmin)+abs(pmax)+abs(cmin)+abs(cmax)))
        roundoff=128*np.finfo(np.float64).eps*magnitude
        threshold=self.absolute_margin_m+roundoff
        return {'gap_m':gap,'roundoff_bound_m':float(roundoff),'threshold_m':float(threshold),'strictly_separated':bool(gap>threshold),'axis_world':axis.tolist()}

    def reject(self,data,g1,g2):
        if g1 in self.pins and g2 in self.candidates[g1]:pin,cell=g1,g2
        elif g2 in self.pins and g1 in self.candidates[g2]:pin,cell=g2,g1
        else:return 0
        pb=self.geom_bodies[pin];cb=self.geom_bodies[cell]
        axis=data.xmat[pb][1::3];pp=data.xpos[pb];cp=data.xpos[cb];delta=float((cp-pp)@axis)
        pmin,pmax=self.bounds[pin];cmin,cmax=self.bounds[cell];cmin+=delta;cmax+=delta
        gap=max(cmin-pmax,pmin-cmax)
        if gap<=self.absolute_margin_m:return 0
        magnitude=max(1.,sum(abs(float(x)) for x in pp)+sum(abs(float(x)) for x in cp)+abs(pmin)+abs(pmax)+abs(cmin)+abs(cmax))
        return int(gap>self.absolute_margin_m+128*np.finfo(np.float64).eps*magnitude)

def install(model,absolute_margin_m=1e-12):
    guard=PinAxialSeparation(model,absolute_margin_m);key=id(model)
    def release(reference):
        if _GUARDS.get(key) is reference:_GUARDS.pop(key,None)
    _GUARDS[key]=weakref.ref(guard,release)
    existing=mujoco.get_mjcb_contactfilter()
    if existing is not None and existing is not _STATE['dispatcher']:_STATE['fallback']=existing
    mujoco.set_mjcb_contactfilter(_STATE['dispatcher'])
    return guard
