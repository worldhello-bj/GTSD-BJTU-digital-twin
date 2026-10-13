"""Read-only endpoint world-frame measurements; never advances or writes state.

This view layer is separate from the frozen physics-source identity. Its own
source hash accompanies the measurements. Values are ideal solver observations,
not physical sensor outputs or reconstruction of inaccessible compressor parts.
"""
from pathlib import Path
import hashlib
import mujoco
import numpy as np
from .runtime import ROTORS

def measure_auxiliary_world_state(plant):
    m,d,r=plant.model,plant.data,plant.mount_runtime
    rows={}
    for name in ROTORS:
        bid=m.body(name).id;parent=int(m.body_parentid[bid]);joint=m.joint(name).id
        linear=np.zeros((3,m.nv));rotor_jac=np.zeros((3,m.nv));stator_jac=np.zeros((3,m.nv))
        mujoco.mj_jacBody(m,d,linear,rotor_jac,bid);mujoco.mj_jacBody(m,d,linear,stator_jac,parent)
        wr=rotor_jac@d.qvel;ws=stator_jac@d.qvel;axis=d.xaxis[joint].copy();relative=float(d.qvel[r.dof[name]])
        residual=float(axis@(wr-ws)-relative)
        if abs(residual)>1e-9:raise ArithmeticError('Native auxiliary world/relative angular rate mismatch')
        rows[name]={'rotor_body':name,'stator_body':m.body(parent).name,'axis_world_xyz':axis.tolist(),
            'angular_velocity_rotor_world_rad_s':wr.tolist(),'angular_velocity_stator_world_rad_s':ws.tolist(),
            'angular_velocity_air_reference_world_rad_s':[0.,0.,0.],
            'relative_native_omega_rad_s':relative,'world_minus_stator_axis_residual_rad_s':residual,
            'rotor_position_world_m':d.xpos[bid].tolist(),'rotor_rotation_world':d.xmat[bid].reshape(3,3).tolist()}
    return {'schema':'gtsd-phase6-readonly-world-observations-v1','endpoint_time_s':float(d.time),
        'rows':rows,'source':'IDEAL_NATIVE_SOLVER_OBSERVATIONS_NOT_HARDWARE',
        'observation_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'air_power_note':'World rates here are instantaneous endpoint observations; accepted interval work uses the separately recorded frozen start Jacobian and generalized displacement.'}
