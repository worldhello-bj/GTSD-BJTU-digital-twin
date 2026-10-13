"""Read native moving-support contact forces; never reapply their reactions.

The host owns integration and native state. Capture immediately after mj_step,
before an endpoint mj_forward replaces the accepted interval's constraint solve.
Alternatively capture_interval reconstructs that one accepted solve in detached
MjData, checks exact endpoint equality, and does not touch the live solver.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from .model import digest

STATE = mujoco.mjtState.mjSTATE_INTEGRATION

def finite(v):
    return type(v) in (int, float) and math.isfinite(v)

def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)

class SupportContactRuntime:
    def __init__(self, model, data, spec):
        self.model, self.data, self.spec = model, data, copy.deepcopy(spec)
        self.support_geoms = {model.geom(r['proxy_geom']).id for r in spec['supports']}
        self.cable_geoms = {model.geom(n).id for n in spec['cable_geoms']}
        self.support_body = model.body(spec['binding_body']).id
        if any(int(model.geom_bodyid[g]) != self.support_body for g in self.support_geoms):
            raise ValueError('support geometry is not bound to the declared vehicle body')
        self.enabled = True; self.events = []; self.steps = 0
        self.impulse_world_Ns = np.zeros(3); self.angular_impulse_about_support_com_Nms = np.zeros(3)
        self.signed_virtual_work_J = 0.; self.signed_tangential_work_J = 0.; self.last = None
        self.identity = dict(schema='gtsd-v5-integrated-cable-support-runtime-v1',
            spec_sha256=digest(spec), dimensions=[model.nq, model.nv, model.nbody, model.ngeom],
            dt_s=float(model.opt.timestep), acceptance={'force_projection_abs_N':1e-8,'force_projection_rel':1e-10,'power_projection_abs_W':1e-9,'power_projection_rel':1e-10,'replay_max_state_error':1e-12}, source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (Path(__file__), Path(__file__).with_name('model.py'))})
        self.command_registry = {'support_contact': {'label':'车体线缆支承接触开关（工程等效）',
            'args':{'enabled':{'type':'boolean'}}}}
        self.controls = self.command_registry
        self._scratch_data = mujoco.MjData(model)

    def _set_enabled(self, enabled):
        for gid in self.support_geoms:
            self.model.geom_contype[gid] = self.spec['parameters']['collision_bit'] if enabled else 0
            self.model.geom_conaffinity[gid] = 0
        self.enabled = enabled

    def command(self, name, args):
        if name != 'support_contact' or type(args) is not dict or set(args) != {'enabled'} or type(args['enabled']) is not bool:
            raise ValueError('support_contact requires exactly one boolean enabled')
        if self.enabled != args['enabled']:
            self._set_enabled(args['enabled'])
            self.events.append({'time_s':float(self.data.time), 'enabled':self.enabled})

    def sample_native(self, data, velocity):
        """Audit selected cable/support pairs from this solver evaluation."""
        m, d = self.model, data
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (m.nv,) or not np.isfinite(velocity).all():
            raise ValueError('finite full generalized velocity required')
        Q = np.zeros(m.nv); Qs = np.zeros(m.nv); Qc = np.zeros(m.nv)
        row_effort = np.zeros(d.nefc); records = []
        force_sum = np.zeros(3); torque_sum = np.zeros(3)
        virtual_power = 0.; tangential_power = 0.; min_dist = 0.
        for i in range(d.ncon):
            c = d.contact[i]; g1, g2 = int(c.geom1), int(c.geom2)
            selected = ((g1 in self.support_geoms and g2 in self.cable_geoms) or
                        (g2 in self.support_geoms and g1 in self.cable_geoms))
            if not selected or c.efc_address < 0: continue
            b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
            f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f)
            R = np.asarray(c.frame).reshape(3, 3).T
            F, T = R @ f[:3], R @ f[3:]
            jp1 = np.zeros((3, m.nv)); jr1 = np.zeros_like(jp1)
            jp2 = np.zeros_like(jp1); jr2 = np.zeros_like(jp1)
            mujoco.mj_jac(m, d, jp1, jr1, c.pos, b1)
            mujoco.mj_jac(m, d, jp2, jr2, c.pos, b2)
            q1, q2 = -jp1.T @ F - jr1.T @ T, jp2.T @ F + jr2.T @ T
            on_second = g2 in self.support_geoms
            Fs, Ts = (F, T) if on_second else (-F, -T)
            qs, qc = (q2, q1) if on_second else (q1, q2)
            torque_com = Ts + np.cross(c.pos - d.xipos[self.support_body], Fs)
            dv, dw = (jp2-jp1) @ velocity, (jr2-jr1) @ velocity
            power = float(F @ dv + T @ dw)
            tp = float(f[1:3] @ (R.T @ dv)[1:3])
            Q += q1+q2; Qs += qs; Qc += qc
            force_sum += Fs; torque_sum += torque_com
            virtual_power += power; tangential_power += tp; min_dist = min(min_dist, float(c.dist))
            a = int(c.efc_address)
            # Elliptic has dim rows, pyramidal has 2*(dim-1). The native row
            # projection handles either sparse or dense Jacobian storage.
            rows = int(c.dim) if m.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC or c.dim == 1 else 2*(int(c.dim)-1)
            row_effort[a:a+rows] = d.efc_force[a:a+rows]
            records.append(dict(geom1=m.geom(g1).name, geom2=m.geom(g2).name,
                support_body=m.body(self.support_body).name, distance_m=float(c.dist),
                position_world_m=c.pos.tolist(), contact_frame_rows=np.asarray(c.frame).tolist(),
                force_on_geom2_world_N=F.tolist(), force_on_geom1_world_N=(-F).tolist(),
                torque_on_geom2_world_Nm=T.tolist(), force_on_support_world_N=Fs.tolist(),
                torque_on_support_about_com_world_Nm=torque_com.tolist(),
                relative_point_velocity_world_m_s=dv.tolist(), relative_angular_velocity_world_rad_s=dw.tolist(),
                virtual_power_W=power, tangential_signed_power_W=tp,
                power_on_support_W=float(qs @ velocity), power_on_cable_W=float(qc @ velocity)))
        row_Q = np.zeros(m.nv)
        if d.nefc: mujoco.mj_mulJacTVec(m, d, row_Q, row_effort)
        return dict(contact_count=len(records), contacts=records,
            support_force_world_N=force_sum.tolist(), support_torque_about_com_world_Nm=torque_sum.tolist(),
            contact_generalized_force=Q.tolist(), support_generalized_force=Qs.tolist(),
            cable_generalized_force=Qc.tolist(), native_contact_row_generalized_force=row_Q.tolist(),
            contact_generalized_force_projection_error_N=float(np.linalg.norm(Q-row_Q)),
            virtual_power_W=virtual_power, tangential_signed_power_W=tangential_power,
            support_power_W=float(Qs @ velocity), cable_power_W=float(Qc @ velocity),
            virtual_power_projection_error_W=abs(float(Q @ velocity)-virtual_power),
            min_contact_distance_m=min_dist, enabled=self.enabled)

    def commit_native(self, data, velocity, *, ending_time_s=None, replay_error=0.):
        """Call exactly once for an accepted host interval, with native solve."""
        now = float(data.time) if ending_time_s is None else float(ending_time_s)
        h = float(self.model.opt.timestep)
        if abs(now - (self.steps+1)*h) > 1e-8:
            raise ValueError('support audit must commit every accepted host interval exactly once')
        result = self.sample_native(data, velocity)
        result.update(step=self.steps+1, time_s=now, dt_s=h,
            accepted_interval_replay_max_state_error=float(replay_error),
            signed_virtual_work_step_J=h*result['virtual_power_W'],
            signed_tangential_work_step_J=h*result['tangential_signed_power_W'])
        _canonical(result)
        self._guard_audit(result)
        self.steps += 1
        self.impulse_world_Ns += h*np.asarray(result['support_force_world_N'])
        self.angular_impulse_about_support_com_Nms += h*np.asarray(result['support_torque_about_com_world_Nm'])
        self.signed_virtual_work_J += result['signed_virtual_work_step_J']
        self.signed_tangential_work_J += result['signed_tangential_work_step_J']
        self.last = result
        return copy.deepcopy(result)

    def capture_interval(self, start_state, end_data, velocity, *, expected_constraint=None):
        """One detached replay preserves the actual solve before host forward.

        The accepted step's effort arrays survive the endpoint mj_forward; copy
        those inputs into its beginning integration state. Abort rather than
        label a different solve as the accepted contact if endpoints disagree.
        """
        m, trial = self.model, self._scratch_data
        mujoco.mj_setState(m, trial, np.asarray(start_state, dtype=float), STATE)
        for field in ('ctrl', 'qfrc_applied', 'xfrc_applied'):
            getattr(trial, field)[:] = getattr(end_data, field)
        mujoco.mj_forward(m, trial); mujoco.mj_step(m, trial)
        error = max(float(np.max(abs(trial.qpos-end_data.qpos))), float(np.max(abs(trial.qvel-end_data.qvel))))
        if error > 1e-12: raise RuntimeError('support accepted interval replay differs: ' + str(error))
        if expected_constraint is not None and not np.allclose(trial.qfrc_constraint, expected_constraint, rtol=1e-12, atol=1e-12):
            raise RuntimeError('support accepted constraint solve differs')
        return self.commit_native(trial, velocity, ending_time_s=float(end_data.time), replay_error=error)

    def snapshot(self):
        return dict(schema='gtsd-v5-integrated-cable-support-snapshot-v1', identity=copy.deepcopy(self.identity),
            enabled=self.enabled, steps=self.steps, binding_body=self.spec['binding_body'],
            contact_switch_events=copy.deepcopy(self.events),
            contact_impulse_on_vehicle_world_Ns=self.impulse_world_Ns.tolist(),
            contact_angular_impulse_about_moving_support_com_Nms=self.angular_impulse_about_support_com_Nms.tolist(),
            signed_contact_virtual_work_J=self.signed_virtual_work_J,
            signed_tangential_work_J=self.signed_tangential_work_J, last=copy.deepcopy(self.last),
            interpretation='Native contact reaction already belongs to vehicle equations. Audit is a decomposition of native signed constraint work, never additional applied force or work.',
            limits=copy.deepcopy(self.spec['limits']))

    def checkpoint(self):
        payload = dict(schema='gtsd-v5-integrated-cable-support-checkpoint-v1', identity=copy.deepcopy(self.identity),
            enabled=self.enabled, events=copy.deepcopy(self.events), steps=self.steps,
            impulse_world_Ns=self.impulse_world_Ns.tolist(),
            angular_impulse_about_support_com_Nms=self.angular_impulse_about_support_com_Nms.tolist(),
            signed_virtual_work_J=self.signed_virtual_work_J,
            signed_tangential_work_J=self.signed_tangential_work_J, last=copy.deepcopy(self.last))
        return payload | {'sha256':digest(payload)}

    def restore(self, value):
        """Validate all metadata before changing contact bits or audit state.

        Host first restores native state in a detached candidate. Successful
        runtime restore only owns its geometry masks and this audit history.
        """
        expected = set(self.checkpoint())
        if type(value) is not dict or set(value) != expected or value['schema'] != 'gtsd-v5-integrated-cable-support-checkpoint-v1' or value['identity'] != self.identity:
            raise ValueError('support checkpoint schema/identity mismatch')
        try:
            if digest({k:v for k,v in value.items() if k != 'sha256'}) != value['sha256']: raise ValueError('support checkpoint checksum mismatch')
        except (TypeError, OverflowError): raise ValueError('support checkpoint encoding invalid')
        if type(value['enabled']) is not bool or type(value['steps']) is not int or value['steps'] < 0:
            raise ValueError('invalid support checkpoint input/count')
        if abs(float(self.data.time) - value['steps']*self.model.opt.timestep) > 1e-8:
            raise ValueError('support checkpoint/host solver time mismatch')
        for key in ('impulse_world_Ns', 'angular_impulse_about_support_com_Nms'):
            if type(value[key]) is not list or len(value[key]) != 3 or not all(finite(x) for x in value[key]): raise ValueError('invalid support impulse')
        for key in ('signed_virtual_work_J', 'signed_tangential_work_J'):
            if not finite(value[key]): raise ValueError('invalid support work')
        if type(value['events']) is not list: raise ValueError('invalid support events')
        active = True; previous = -1.
        for event in value['events']:
            if type(event) is not dict or set(event) != {'time_s','enabled'} or not finite(event['time_s']) or event['time_s'] < 0 or event['time_s'] < previous or event['time_s'] > self.data.time+1e-12 or type(event['enabled']) is not bool or event['enabled'] == active:
                raise ValueError('invalid support toggle history')
            active = event['enabled']; previous = event['time_s']
        if active != value['enabled']: raise ValueError('support toggle final state mismatch')
        last = value['last']
        if value['steps'] == 0:
            if last is not None or any(value[k] != 0 for k in ('signed_virtual_work_J','signed_tangential_work_J')) or any(value[k] != [0.,0.,0.] for k in ('impulse_world_Ns','angular_impulse_about_support_com_Nms')):
                raise ValueError('nonzero initial support audit')
        else:
            self._validate_last(last, value['steps'])
        if value['steps'] == 1:
            pairs={'impulse_world_Ns':np.asarray(last['support_force_world_N'])*self.model.opt.timestep,'angular_impulse_about_support_com_Nms':np.asarray(last['support_torque_about_com_world_Nm'])*self.model.opt.timestep,'signed_virtual_work_J':last['signed_virtual_work_step_J'],'signed_tangential_work_J':last['signed_tangential_work_step_J']}
            if any(not np.allclose(value[k],v,rtol=1e-12,atol=1e-12) for k,v in pairs.items()):raise ValueError('single-step support cumulative ledger mismatch')
        # Commit only after every rejection point.
        self._set_enabled(value['enabled']); self.events = copy.deepcopy(value['events']); self.steps = value['steps']
        self.impulse_world_Ns = np.asarray(value['impulse_world_Ns'], dtype=float)
        self.angular_impulse_about_support_com_Nms = np.asarray(value['angular_impulse_about_support_com_Nms'], dtype=float)
        self.signed_virtual_work_J = float(value['signed_virtual_work_J'])
        self.signed_tangential_work_J = float(value['signed_tangential_work_J']); self.last = copy.deepcopy(last)

    def _guard_audit(self, result):
        a=self.identity['acceptance']
        if result['contact_generalized_force_projection_error_N'] > a['force_projection_abs_N']+a['force_projection_rel']*np.linalg.norm(result['native_contact_row_generalized_force']):raise ValueError('support force projection outside acceptance')
        if result['virtual_power_projection_error_W'] > a['power_projection_abs_W']+a['power_projection_rel']*abs(result['virtual_power_W']):raise ValueError('support virtual-power projection outside acceptance')
        if not 0 <= result['accepted_interval_replay_max_state_error'] <= a['replay_max_state_error']:raise ValueError('support replay outside acceptance')
        if not result['enabled'] and result['contact_count']:raise ValueError('disabled support generated contact')

    def _validate_last(self, last, steps):
        template = self.sample_native(self.data, self.data.qvel)
        keys = set(template) | {'step','time_s','dt_s','accepted_interval_replay_max_state_error','signed_virtual_work_step_J','signed_tangential_work_step_J'}
        if type(last) is not dict or set(last) != keys or last['step'] != steps or last['time_s'] != self.data.time or last['dt_s'] != self.model.opt.timestep or type(last['enabled']) is not bool:
            raise ValueError('invalid last support audit schema/time')
        vectors = {'support_force_world_N':3, 'support_torque_about_com_world_Nm':3,
            **{k:self.model.nv for k in ('contact_generalized_force','support_generalized_force','cable_generalized_force','native_contact_row_generalized_force')}}
        for key, size in vectors.items():
            if type(last[key]) is not list or len(last[key]) != size or not all(finite(x) for x in last[key]): raise ValueError('invalid support audit vector')
        for key in keys - set(vectors) - {'contacts','enabled'}:
            if not finite(last[key]): raise ValueError('invalid support audit scalar')
        if type(last['contact_count']) is not int or last['contact_count'] < 0 or type(last['contacts']) is not list or len(last['contacts']) != last['contact_count']:
            raise ValueError('invalid support contact count')
        def close(a,b): return np.allclose(a,b,rtol=1e-11,atol=1e-11)
        Q=np.asarray(last['contact_generalized_force']); row=np.asarray(last['native_contact_row_generalized_force'])
        if not close(Q,np.asarray(last['support_generalized_force'])+last['cable_generalized_force']) or not close(last['contact_generalized_force_projection_error_N'],np.linalg.norm(Q-row)):
            raise ValueError('support reaction projection mismatch')
        forces=np.zeros(3); torques=np.zeros(3); power=0.; tangent=0.; sp=0.; cp=0.; dist=0.
        contact_keys={'geom1','geom2','support_body','distance_m','position_world_m','contact_frame_rows',
            'force_on_geom2_world_N','force_on_geom1_world_N','torque_on_geom2_world_Nm','force_on_support_world_N',
            'torque_on_support_about_com_world_Nm','relative_point_velocity_world_m_s',
            'relative_angular_velocity_world_rad_s','virtual_power_W','tangential_signed_power_W','power_on_support_W','power_on_cable_W'}
        for c in last['contacts']:
            if type(c) is not dict or set(c)!=contact_keys or c['support_body']!=self.spec['binding_body']: raise ValueError('invalid support contact schema')
            for key,v in c.items():
                if key in ('geom1','geom2','support_body'):
                    if type(v) is not str: raise ValueError('invalid support contact name')
                elif key in ('distance_m','virtual_power_W','tangential_signed_power_W','power_on_support_W','power_on_cable_W'):
                    if not finite(v): raise ValueError('invalid support contact scalar')
                elif type(v) is not list or len(v)!=(9 if key=='contact_frame_rows' else 3) or not all(finite(x) for x in v): raise ValueError('invalid support contact vector')
            g1=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,c['geom1']);g2=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,c['geom2'])
            if not ((g1 in self.support_geoms and g2 in self.cable_geoms) or (g2 in self.support_geoms and g1 in self.cable_geoms)): raise ValueError('invalid support contact pair')
            F=np.asarray(c['force_on_geom2_world_N']);T=np.asarray(c['torque_on_geom2_world_Nm']);dv=np.asarray(c['relative_point_velocity_world_m_s']);dw=np.asarray(c['relative_angular_velocity_world_rad_s']);R=np.asarray(c['contact_frame_rows']).reshape(3,3)
            if not close(c['force_on_geom1_world_N'],-F) or not close(c['force_on_support_world_N'],F if g2 in self.support_geoms else -F) or not close(R@R.T,np.eye(3)) or not close(c['virtual_power_W'],F@dv+T@dw) or not close(c['tangential_signed_power_W'],(R@F)[1:3]@(R@dv)[1:3]) or not close(c['virtual_power_W'],c['power_on_support_W']+c['power_on_cable_W']): raise ValueError('inconsistent support contact force/power')
            forces+=c['force_on_support_world_N'];torques+=c['torque_on_support_about_com_world_Nm'];power+=c['virtual_power_W'];tangent+=c['tangential_signed_power_W'];sp+=c['power_on_support_W'];cp+=c['power_on_cable_W'];dist=min(dist,c['distance_m'])
        checks={'support_force_world_N':forces,'support_torque_about_com_world_Nm':torques,'virtual_power_W':power,'tangential_signed_power_W':tangent,'support_power_W':sp,'cable_power_W':cp,'min_contact_distance_m':dist,'signed_virtual_work_step_J':power*self.model.opt.timestep,'signed_tangential_work_step_J':tangent*self.model.opt.timestep}
        if any(not close(last[k],v) for k,v in checks.items()): raise ValueError('inconsistent aggregate support audit')
        self._guard_audit(last)
