"""Isolated, bounded research controls; construction and GETs never start physics.

A run is a single transaction. A numerical failure restores the entire pre-run
native state and ledgers; an explicit pause commits completed native steps.
"""
from collections import OrderedDict
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = ROOT.parent
SCHEMA = 'gtsd-research-rig-session-v1'
ACK = 'UNCALIBRATED_RESEARCH_NOT_AS_BUILT_NOT_WHOLE_CONTACT_ACCEPTANCE'
MODES = ('mount_only', 'pin_core_contact')


class RigError(ValueError):
    pass


class RigConflict(RigError):
    pass


def canonical(value):
    try:
        return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (ValueError, TypeError, RecursionError) as exc:
        raise RigError('A finite JSON object is required.') from exc


def defaults():
    """Read literal builder defaults without importing NumPy or native code."""
    for node in ast.parse((ROOT / 'build_rig.py').read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DEFAULTS' for t in node.targets):
            return ast.literal_eval(node.value)
    raise RigError('Builder parameter declaration is missing.')


class RigSession:
    max_run_steps = 500
    max_total_steps = 100000
    max_time_s = 5.
    max_checkpoints = 8
    max_requests = 1024

    def __init__(self, plant_factory=None, step_delay_s=0.001):
        self._lock = threading.RLock()
        self._factory = plant_factory
        self._plant = None
        self._worker = None
        self._stop = threading.Event()
        self._step_delay_s = step_delay_s
        self.server_generation_id = uuid.uuid4().hex
        self.session_id = None
        self.revision = 0
        self.phase = 'disabled'
        self._initial = None
        self._checkpoints = OrderedDict()
        self._requests = {}
        self._failure = None
        self._last_trace = None
        self._run = None
        self._events = []

    def _remember(self, rid, fp, result):
        self._requests[rid] = (fp, copy.deepcopy(result))
        return copy.deepcopy(result)

    def _request(self, payload, allowed):
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise RigError('Unknown command fields.')
        rid = payload.get('request_id')
        if not isinstance(rid, str) or not 1 <= len(rid) <= 80 or not all(c.isalnum() or c in '_-.' for c in rid):
            raise RigError('A 1–80 character request_id is required.')
        fp = canonical(payload)
        if rid in self._requests:
            old_fp, result = self._requests[rid]
            if old_fp != fp:
                raise RigConflict('request_id was already used for another payload.')
            return rid, fp, copy.deepcopy(result)
        if len(self._requests) >= self.max_requests:
            raise RigConflict('Request budget exhausted. Restart this local server for a new experiment.')
        if payload.get('server_generation_id') != self.server_generation_id:
            raise RigConflict('Server generation changed. Refresh before issuing another command.')
        if payload.get('session_id') != self.session_id:
            raise RigConflict('Session identity changed. Refresh; do not resend old input.')
        if type(payload.get('expected_revision')) is not int or payload['expected_revision'] != self.revision:
            raise RigConflict('Stale revision. Refresh before issuing another command.')
        return rid, fp, None

    def _event(self, action):
        self._events.append({'action': action, 'revision': self.revision,
                             'steps': self._plant.steps if self._plant else 0})
        self._events = self._events[-24:]

    def metadata(self):
        with self._lock:
            return {'schema': 'gtsd-research-rig-metadata-v1', 'server_generation_id': self.server_generation_id, 'modes': list(MODES),
                    'assumption_ack_required': ACK, 'defaults': defaults(),
                    'pin_geometry_variants': ['nominal_d5', 'research_d4p98'],
                    'limits': {'step_choices': [1, 10, 50], 'run_steps_max': self.max_run_steps,
                               'total_steps_max': self._step_horizon(), 'time_horizon_s_max': self.max_time_s, 'checkpoint_count_max': self.max_checkpoints,
                               'requests_max': self.max_requests, 'force_norm_max_N': 20.},
                    'as_built': False, 'whole_contact_acceptance': False, 'hardware_connected': False,
                    'checkpoint_import_supported': False,
                    'contract': copy.deepcopy(self._plant.contract) if self._plant else None,
                    'session_id': self.session_id,
                    'warnings': ['Historical engineering envelope, not the new photo-layered geometry.',
                                 'All mass, stiffness, damping and contact parameters remain research assumptions.',
                                 'Nominal D5 has zero clearance and unqualified contact transients.',
                                 'D4.98 changes the entire pin display/collision/inertia geometry; it is not a measured hardware pin.',
                                 'Only two named pin/plate/slot interfaces may contact. Other contact paths are not admitted.']}

    def geometry(self):
        """Exact fixed source meshes, plus identified carrier background; no CAD generation."""
        with self._lock:
            profile = json.loads((SOURCE_ROOT / 'mechanical_fit/profiles.json').read_text())
            profile = next(p for p in profile['profiles'] if p['id'] == 'combined')
            rows = [dict(m, branch=g['id']) for g in profile['groups'] for m in g['add_meshes']]
            carriers = json.loads((SOURCE_ROOT / 'mechanical_fit/components/clamps/published_clamp_meshes.json').read_text())['meshes']
            carriers = list(carriers.values()) if isinstance(carriers, dict) else carriers
            rows += [dict(m, branch='clamps') for m in carriers if m['name'] in ('CAD_50DF4_F643_profile_0', 'CAD_51409_F643_profile_0')]
            contract = self._plant.contract if self._plant else None
            overrides = {m['name']: m for m in (contract or {}).get('display_geometry_overrides', [])}
            # An override is keyed by its new mesh identity. The binding may also
            # carry that new name, while its body retains the nominal source name.
            original_to_override = {m.get('solver_body_name', m['name'].removeprefix('RESEARCH_D4p98_')): m for m in overrides.values()}
            result = []
            for mesh in rows:
                source_name = mesh['name']
                replacement = original_to_override.get(source_name)
                m = dict(replacement or mesh, branch=mesh['branch'])
                m['id'] = m['name']
                m['source_mesh_name'] = source_name
                m['geometry_sha256'] = hashlib.sha256(canonical({'vertices': m['vertices'], 'indices': m['indices']}).encode()).hexdigest()
                result.append(m)
            return {'schema': 'gtsd-research-rig-geometry-v1', 'server_generation_id': self.server_generation_id, 'units': 'm', 'meshes': result,
                    'session_id': self.session_id, 'identity_sha256': self._plant.identity if self._plant else None,
                    'as_built': False, 'source_profile': 'historical_combined', 'private_photos_included': False}

    def _step_horizon(self):
        if self._plant is None:
            return self.max_total_steps
        dt = self._plant.contract['parameters']['time_step_s']
        return min(self.max_total_steps, int(self.max_time_s / dt))

    def _parameters(self, mode, parameters):
        if mode not in MODES or not isinstance(parameters, dict):
            raise RigError('Choose an explicit mode and a parameters object.')
        cfg = defaults()
        allowed = set(cfg) - {'mode'}
        if mode == 'pin_core_contact':
            allowed.add('pin_geometry_variant')
        if set(parameters) - allowed:
            raise RigError('Unknown parameters or mode inside parameters.')
        cfg.update(parameters)
        cfg['mode'] = mode
        if mode == 'pin_core_contact':
            cfg.setdefault('pin_geometry_variant', 'nominal_d5')
        for key, value in cfg.items():
            if key in ('mode', 'historical_profile', 'pin_geometry_variant'):
                if not isinstance(value, str):
                    raise RigError('Invalid named parameter ' + key)
                continue
            if key == 'rig_fixed':
                if type(value) is not bool:
                    raise RigError('rig_fixed must be boolean.')
                continue
            values = value if isinstance(value, list) else [value]
            if key in ('gravity_m_s2', 'rig_stiffness', 'rig_inertia_kg_m2') and len(values) != 3:
                raise RigError('Three values are required for ' + key)
            if any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1e9 for x in values):
                raise RigError('Finite bounded numeric parameter required: ' + key)
        canonical(cfg)
        return cfg

    def initialize(self, payload):
        fields = {'mode', 'assumption_ack', 'parameters', 'server_generation_id', 'session_id', 'request_id', 'expected_revision'}
        with self._lock:
            rid, fp, cached = self._request(payload, fields)
            if cached is not None:
                return cached
            if set(payload) != fields or payload['assumption_ack'] != ACK:
                raise RigError('Explicit mode, all control identity fields, parameters and research-assumption acknowledgment are required.')
            if self.phase == 'running':
                raise RigConflict('Pause the current run before starting a new session.')
            cfg = self._parameters(payload['mode'], payload['parameters'])
            factory = self._factory
            if factory is None:
                from plant import Plant
                factory = Plant
            # Build fully before swapping: failed initialization preserves old session.
            candidate = factory(cfg)
            initial = candidate.checkpoint()
            canonical(candidate.snapshot())
            self._plant = candidate
            self._initial = initial
            self.session_id = uuid.uuid4().hex
            self.revision += 1
            self.phase = 'paused'
            self._checkpoints = OrderedDict()
            self._failure = self._last_trace = self._run = None
            self._stop.clear()
            self._event('initialize_new_session')
            return self._remember(rid, fp, self.snapshot())

    def snapshot(self):
        with self._lock:
            result = {'schema': SCHEMA, 'server_generation_id': self.server_generation_id, 'initialized': self._plant is not None, 'session_id': self.session_id,
                      'revision': self.revision, 'phase': self.phase, 'as_built': False,
                      'whole_contact_acceptance': False, 'hardware_connected': False,
                      'background_running': self.phase == 'running', 'mode': None, 'steps': 0, 'time_s': 0.,
                      'body_poses': {}, 'failure': self._failure, 'last_step_trace': copy.deepcopy(self._last_trace),
                      'run': copy.deepcopy({k: v for k, v in self._run.items() if k != 'prior_trace'}) if self._run else None, 'events': copy.deepcopy(self._events),
                      'checkpoints': [{'id': key, 'steps': cp['state']['steps'],
                                       'time_s': cp['time_s'], 'created_revision': cp['created_revision']}
                                      for key, cp in self._checkpoints.items()]}
            if self._plant:
                result.update(copy.deepcopy(self._plant.snapshot()))
                result['mode'] = self._plant.contract['parameters']['mode']
                result['parameters'] = copy.deepcopy(self._plant.contract['parameters'])
                result['remaining_steps'] = self._step_horizon() - self._plant.steps
                result['step_horizon'] = self._step_horizon()
                result['time_horizon_s_max'] = self.max_time_s
            return result

    def _forces(self, loads):
        if not isinstance(loads, dict) or len(loads) > len(self._plant.body_ids):
            raise RigError('A bounded world-force map is required.')
        for name, force in loads.items():
            if name not in self._plant.body_ids or not isinstance(force, list) or len(force) != 3:
                raise RigError('Unknown body or non-three-vector force.')
            if any(type(x) not in (int, float) or not math.isfinite(x) for x in force) or math.hypot(*force) > 20.:
                raise RigError('Each force must be finite and at most 20 N in norm.')
        return copy.deepcopy(loads)

    def _advance(self, loads):
        trace = self._plant.step(loads)
        # Pose and ledgers are the native result. Avoid storing a duplicate state
        # for each interval, but retain the actual used contact force diagnostics.
        self._last_trace = {k: copy.deepcopy(v) for k, v in trace.items() if k != 'state'}

    def _run_steps(self, count, loads, before):
        failed = None
        try:
            for _ in range(count):
                with self._lock:
                    if self._stop.is_set():
                        break
                    self._advance(loads)
                    self._run['completed_steps'] += 1
                if self._stop.wait(self._step_delay_s):
                    break
        except Exception as exc:
            failed = str(exc)
        finally:
            with self._lock:
                if failed:
                    self._plant.restore(before)
                    self._last_trace = self._run.pop('prior_trace')
                    self._failure = 'Run rolled back: ' + failed
                    self.phase = 'error'
                    self._run['outcome'] = 'rolled_back'
                else:
                    self._run.pop('prior_trace', None)
                    self.phase = 'paused'
                    self._run['outcome'] = 'paused' if self._stop.is_set() else 'completed'
                self.revision += 1
                self._event('run_' + self._run['outcome'])

    def command(self, payload):
        base = {'action', 'server_generation_id', 'session_id', 'request_id', 'expected_revision'}
        choices = {'step': {'steps', 'loads'}, 'run': {'steps', 'loads'}, 'pause': set(),
                   'reset': set(), 'checkpoint': set(), 'restore': {'checkpoint_id'}}
        with self._lock:
            if not isinstance(payload, dict) or payload.get('action') not in choices:
                raise RigError('Unknown action; imports and hardware control are unsupported.')
            action = payload['action']
            rid, fp, cached = self._request(payload, base | choices[action])
            if cached is not None:
                return cached
            if not self._plant:
                raise RigConflict('Initialize an explicit research session first.')
            if self.phase == 'running' and action != 'pause':
                raise RigConflict('Only pause is allowed during an active bounded run.')
            if self.phase == 'error' and action not in ('pause', 'reset', 'restore'):
                raise RigConflict('A numerical stop requires reset or an existing checkpoint restore.')
            if action in ('step', 'run'):
                count = payload.get('steps')
                if type(count) is not int or (action == 'step' and count not in (1, 10, 50)) or (action == 'run' and not 1 <= count <= self.max_run_steps):
                    raise RigError('Step supports 1/10/50; run supports 1–500 native steps.')
                if self._plant.steps + count > self._step_horizon():
                    raise RigError('Research step horizon exhausted. Reset before continuing.')
                loads = self._forces(payload.get('loads', {}))
                before = self._plant.checkpoint()
                prior_trace = copy.deepcopy(self._last_trace)
                if action == 'run':
                    self._failure = None
                    self._stop.clear()
                    self.phase = 'running'
                    self._run = {'requested_steps': count, 'completed_steps': 0, 'outcome': 'running', 'prior_trace': prior_trace}
                    self.revision += 1
                    self._event('run_start')
                    result = self.snapshot()
                    self._remember(rid, fp, result)
                    self._worker = threading.Thread(target=self._run_steps, args=(count, loads, before), daemon=True, name='gtsd-bounded-research-run')
                    self._worker.start()
                    return result
                try:
                    for _ in range(count):
                        self._advance(loads)
                    self._failure = None
                    self.phase = 'paused'
                    self._run = None
                except Exception as exc:
                    self._plant.restore(before)
                    self._last_trace = prior_trace
                    self.phase = 'error'
                    self._failure = 'Command rolled back: ' + str(exc)
            elif action == 'pause':
                self._stop.set()
                # Worker checks this flag between complete native steps; this
                # response can be "running" until that boundary is published.
            elif action == 'reset':
                self._plant.restore(self._initial)
                self.phase = 'paused'
                self._failure = self._last_trace = self._run = None
            elif action == 'checkpoint':
                if len(self._checkpoints) >= self.max_checkpoints:
                    raise RigError('Eight checkpoints already saved. Start a new session to clear them.')
                cid = uuid.uuid4().hex
                self._checkpoints[cid] = {'state': self._plant.checkpoint(), 'created_revision': self.revision + 1,
                                          'time_s': self._plant.snapshot()['time_s']}
            elif action == 'restore':
                cid = payload.get('checkpoint_id')
                if not isinstance(cid, str) or cid not in self._checkpoints:
                    raise RigError('Only a saved checkpoint from this session can be restored. File import is unsupported.')
                self._plant.restore(self._checkpoints[cid]['state'])
                self.phase = 'paused'
                self._failure = self._last_trace = self._run = None
            self.revision += 1
            self._event(action)
            result = self.snapshot()
            if action == 'checkpoint':
                result['created_checkpoint_id'] = cid
            return self._remember(rid, fp, result)

    def export_checkpoint(self, cid, session_id):
        with self._lock:
            if session_id != self.session_id or not self._plant:
                raise RigConflict('Export session identity mismatch.')
            if not isinstance(cid, str) or cid not in self._checkpoints:
                raise RigError('Unknown saved checkpoint.')
            return copy.deepcopy({'schema': 'gtsd-research-rig-checkpoint-export-v1', 'session_id': self.session_id,
                                  'checkpoint_id': cid, 'identity_sha256': self._plant.identity,
                                  'as_built': False, 'whole_contact_acceptance': False,
                                  'export_only_no_import_route': True, 'checkpoint': self._checkpoints[cid],
                                  'contract': self._plant.contract})

    def close(self):
        self._stop.set()
        worker = self._worker
        if worker and worker is not threading.current_thread():
            worker.join(timeout=10)
