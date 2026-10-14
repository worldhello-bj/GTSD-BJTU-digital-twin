"""Operational assembly controls using the established atomic session engine."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = ROOT.parent
_spec = importlib.util.spec_from_file_location('_gtsd_research_session', SOURCE_ROOT / 'research_rig/session.py')
_legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_legacy)
RigError, RigConflict, canonical = _legacy.RigError, _legacy.RigConflict, _legacy.canonical
SCHEMA = 'gtsd-operational-rig-session-v1'
MODE = 'operational_assembly'


def parameter_document():
    return json.loads((ROOT / 'parameters.json').read_text())


def defaults():
    doc = parameter_document()
    return copy.deepcopy(doc.get('defaults', doc))


def parameter_sources():
    path = ROOT / 'parameter_sources.json'
    return json.loads(path.read_text()) if path.exists() else []


class RigSession(_legacy.RigSession):
    max_run_steps = 40000
    max_total_steps = 100000
    max_time_s = 5.

    def __init__(self, plant_factory=None, step_delay_s=0.):
        super().__init__(plant_factory, step_delay_s)
        self._demo_active = False
        self._run_timing = None
        self._initialization_wall_s = None

    def metadata(self):
        with self._lock:
            return {'schema': 'gtsd-operational-rig-metadata-v1',
                    'server_generation_id': self.server_generation_id,
                    'session_id': self.session_id, 'mode': MODE, 'modes': [MODE],
                    'defaults': defaults(), 'parameter_document': parameter_document(),
                    'parameter_sources': parameter_sources(),
                    'limits': {'step_choices': [1, 10, 50], 'run_steps_max': self.max_run_steps,
                               'demo_steps_max': self.max_total_steps,
                               'total_steps_max': self._step_horizon(), 'time_horizon_s_max': self.max_time_s,
                               'checkpoint_count_max': self.max_checkpoints,
                               'requests_max': self.max_requests, 'force_norm_max_N': 20.},
                    'contract': copy.deepcopy(self._plant.contract) if self._plant else None,
                    'as_built': False, 'hardware_connected': False,
                    'checkpoint_import_supported': False}

    def geometry(self):
        with self._lock:
            from build_rig import load_geometry
            parameters = self._plant.contract['parameters'] if self._plant else defaults()
            meshes = load_geometry(parameters, contract=self._plant.contract if self._plant else None)
            if isinstance(meshes, dict):
                meshes = meshes['meshes']
            return {'schema': 'gtsd-operational-rig-geometry-v1',
                    'server_generation_id': self.server_generation_id,
                    'session_id': self.session_id,
                    'identity_sha256': self._plant.identity if self._plant else None,
                    'units': 'm', 'meshes': copy.deepcopy(meshes), 'source_profile': MODE}

    def _parameters(self, mode, parameters):
        if mode != MODE or not isinstance(parameters, dict):
            raise RigError('Choose the operational assembly and a parameters object.')
        cfg = defaults()
        if set(parameters) - set(cfg) or 'mode' in parameters:
            raise RigError('Unknown parameter. Use the keys listed on the parameters page.')
        cfg.update(parameters)
        cfg['mode'] = MODE
        canonical(cfg)
        # Detailed dimensional bounds belong to the model builder. Reject bools,
        # non-finite and oversized values before any native allocation.
        for key, value in cfg.items():
            original = defaults().get(key)
            if isinstance(original, str):
                if not isinstance(value, str): raise RigError('Text required for ' + key)
                continue
            if isinstance(original, bool):
                if type(value) is not bool: raise RigError('Boolean required for ' + key)
                continue
            values = value if isinstance(value, list) else [value]
            if isinstance(original, list) and len(values) != len(original):
                raise RigError('Vector length differs for ' + key)
            if any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > 1e10 for x in values):
                raise RigError('Finite numeric values required for ' + key)
        return cfg

    def initialize(self, payload):
        fields = {'mode', 'parameters', 'server_generation_id', 'session_id', 'request_id', 'expected_revision'}
        with self._lock:
            rid, fp, cached = self._request(payload, fields)
            if cached is not None: return cached
            if set(payload) != fields:
                raise RigError('Initialization requires mode, parameters and current session identity.')
            if self.phase == 'running':
                raise RigConflict('Pause before initializing another assembly.')
            cfg = self._parameters(payload['mode'], payload['parameters'])
            factory = self._factory
            if factory is None:
                from plant import Plant
                factory = Plant
            started = time.perf_counter()
            candidate = factory(cfg)
            dt = candidate.contract['parameters']['time_step_s']
            duration = candidate.contract.get('demo', {}).get('duration_s', 1.)
            demo_steps = math.ceil(duration / dt - 1e-10)
            if demo_steps > min(self.max_total_steps, int(self.max_time_s / dt)):
                raise RigError('The configured complete motion exceeds the session horizon. Shorten its duration or increase the step size.')
            initial = candidate.checkpoint()
            canonical(candidate.snapshot())
            initialization_wall_s = time.perf_counter() - started
            self._plant, self._initial = candidate, initial
            self.session_id = uuid.uuid4().hex
            self.revision += 1
            self.phase = 'paused'
            self._checkpoints.clear()
            self._failure = self._last_trace = self._run = None
            self._demo_active = False
            self._run_timing = None
            self._initialization_wall_s = initialization_wall_s
            self._stop.clear()
            self._event('initialize_new_session')
            return self._remember(rid, fp, self.snapshot())

    def snapshot(self):
        with self._lock:
            state = super().snapshot()
            state['schema'] = SCHEMA
            state['demo_active'] = self._demo_active
            state['initialization_wall_s'] = self._initialization_wall_s
            if self._run_timing:
                timing = self._run_timing
                elapsed = timing.get('elapsed_wall_s', time.perf_counter() - timing['started'])
                advanced = max(0, self._plant.steps - timing['start_steps'])
                state['performance'] = {'elapsed_wall_s': elapsed, 'advanced_steps': advanced,
                    'simulated_s_per_wall_s': advanced * self._plant.contract['parameters']['time_step_s'] / elapsed if elapsed else 0.}
            state.pop('whole_contact_acceptance', None)
            if self._plant:
                demo = self._plant.contract.get('demo', {})
                duration = demo.get('duration_s', 1.)
                state['demo'] = dict(demo, duration_s=duration,
                                     progress=min(1., state['time_s'] / duration))
            return state

    def _advance(self, loads):
        if self._demo_active:
            loads = self._plant.demo_loads(self._plant.steps * self._plant.contract['parameters']['time_step_s'])
        super()._advance(loads)

    def _run_steps(self, count, loads, before):
        with self._lock:
            self._run_timing = {'started': time.perf_counter(), 'start_steps': self._plant.steps}
        failed = None
        try:
            for _ in range(count):
                with self._lock:
                    if self._stop.is_set(): break
                    self._advance(loads)
                    self._run['completed_steps'] += 1
                if self._stop.wait(self._step_delay_s): break
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
                # Publish terminal phase, program state and timing atomically.
                # A fresh command cannot enter between "paused" and this reset.
                self._demo_active = False
                self._run_timing['elapsed_wall_s'] = time.perf_counter() - self._run_timing['started']
                self.revision += 1
                self._event('run_' + self._run['outcome'])

    def command(self, payload):
        if not isinstance(payload, dict) or payload.get('action') != 'demo':
            try:
                return super().command(payload)
            except RigError as exc:
                if str(exc) == 'Step supports 1/10/50; run supports 1–500 native steps.':
                    raise RigError(f'Step supports 1/10/50; run supports 1–{self.max_run_steps} native steps.') from exc
                raise
        fields = {'action', 'server_generation_id', 'session_id', 'request_id', 'expected_revision'}
        with self._lock:
            rid, fp, cached = self._request(payload, fields)
            if cached is not None: return cached
            if self._plant is None:
                raise RigConflict('Initialize the assembly first.')
            if self.phase == 'running':
                raise RigConflict('A run is already active.')
            if self.phase == 'error':
                raise RigConflict('Reset or restore a checkpoint before continuing.')
            dt = self._plant.contract['parameters']['time_step_s']
            duration = self._plant.contract.get('demo', {}).get('duration_s', 1.)
            end = math.ceil(duration / dt - 1e-10)
            count = end - self._plant.steps
            if count <= 0:
                raise RigConflict('The complete motion has finished. Reset or restore to replay it.')
            if self._plant.steps + count > self._step_horizon():
                raise RigError('This demonstration exceeds the current step budget.')
            before = self._plant.checkpoint()
            self._failure = None
            self._stop.clear()
            self.phase = 'running'
            self._demo_active = True
            self._run = {'requested_steps': count, 'completed_steps': 0, 'outcome': 'running',
                         'program': 'complete_motion', 'prior_trace': copy.deepcopy(self._last_trace)}
            self.revision += 1
            self._event('demo_start')
            result = self.snapshot()
            self._remember(rid, fp, result)
            self._worker = threading.Thread(target=self._run_steps, args=(count, {}, before),
                                            daemon=True, name='gtsd-operational-demo')
            self._worker.start()
            return result

    def export_checkpoint(self, cid, session_id):
        result = super().export_checkpoint(cid, session_id)
        result['schema'] = 'gtsd-operational-rig-checkpoint-export-v1'
        result.pop('whole_contact_acceptance', None)
        return result
