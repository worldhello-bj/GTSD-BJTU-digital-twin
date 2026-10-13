"""Bounded native MuJoCo session for the isolated CAD H1 research candidate.
Construction and read-only status never import/start MuJoCo. No background run.
"""
from pathlib import Path
import copy,hashlib,importlib.util,json,math,threading,time,uuid
from collections import OrderedDict
ROOT=Path(__file__).resolve().parent
MODE='H1_DUAL_FOURBAR_PLANAR'
SCHEMA='gtsd-pantograph-candidate-session-v1'

class CandidateError(ValueError):pass
class CandidateConflict(CandidateError):pass

class CandidateSession:
    max_step_command=50
    max_steps_until_reset=500
    max_checkpoints=8
    def __init__(self):
        self._lock=threading.RLock();self._model=None;self._data=None;self._mj=None
        self.session_id=None;self.revision=0;self.steps=0;self.lifetime_steps=0
        self.phase='disabled';self._checkpoints=OrderedDict();self._requests=OrderedDict()
        self._events=[];self._last_step_trace=[];self._failure=None
        self.review=json.loads((ROOT/'binding_review.json').read_text())
        self.binding=json.loads((ROOT/'candidate/candidate_binding.json').read_text())
    @property
    def initialized(self):return self._model is not None
    def _event(self,action):
        self._events.append({'revision':self.revision,'action':action,'time_s':float(self._data.time) if self.initialized else 0.});self._events=self._events[-32:]
    def metadata(self):
        return {'schema':'gtsd-pantograph-candidate-metadata-v1','mode_required':MODE,'production_allowed':False,'as_built':False,'candidate_model_identity_sha256':hashlib.sha256((ROOT/'candidate/candidate.xml').read_text().strip().encode()).hexdigest(),'review':self.review,'binding':self.binding,'limits':{'step_command_max':50,'steps_until_reset':500,'checkpoint_count_max':8,'background_run_supported':False,'checkpoint_upload_supported':False},'source_status':'New CAD mesh coordinates verified against exact source DXF; attachment hypotheses remain unverified.'}
    def initialize(self,mode):
        with self._lock:
            if mode!=MODE:raise CandidateError('Explicit H1 research mode is required.')
            if self.initialized:return self.snapshot()
            # All native imports are confined to this explicit opt-in path.
            import mujoco
            import numpy as np
            spec=importlib.util.spec_from_file_location('gtsd_cad_panto_candidate_builder',ROOT/'scripts/build_candidate.py');builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
            xml,binding=builder.build(MODE)
            if xml.strip()!=(ROOT/'candidate/candidate.xml').read_text().strip():raise CandidateError('Candidate XML no longer matches reviewed artifact.')
            model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data)
            self._mj=mujoco;self._np=np;self._model=model;self._data=data;self.binding=binding
            self.model_sha256=hashlib.sha256(xml.encode()).hexdigest();self.session_id=uuid.uuid4().hex;self.phase='paused';self.revision=1;self._event('initialize_explicit_H1')
            self._state_spec=mujoco.mjtState.mjSTATE_INTEGRATION;self._state_size=mujoco.mj_stateSize(model,self._state_spec)
            self._initial=self._capture();self._last_good=self._capture()
            return self.snapshot()
    def _capture(self):
        state=self._np.empty(self._state_size);self._mj.mj_getState(self._model,self._data,state,self._state_spec)
        return {'state':state.copy(),'steps':self.steps,'model_sha256':self.model_sha256,'session_id':self.session_id,'state_spec':int(self._state_spec)}
    def _restore(self,cp):
        if cp['model_sha256']!=self.model_sha256 or cp['session_id']!=self.session_id:raise CandidateConflict('Checkpoint identity does not match this session.')
        self._mj.mj_resetData(self._model,self._data);self._mj.mj_setState(self._model,self._data,cp['state'],self._state_spec);self._mj.mj_forward(self._model,self._data);self.steps=cp['steps']
    def _warnings(self):return {self._mj.mjtWarning(i).name:int(w.number) for i,w in enumerate(self._data.warning) if w.number}
    def _diagnostic(self):
        d=self._data
        return {'time_s':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'energy_J':d.energy.tolist(),'constraint_residual_m':d.efc_pos.tolist(),'max_constraint_residual_m':float(self._np.max(self._np.abs(d.efc_pos))) if len(d.efc_pos) else 0.,'contacts':int(d.ncon),'warnings':self._warnings(),'torque_input_Nm':float(d.ctrl[0])}
    def snapshot(self):
        with self._lock:
            v={'schema':SCHEMA,'mode':MODE if self.initialized else None,'initialized':self.initialized,'phase':self.phase,'session_id':self.session_id,'revision':self.revision,'steps':self.steps,'lifetime_steps':self.lifetime_steps,'remaining_steps_until_reset':self.max_steps_until_reset-self.steps,'production_allowed':False,'as_built':False,'hardware_connected':False,'new_contact_enabled':False,'background_running':False,'model_identity_sha256':getattr(self,'model_sha256',None),'body_poses':{},'time_s':0.,'checkpoints':[{'id':k,'steps':x['steps'],'time_s':float(x['state'][0]),'created_revision':x['created_revision']} for k,x in self._checkpoints.items()],'failure':self._failure,'events':copy.deepcopy(self._events),'last_step_trace':copy.deepcopy(self._last_step_trace)}
            if self.initialized:
                v.update(self._diagnostic());v['body_poses']={n:{'position':self._data.xpos[self._model.body(n).id].tolist(),'quaternion':self._data.xquat[self._model.body(n).id].tolist()} for n in self.binding['reference_poses']};v['mujoco_version']=self._mj.__version__
            return v
    def command(self,payload):
        with self._lock:
            if not self.initialized:raise CandidateConflict('Initialize explicit H1 candidate mode first.')
            if not isinstance(payload,dict) or set(payload)-{'action','session_id','expected_revision','request_id','steps','torque_Nm','checkpoint_id'}:raise CandidateError('Unknown candidate command fields.')
            action=payload.get('action');allowed={'step':{'steps','torque_Nm'},'pause':set(),'reset':set(),'checkpoint':set(),'restore':{'checkpoint_id'}}
            if action not in allowed:raise CandidateError('Only bounded step, manual pause, reset, checkpoint and in-memory restore are supported.')
            if (set(payload)-{'action','session_id','expected_revision','request_id'})-allowed[action]:raise CandidateError('Fields do not match this action.')
            if payload.get('session_id')!=self.session_id:raise CandidateConflict('Candidate session identity mismatch.')
            rid=payload.get('request_id')
            if not isinstance(rid,str) or not 1<=len(rid)<=80:raise CandidateError('A bounded request_id is required.')
            fingerprint=json.dumps(payload,sort_keys=True,allow_nan=False)
            if rid in self._requests:
                old_fp,result=self._requests[rid]
                if old_fp!=fingerprint:raise CandidateConflict('request_id already used with a different command.')
                return copy.deepcopy(result)
            if type(payload.get('expected_revision')) is not int or payload['expected_revision']!=self.revision:raise CandidateConflict('Stale candidate revision. Refresh before retrying.')
            if self.phase=='error' and action not in ['reset','restore','pause']:raise CandidateConflict('Candidate is paused after failure; reset or restore an existing checkpoint.')
            if action=='step':
                steps=payload.get('steps');torque=payload.get('torque_Nm',0.)
                if type(steps) is not int or not 1<=steps<=50:raise CandidateError('Only 1–50 native steps per command.')
                if type(torque) not in [int,float] or not math.isfinite(torque) or abs(torque)>.1:raise CandidateError('Research torque must be finite and within ±0.1 Nm.')
                if self.steps+steps>self.max_steps_until_reset:raise CandidateError('500-step budget reached; checkpoint and reset before more experiments.')
                saved=self._capture();self._last_step_trace=[];self.phase='stepping'
                try:
                    for _ in range(steps):
                        self._data.ctrl[0]=torque;self._mj.mj_step(self._model,self._data);self.lifetime_steps+=1;self.steps+=1;self._mj.mj_forward(self._model,self._data)
                        d=self._diagnostic();self._last_step_trace.append(d)
                        if not self._np.isfinite(self._data.qpos).all() or not self._np.isfinite(self._data.qvel).all() or not self._np.isfinite(self._data.energy).all() or d['warnings'] or d['contacts'] or d['max_constraint_residual_m']>.001:raise RuntimeError('Numerical safety check failed; restored pre-command state.')
                    self.phase='paused';self._last_good=self._capture();self._failure=None
                except Exception as e:
                    self._restore(saved);self.phase='error';self._failure=str(e)
            elif action=='pause':self.phase='paused' if self.phase!='error' else 'error'
            elif action=='reset':
                self._restore(self._initial);self.phase='paused';self._failure=None;self._last_step_trace=[]
            elif action=='checkpoint':
                if len(self._checkpoints)>=8:raise CandidateError('Eight in-memory checkpoints already exist; export them before opening a new session.')
                cid=uuid.uuid4().hex;cp=self._capture();cp['created_revision']=self.revision;self._checkpoints[cid]=cp
            elif action=='restore':
                cid=payload.get('checkpoint_id')
                if not isinstance(cid,str) or cid not in self._checkpoints:raise CandidateError('Only an existing in-memory checkpoint ID may be restored. File upload is unsupported.')
                self._restore(self._checkpoints[cid]);self.phase='paused';self._failure=None;self._last_step_trace=[]
            self.revision+=1;self._event(action);result=self.snapshot()
            if action=='checkpoint':result['created_checkpoint_id']=cid
            self._requests[rid]=(fingerprint,copy.deepcopy(result))
            while len(self._requests)>128:self._requests.popitem(last=False)
            return result
    def export_checkpoint(self,cid):
        with self._lock:
            if cid not in self._checkpoints:raise CandidateError('Unknown in-memory checkpoint ID.')
            cp=self._checkpoints[cid]
            return {'schema':'gtsd-pantograph-candidate-checkpoint-export-v1','export_only_no_import_route':True,'mode':MODE,'as_built':False,'checkpoint_id':cid,'session_id':self.session_id,'model_identity_sha256':cp['model_sha256'],'source_dxf_sha256':self.review.get('native_source_check',{}).get('source_dxf_sha256'),'state_spec':cp['state_spec'],'state':cp['state'].tolist(),'steps':cp['steps'],'created_revision':cp['created_revision'],'warning':'Research numerical state. Not hardware truth, not transferable to old solver, and not accepted as a file upload.'}
