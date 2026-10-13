"""Versioned mesh lineage and named semantic observations. No physics writes.

Old registry bytes/expected SHA stay intact. Array-address definitions are
explicitly unavailable even if a new model happens to have the same DOF count.
"""
from __future__ import annotations
import copy, hashlib, json, math
from collections import Counter

SCHEMA='gtsd-semantic-lineage-registry-v1'

def canonical(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(v):return hashlib.sha256(canonical(v)).hexdigest()
def sha(b):return hashlib.sha256(b).hexdigest()

def mesh_map(resource):
    rows=resource.get('meshes',[]);by_name={m['name']:m for m in rows};by_id={m['id']:m for m in rows}
    if len(rows)!=len(by_name) or len(rows)!=len(by_id):raise ValueError('Duplicate geometry mesh identity')
    return by_name,by_id

def verify_stage(prior_bytes,current_bytes,proof,removed_names,added_names):
    """Verify all retained records against both actual assets, not a count alone."""
    prior=json.loads(prior_bytes);current=json.loads(current_bytes);old,old_ids=mesh_map(prior);new,new_ids=mesh_map(current)
    before=proof.get('baseline_geometry_sha256',proof.get('prior_geometry_sha256'))
    after=proof.get('composed_geometry_sha256',proof.get('current_geometry_sha256'))
    if before!=sha(prior_bytes) or after!=sha(current_bytes):raise ValueError('Lineage stage geometry SHA mismatch')
    removed=set(removed_names);added=set(added_names)
    if len(removed)!=len(removed_names) or len(added)!=len(added_names):raise ValueError('Duplicate replacement selectors')
    if not removed<=old.keys() or removed & new.keys() or not added<=new.keys() or added & old.keys():raise ValueError('Invalid exact replacement selectors')
    retained=set(old)-removed
    if set(new)!=retained|added:raise ValueError('Unaccounted geometry change')
    rows=proof.get('meshes',[])
    if proof.get('count')!=len(rows) or len(rows)!=len(retained) or {r['mesh_name'] for r in rows}!=retained:raise ValueError('Retained proof denominator mismatch')
    seen=set()
    for row in rows:
        name=row['mesh_name']
        if name in seen:raise ValueError('Duplicate retained proof row')
        seen.add(name)
        if old[name]['id']!=row['mesh_id'] or new[name]['id']!=row['mesh_id'] or digest(old[name])!=row['canonical_mesh_sha256'] or digest(new[name])!=row['canonical_mesh_sha256']:
            raise ValueError('Retained mesh content mismatch: '+name)
    for name in added:
        if not new[name].get('source_refs'):raise ValueError('New mesh lacks explicit source provenance: '+name)
    return {'prior_geometry_sha256':sha(prior_bytes),'current_geometry_sha256':sha(current_bytes),'retained_names':sorted(retained),'removed_names':sorted(removed),'added_names':sorted(added),'proof_rows':len(rows),'every_retained_mesh_checked':True}

def get(value,path):
    for key in path:
        if isinstance(value,list):
            if type(key)is not int or key<0 or key>=len(value):return False,None
            value=value[key]
        elif isinstance(value,dict) and key in value:value=value[key]
        else:return False,None
    return True,value

def valid_value(value,unit,shape=None):
    if shape is not None:return isinstance(value,list) and len(value)==shape and all(type(v) in (int,float) and math.isfinite(v) for v in value)
    if unit=='boolean':return type(value)is bool
    if unit=='enum':return type(value)is str
    if unit is None:return type(value) in (str,bool,int,float) and (type(value)not in(int,float) or math.isfinite(value))
    return type(value) in (int,float) and math.isfinite(value)

class LineageRegistry:
    def __init__(self,legacy,baseline_bytes,current_bytes,binding,model_identity,variant,stages=()):
        self.legacy=copy.deepcopy(legacy);self.identity=model_identity;self.variant=variant
        self.baseline_sha=sha(baseline_bytes);self.current_sha=sha(current_bytes)
        if legacy['source_geometry_sha256']!=self.baseline_sha:raise ValueError('Legacy registry baseline mismatch')
        if binding.get('source_geometry_sha256')!=self.current_sha:raise ValueError('Native binding/current geometry mismatch')
        old,old_ids=mesh_map(json.loads(baseline_bytes));new,new_ids=mesh_map(json.loads(current_bytes))
        self.stages=copy.deepcopy(list(stages))
        if stages:
            if stages[0]['prior_geometry_sha256']!=self.baseline_sha or stages[-1]['current_geometry_sha256']!=self.current_sha:raise ValueError('Incomplete lineage chain')
            for a,b in zip(stages,stages[1:]):
                if a['current_geometry_sha256']!=b['prior_geometry_sha256']:raise ValueError('Broken lineage chain')
        elif self.current_sha!=self.baseline_sha:raise ValueError('No verified lineage for changed geometry')
        retained_names=set(old)
        for stage in stages:retained_names &= set(stage['retained_names'])
        self.retained_ids={old[n]['id'] for n in retained_names}
        binding_by_name={b['mesh_name']:b for b in binding['meshes']}
        if len(binding_by_name)!=len(binding['meshes']) or not set(new)<=binding_by_name.keys():raise ValueError('Binding mesh names missing/duplicate')
        self.meshes=[]
        for name,m in new.items():
            retained=name in retained_names
            if retained and digest(m)!=digest(old[name]):raise ValueError('Baseline retained mesh changed across composed chain')
            b=binding_by_name[name]
            self.meshes.append({'id':m['id'],'name':name,'lineage':'EXACT_BASELINE_MESH_RETAINED' if retained else 'NEW_REVIEWED_DESIGN_MESH',
                'canonical_mesh_sha256':digest(m),'baseline_geometry_sha256':self.baseline_sha if retained else None,
                'current_geometry_sha256':self.current_sha,'body_name':b.get('body_name'),'binding_mode':b.get('mode'),
                'visible_in_dynamic_view':b.get('mode')=='rigid' and b.get('visible_in_dynamic_view',True),
                'registered_component_id':m.get('registered_component_id') or None,'source_refs':copy.deepcopy(m.get('source_refs',[])),
                'geometry_evidence':b.get('evidence',m.get('geometry_status')),'mounting_status':b.get('mounting_status','UNVERIFIED_AS_BUILT'),
                'physical_installation_verified':False,'hardware_calibrated':False})
        signal_keys=('id','label','unit','observation_id','path_tokens','body_names','geometry_mesh_ids','hardware_only','required_coordinate_lengths','requires_accepted_interval','contact_active_path','display_priority','evidence_level')
        self.signals=[{k:copy.deepcopy(row[k]) for k in signal_keys if k in row} for row in legacy['signals']]
        for row in self.signals:
            row['legacy_definition_id']=row['id'];row['definition_origin']='LEGACY_SEMANTIC_DEFINITION'
            old_targets=list(row.get('geometry_mesh_ids',[]))
            row['geometry_mesh_ids']=[i for i in old_targets if i in self.retained_ids]
            row['retired_geometry_mesh_ids']=[i for i in old_targets if i not in self.retained_ids]
            row['lineage_mapping_status']='RETAINED_CONTEXT_ONLY' if row['geometry_mesh_ids'] else 'NO_RETAINED_PHYSICAL_MAPPING'
        item_keys=('id','label','kind','parent_id','geometry_mesh_ids','signal_ids','body_names','evidence_level','assumption','physical_installation_verified','hardware_verified')
        self.items=[{k:copy.deepcopy(row[k]) for k in item_keys if k in row} for row in legacy['items']]
        for item in self.items:
            item['retired_geometry_mesh_ids']=[i for i in item.get('geometry_mesh_ids',[]) if i not in self.retained_ids]
            item['geometry_mesh_ids']=[i for i in item.get('geometry_mesh_ids',[]) if i in self.retained_ids]
        self.augmented=False;self.catalogue_sha=None;self.body_signal_ids={};self.mesh_signal_groups={}

    def augment_named(self,plant):
        if self.augmented:return
        if plant.get('model_identity_sha256')!=self.identity or plant.get('source_geometry_sha256')!=self.current_sha:raise ValueError('Cannot build semantic catalogue for mismatched model')
        existing={s.get('observation_id') for s in self.signals if s.get('observation_id')}
        obs=plant.get('ideal_observations',{})
        if obs.get('source_geometry_sha256')!=self.current_sha:raise ValueError('Native named observation geometry mismatch')
        rows=obs.get('observations',[]);ids=[r.get('signal_id') for r in rows]
        if len(ids)!=len(set(ids)):raise ValueError('Duplicate named observation ID')
        for r in rows:
            if r['signal_id'] in existing:continue
            self.signals.append({'id':'native.'+r['signal_id'],'label':r['signal_id'],'unit':r['unit'],'observation_id':r['signal_id'],
                'body_names':[r['body_name']] if r.get('body_name') else [],'geometry_mesh_ids':[], 'source_refs':[{'source':r.get('source'),'definition':r.get('definition')}],
                'definition_origin':'CURRENT_NAMED_NATIVE_OBSERVER','evidence_level':'IDEAL_SOLVER_NOT_HARDWARE_CALIBRATED','hardware_only':False})
        for body in plant.get('body_poses',{}):
            for field,unit,count in [('position','m',3),('quaternion','1',4)]:
                self.signals.append({'id':f'pose.{body}.{field}','label':f'{body} world {field}','unit':unit,'shape':count,
                    'path_tokens':[['body_poses',body,field]],'body_names':[body],'geometry_mesh_ids':[],
                    'definition_origin':'CURRENT_NAMED_BODY_POSE','evidence_level':'NATIVE_RIGID_BODY_CONTEXT_NOT_COMPONENT_SENSOR','hardware_only':False})
        for key in (plant.get('telemetry',{}).get('cad_doors',{}).get('latches') or {}):
            for field,unit,shape in [('active','boolean',None),('point_gap_m','m',None),('leaf_point_world_m','m',3),('frame_point_world_m','m',3),('native_constraint_force_components','N',3),('hinge_generalized_reaction_Nm','N m',None)]:
                definition={'id':f'cad_latch.{key}.{field}','label':f'{key} latch {field}','unit':unit,'shape':shape,
                    'path_tokens':[['telemetry','cad_doors','latches',key,field]],'body_names':['door_'+key], 'geometry_mesh_ids':[],
                    'definition_origin':'CURRENT_NAMED_ENDPOINT_LATCH','evidence_level':'DETACHED_ENDPOINT_NATIVE_CONSTRAINT_NOT_ACCEPTED_INTERVAL_WORK','hardware_only':False,
                    'source_refs':[{'path':f'telemetry.cad_doors.latches.{key}.{field}','sampling':'DETACHED_ENDPOINT_MJ_FORWARD','force_components_frame':'world','force_components_on':'leaf; frame has opposite sign','not_additional_energy_input':True}]}
                if field=='native_constraint_force_components':definition['label']=key+' 锁扣叶侧世界XYZ约束力（框侧反力=-F）'
                if field=='native_constraint_force_components':definition.update(contact_active_path=['telemetry','cad_doors','latches',key,'active'],inactive_reason='INACTIVE_IDEAL_LATCH_NO_FORCE')
                self.signals.append(definition)
        if 'cad_doors' in plant.get('telemetry',{}):
            for field,unit in [('door_contact_enabled','boolean'),('native_independent_door_fans','1'),('photo_visible_door_fan_mounts','1')]:
                self.signals.append({'id':'cad_doors.'+field,'label':field,'unit':unit,'path_tokens':[['telemetry','cad_doors',field]],'body_names':['payload_secondary'],'geometry_mesh_ids':[],'definition_origin':'CURRENT_EXPLICIT_DOOR_MODEL_STATUS','hardware_only':False})
        for signal in self.signals:
            if signal.get('required_coordinate_lengths') or signal.get('display_priority')=='detail':continue
            for body in signal.get('body_names',[]):self.body_signal_ids.setdefault(body,[]).append(signal['id'])
        for mesh in self.meshes:
            body=mesh['body_name']
            direct=[s['id'] for s in self.signals if mesh['id'] in s.get('geometry_mesh_ids',[]) and s.get('display_priority')!='detail']
            direct_key=digest(direct)[:24];self.mesh_signal_groups[direct_key]=direct
            self.items.append({'id':'mesh.'+mesh['id'],'label':mesh['name'],'kind':'verified_mesh_lineage','geometry_mesh_ids':[mesh['id']],
                'signal_ids':[],'direct_signal_group':direct_key,'body_context_signal_group':body,'body_names':[body] if body else [],'registered_component_id':mesh['registered_component_id'],
                'evidence_level':mesh['lineage'],'assumption':'Readouts associated by native rigid body are shared body context, not a sensor installed on this mesh.','source_refs':mesh['source_refs'],
                'physical_installation_verified':False,'hardware_verified':False})
        self.augmented=True;self.catalogue_sha=digest(self.catalogue(include_hash=False))

    def catalogue(self,include_hash=True):
        result={'schema':SCHEMA,'baseline_registry_source_geometry_sha256':self.legacy['source_geometry_sha256'],
                'current_geometry_sha256':self.current_sha,'model_identity_sha256':self.identity,'variant':self.variant,
                'legacy_expected_hash_modified':False,'lineage_stages':self.stages,'meshes':self.meshes,'signals':[{k:v for k,v in s.items() if k in ('id','label','unit','shape','definition_origin','legacy_definition_id','observation_id','source_refs','evidence_level')} for s in self.signals],'items':self.items,'body_context_signal_groups':self.body_signal_ids,'mesh_direct_signal_groups':self.mesh_signal_groups,
                'denominators':{'legacy_signal_definitions':len(self.legacy['signals']),'additional_named_definitions':len(self.signals)-len(self.legacy['signals']),
                    'total_signal_definitions':len(self.signals),'current_mesh_asset_records':len(self.meshes),'retained_baseline_mesh_records':len(self.retained_ids)},
                'calibrated':False,'hardware_connected':False,'counts_are_hardware_coverage':False}
        if include_hash:result['catalogue_sha256']=self.catalogue_sha
        return copy.deepcopy(result)

    def observe(self,plant):
        self.augment_named(plant)
        identity=plant.get('model_identity_sha256')==self.identity and plant.get('source_geometry_sha256')==self.current_sha and plant.get('variant')==self.variant
        named=plant.get('ideal_observations',{});native_rows=named.get('observations',[])
        native_map={r.get('signal_id'):r for r in native_rows}
        native_valid=len(native_map)==len(native_rows) and named.get('source_geometry_sha256')==self.current_sha and named.get('sample_time_s')==plant.get('time_s')
        result=[]
        for definition in self.signals:
            row={'id':definition['id'],'unit':definition['unit'],'value':None,'available':False,'availability':'MISSING_IN_CURRENT_SNAPSHOT','sample_time_s':plant.get('time_s'),'source_path_used':None}
            if not identity:row['availability']='MODEL_OR_GEOMETRY_IDENTITY_MISMATCH'
            elif definition.get('hardware_only'):row['availability']='UNAVAILABLE_HARDWARE_UNBOUND'
            elif definition.get('required_coordinate_lengths'):row['availability']='UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS'
            elif definition.get('requires_accepted_interval') and not plant.get('steps',0):row['availability']='NO_ACCEPTED_INTERVAL'
            elif definition.get('contact_active_path') and get(plant,definition['contact_active_path'])[1] is not True:row['availability']=definition.get('inactive_reason','INACTIVE_CONTACT_NO_MEASUREMENT')
            else:
                found,value=False,None
                if definition.get('observation_id'):
                    obs=native_map.get(definition['observation_id']) if native_valid else None
                    if obs is not None:
                        if obs.get('unit')!=definition['unit']:row['availability']='NATIVE_UNIT_MISMATCH'
                        elif obs.get('sample_time_s')!=plant.get('time_s'):row['availability']='STALE_NATIVE_SAMPLE'
                        else:found,value=True,obs.get('value');row['source_path_used']=['ideal_observations','signal_id='+definition['observation_id']]
                    elif not native_valid:row['availability']='NAMED_OBSERVATION_IDENTITY_OR_TIME_MISMATCH'
                else:
                    for path in definition.get('path_tokens',[]):
                        # No root generalized state arrays accepted by this semantic adapter.
                        if path and path[0] in ('qpos','qvel','qacc','qfrc_applied'):row['availability']='UNMIGRATED_GENERALIZED_ARRAY_SEMANTICS';continue
                        found,value=get(plant,path)
                        if found:row['source_path_used']=path;break
                if found:
                    if value is None:row['availability']='NULL_IN_CURRENT_SNAPSHOT'
                    elif not valid_value(value,definition['unit'],definition.get('shape')):row['availability']='INVALID_TYPE_OR_NONFINITE'
                    else:row.update(value=copy.deepcopy(value),available=True,availability='SOLVER_EQUIVALENT_UNCALIBRATED')
            result.append(row)
        legacy_n=len(self.legacy['signals']);counts=Counter(r['availability'] for r in result[:legacy_n]);counts_add=Counter(r['availability'] for r in result[legacy_n:])
        return {'schema':'gtsd-semantic-lineage-observations-v1','catalogue_sha256':self.catalogue_sha,'model_identity_sha256':plant.get('model_identity_sha256'),
                'current_geometry_sha256':plant.get('source_geometry_sha256'),'identity_match':identity,'sample_time_s':plant.get('time_s'),'steps':plant.get('steps'),
                'calibrated':False,'hardware_connected':False,'legacy_expected_hash_modified':False,'observations':result,
                'denominators':{'legacy':{'total':legacy_n,'available':sum(r['available'] for r in result[:legacy_n]),'reasons':dict(counts)},
                    'added_named':{'total':len(result)-legacy_n,'available':sum(r['available'] for r in result[legacy_n:]),'reasons':dict(counts_add)}},
                'counts_are_hardware_coverage':False}
