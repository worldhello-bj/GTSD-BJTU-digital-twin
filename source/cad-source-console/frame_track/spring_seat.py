"""Opt-in near-side CAD seat research variant; preserve initial axial preload.

No engine import on the XML path. Do not apply to a frozen baseline file or reuse
the old native model/checkpoint identity after changing these force lever arms.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent

def sha(x):return hashlib.sha256(x if isinstance(x,bytes) else x.encode()).hexdigest()
def vec(x):return ' '.join(format(float(v),'.17g') for v in x)
def parse(x):
    v=list(map(float,x.split()))
    if not all(math.isfinite(y) for y in v):raise ValueError('Nonfinite vector')
    return v
def close(x,y):return len(x)==len(y) and all(abs(a-b)<2e-12 for a,b in zip(x,y))

def augment_xml(xml, *, enable_uncalibrated_cad_seat_fit=False):
    if enable_uncalibrated_cad_seat_fit is not True:
        raise ValueError('Explicit enable_uncalibrated_cad_seat_fit=True is required')
    contract=json.loads((ROOT/'spring_seat_contract.json').read_text())
    if contract['schema']!='gtsd-near-side-cad-spring-seat-contract-v1':raise ValueError('Source contract mismatch')
    root=ET.fromstring(xml);changes=[]
    for row in contract['rows']:
        lower=root.find(f'.//body[@name="{row["body_lower"]}"]/site[@name="{row["site_lower"]}"]')
        upper=root.find(f'.//body[@name="{row["body_upper"]}"]/site[@name="{row["site_upper"]}"]')
        tendon=root.find(f'tendon/spatial[@name="{row["tendon_name"]}"]')
        if lower is None or upper is None or tendon is None:raise ValueError('Required native sites/tendon missing')
        if not close(parse(lower.get('pos','')),row['old_lower_local_m']) or not close(parse(upper.get('pos','')),row['old_upper_local_m']):
            raise ValueError('Baseline site mismatch or duplicate CAD fit: '+row['tendon_name'])
        sites=[x.get('site') for x in tendon]
        if sites != [row['site_lower'],row['site_upper']]:raise ValueError('Expected exactly two direct primary tendon sites')
        stiffness=float(tendon.get('stiffness','0'));rest=parse(tendon.get('springlength',''))
        if not math.isfinite(stiffness) or stiffness<=0 or len(rest)!=1 or rest[0]<=0:
            raise ValueError('Explicit positive stiffness and single rest length required')
        delta=row['required_restlength_delta_m'];new_rest=rest[0]+delta
        if new_rest<=0:raise ValueError('Nonpositive candidate rest length')
        old_preload=stiffness*(rest[0]-row['old_initial_length_m'])
        new_preload=stiffness*(new_rest-row['new_initial_length_m'])
        if abs(old_preload-new_preload)>1e-9:raise ValueError('Axial preload was not preserved')
        lower.set('pos',vec(row['new_lower_local_m']));upper.set('pos',vec(row['new_upper_local_m']))
        tendon.set('springlength',format(new_rest,'.17g'))
        changes.append({'tendon_name':row['tendon_name'],'site_lower':row['site_lower'],'site_upper':row['site_upper'],
            'new_lower_local_m':row['new_lower_local_m'],'new_upper_local_m':row['new_upper_local_m'],
            'old_initial_length_m':row['old_initial_length_m'],'new_initial_length_m':row['new_initial_length_m'],
            'old_restlength_m':rest[0],'new_restlength_m':new_rest,'initial_axial_preload_N':new_preload,
            'preload_error_N':new_preload-old_preload})
    result=ET.tostring(root,encoding='unicode')
    return result,{'schema':'gtsd-near-side-cad-spring-seat-application-v1','source_xml_sha256':sha(xml),'new_xml_sha256':sha(result),
        'contract_sha256':sha((ROOT/'spring_seat_contract.json').read_bytes()),'module_sha256':sha(Path(__file__).read_bytes()),
        'uncalibrated_research_opt_in':True,'new_physics_identity_required':True,'old_checkpoint_compatible':False,
        'added_mass_kg':0.,'added_dof':0,'far_side_unchanged':True,'changes':changes,
        'limits':'Preserves initial axial preload only. Changed sites can change suspension moments, Jacobians and motion; not a calibrated joint or spring law.'}

def native_site_snapshot(model,data):
    """Use after the owning native solver has forwarded its current state; never step it here."""
    contract=json.loads((ROOT/'spring_seat_contract.json').read_text());result={}
    for row in contract['rows']:
        for key in ('site_lower','site_upper'):
            name=row[key]
            try:index=model.site(name).id
            except KeyError:raise ValueError('Native spring site missing: '+name)
            position=data.site_xpos[index].tolist()
            if len(position)!=3 or not all(math.isfinite(x) for x in position):raise ValueError('Nonfinite native site')
            result[name]={'position':position,'source':'mujoco.site_xpos'}
    return result
