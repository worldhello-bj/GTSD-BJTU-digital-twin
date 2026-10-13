"""Opt-in, isolated H1 closed-linkage MJCF built from new CAD reference pose.
This is a falsifiable planar topology experiment, never a production migration.
The unresolved head guide, shaft ownership, bellows and all collisions are omitted.
"""
from pathlib import Path
import json,math,argparse,xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
HYPOTHESIS='H1_DUAL_FOURBAR_PLANAR'

def vec(v):return ' '.join(format(float(x),'.17g') for x in v)
def add(p,t,**kw):return ET.SubElement(p,t,{k:str(v) for k,v in kw.items()})
def rotation(q):
 c,s=math.cos(q),math.sin(q)
 return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def contract():return json.loads((ROOT/'binding_review.json').read_text())
def layout(c):
 a={k:np.array(v['world_m']) for k,v in c['anchors'].items()}
 # The complete primary-body pin point is used for B/H/F, not old offsets.
 # C has explicit micron source mismatch; pivot is the plate center.
 origins={'cad_panto_base':np.zeros(3),'cad_panto_lower':a['A'],'cad_panto_upper':a['B'],'cad_panto_lower_tie':a['C_upper'],'cad_panto_head_rocker':a['H'],'cad_panto_upper_tie':a['F']}
 parents={'cad_panto_lower':'cad_panto_base','cad_panto_upper':'cad_panto_lower','cad_panto_lower_tie':'cad_panto_upper','cad_panto_head_rocker':'cad_panto_upper','cad_panto_upper_tie':'cad_panto_head_rocker'}
 order=list(parents)
 return a,origins,parents,order

def forward(q,c=None):
 c=c or contract();a,o,parents,order=layout(c);R={'cad_panto_base':np.eye(3)};P={'cad_panto_base':np.zeros(3)}
 for i,b in enumerate(order):
  p=parents[b];P[b]=P[p]+R[p]@(o[b]-o[p]);R[b]=R[p]@rotation(q[i])
 return {b:(P[b],R[b]) for b in P}
def closure_residual(q,c=None):
 c=c or contract();a,o,p,order=layout(c);pose=forward(q,c)
 def point(b,k):
  P,R=pose[b];return P+R@(a[k]-o[b])
 # Distinct sites preserve the native D mismatch. It is not silently recentered.
 return np.r_[point('cad_panto_lower_tie','D_tie')-point('cad_panto_base','D_base'),point('cad_panto_upper_tie','G')-point('cad_panto_lower','G')]

def build(research_hypothesis=None,out=None,masses=None):
 if research_hypothesis!=HYPOTHESIS:raise ValueError('Explicit --research-hypothesis H1_DUAL_FOURBAR_PLANAR is required; no production or old-body mode exists.')
 c=contract();a,o,parents,order=layout(c)
 # Unit masses are deliberately dimensioned test parameters, not estimates.
 p={b:1.0 for b in order};p.update(masses or {})
 if set(p)!=set(order) or any(not math.isfinite(x) or x<=0 for x in p.values()):raise ValueError('Each test mass must be finite and positive.')
 root=ET.Element('mujoco',model='gtsd_cad_pantograph_H1_RESEARCH_NOT_HARDWARE')
 add(root,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
 opt=add(root,'option',timestep=.0002,gravity='0 0 0',integrator='implicitfast',solver='Newton',iterations=50,tolerance='1e-10');add(opt,'flag',energy='enable')
 defaults=add(root,'default');add(defaults,'geom',contype=0,conaffinity=0,group=1);add(defaults,'joint',limited='false',damping=.02,armature=.00001)
 custom=add(root,'custom');add(custom,'text',name='truth_status',data='RESEARCH_HYPOTHESIS_UNCALIBRATED_NO_PRODUCTION_BINDING');add(custom,'text',name='geometry_source',data=c['source_glb_gzip_sha256'])
 world=add(root,'worldbody');bodies={'cad_panto_base':add(world,'body',name='cad_panto_base')}
 meshes=json.loads((ROOT/'evidence/pantograph_meshes.json').read_text())['meshes'];ms={m['name']:m for m in meshes}
 inertials={}
 for b in order:
  parent=parents[b];el=add(bodies[parent],'body',name=b,pos=vec(o[b]-o[parent]));bodies[b]=el
  add(el,'joint',name=b+'_hinge',type='hinge',axis='0 1 0')
  assigned=[r['mesh_name'] for r in c['meshes'] if r['candidate_included_in_kinematic_model'] and r['candidate_body']==b]
  vs=np.concatenate([np.array(ms[n]['vertices']) for n in assigned]);lo,hi=vs.min(0),vs.max(0);size=np.maximum(hi-lo,.001);com=(lo+hi)/2-o[b]
  # AABB uniform-box equivalent only, never volume/density material inference.
  x,y,z=size;I=p[b]/12*np.array([y*y+z*z,x*x+z*z,x*x+y*y])
  add(el,'inertial',mass=p[b],pos=vec(com),diaginertia=vec(I))
  inertials[b]={'test_mass_kg':p[b],'evidence':'ARBITRARY_POSITIVE_UNIT_TEST_MASS_NOT_PHYSICAL_ESTIMATE','inertia_method':'UNIFORM_AABB_EQUIVALENT_NOT_MESH_DENSITY','com_world_m':((lo+hi)/2).tolist(),'diaginertia_kg_m2':I.tolist(),'source_meshes':assigned}
  add(el,'geom',name=b+'_research_envelope',type='box',pos=vec(com),size=vec(size/2),rgba='.2 .5 .7 .2')
 def site(body,key,name):add(bodies[body],'site',name=name,pos=vec(a[key]-o[body]),size='.003')
 for b,k,n in [('cad_panto_lower_tie','D_tie','D_tie_native'),('cad_panto_base','D_base','D_base_native'),('cad_panto_upper_tie','G','G_tie_native'),('cad_panto_lower','G','G_lower_candidate'),('cad_panto_upper','C_upper','C_plate_native'),('cad_panto_lower_tie','C_tie','C_tie_native'),('cad_panto_head_rocker','E','E_head_axis_unmodeled')]:site(b,k,n)
 eq=add(root,'equality')
 add(eq,'connect',name='lower_C_D_loop',site1='D_tie_native',site2='D_base_native',solref='.002 1',solimp='.95 .995 .001')
 add(eq,'connect',name='upper_F_G_loop',site1='G_tie_native',site2='G_lower_candidate',solref='.002 1',solimp='.95 .995 .001')
 act=add(root,'actuator');add(act,'motor',name='research_root_torque',joint='cad_panto_lower_hinge',gear=1,ctrllimited='true',ctrlrange='-.1 .1')
 ET.indent(root);xml=ET.tostring(root,encoding='unicode')
 binding={'schema':'gtsd-cad-panto-research-binding-v1','hypothesis':HYPOTHESIS,'status':'ISOLATED_RESEARCH_ONLY','production_allowed':False,'as_built':False,'contact_enabled':False,'old_body_pose_reuse':False,'physical_mass_calibration':False,'reference_poses':{b:{'position':v.tolist(),'quaternion':[1.,0.,0.,0.]} for b,v in o.items()},'transform_contract':'v_dynamic_world = R(candidate_body.xquat) @ (v_source_world-reference.position) + candidate_body.xpos','meshes':[{'mesh_name':r['mesh_name'],'body_name':r['candidate_body'],'mode':'candidate_rigid','geometry_sha256':r['geometry_sha256'],'required_hypotheses':r['required_hypotheses']} for r in c['meshes'] if r['candidate_included_in_kinematic_model']],'test_inertials':inertials,'initial_closure_residual_m':closure_residual(np.zeros(5),c).tolist(),'C_pin_axis_source_mismatch_m':c['source_interface_gaps']['C_source_hole_axis_xz_mismatch_m'],'reference_geometry_moved':False,'caveats':['Closure D starts with the recorded native hole mismatch and may settle under constraints.','C hinge uses upper-plate hole; tie-eye source mismatch is retained in mesh and diagnostic sites.','All hinges parallel Y is a candidate; lateral spacers and bushings not solved.','Neither an independent U hinge nor head E/guide compliance is solved.','Research envelope geoms are diagnostics only; source mesh render binding carries actual CAD geometry.'],'mujoco_reference':'https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-connect'}
 if out:
  out=Path(out);out.mkdir(parents=True,exist_ok=True);(out/'candidate.xml').write_text(xml+'\n');(out/'candidate_binding.json').write_text(json.dumps(binding,indent=2)+'\n')
 return xml,binding

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--research-hypothesis',required=True,choices=[HYPOTHESIS]);ap.add_argument('--out',type=Path,default=ROOT/'candidate');args=ap.parse_args();xml,b=build(args.research_hypothesis,args.out);print(json.dumps({'candidate_meshes':len(b['meshes']),'output':str(args.out),'simulation_executed':False}))
