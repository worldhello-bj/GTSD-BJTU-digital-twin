"""Small dependency-free, atomic mesh-profile composition adapter.

The caller supplies the current static source-default display meshes as a
name -> mesh dictionary. This module never edits runtime bodies or collisions.
"""
import math

def validate_profile(profile, source_replacements_sha256):
 if profile.get('schema') != 'gtsd-fit-profile-v1':raise ValueError('Unsupported fit profile')
 req=profile.get('requirements',{})
 if req.get('source_replacements_sha256')!=source_replacements_sha256:raise ValueError('Source replacement revision mismatch')
 if not req.get('atomic') or not req.get('no_solver_changes') or not req.get('physics_collision_mass_unchanged'):raise ValueError('Static-only atomic contract is required')
 if profile.get('default_enabled') or not profile.get('engineering_candidate') or profile.get('physical_fit_certified'):raise ValueError('Candidate must remain explicit and uncertified')
 remove=profile.get('remove_names',[]);add=profile.get('add_meshes',[])
 if not remove or len(remove)!=len(set(remove)):raise ValueError('Invalid removal identities')
 names=[m.get('name')for m in add]
 if not all(names)or len(names)!=len(set(names)):raise ValueError('Duplicate or missing added identities')
 for m in add:
  v=m.get('vertices',[]);f=m.get('indices',[]);md=m.get('metadata',{})
  if len(v)%3 or len(f)%3 or not v or not f or not all(isinstance(x,(int,float))and not isinstance(x,bool)and math.isfinite(x)for x in v):raise ValueError('Invalid vertex/index arrays')
  if not all(isinstance(i,int)and not isinstance(i,bool)and 0<=i<len(v)//3 for i in f):raise ValueError('Invalid triangle index')
  if m.get('physics_binding')is not False or m.get('collision_enabled')is not False or m.get('added_mass_kg')!=0 or md.get('installed_mechanical_connection_confirmed')is not False:raise ValueError('Static candidate flags violated')
 return True

def compose_fit_scene(source_default_meshes, profile, source_replacements_sha256):
 """Return a new complete map, or reject without changing the input map."""
 validate_profile(profile,source_replacements_sha256)
 remove=profile['remove_names'];add=profile['add_meshes']
 missing=sorted(set(remove)-set(source_default_meshes))
 if missing:raise ValueError('Incomplete group or wrong scene: '+', '.join(missing))
 collisions=sorted({m['name']for m in add}&(set(source_default_meshes)-set(remove)))
 if collisions:raise ValueError('Conflicting fit profile already active: '+', '.join(collisions))
 out={n:m for n,m in source_default_meshes.items()if n not in remove}
 out.update({m['name']:m for m in add})
 return out
