"""Independent material-overlap witnesses; no source geometry changed."""
import sys,numpy as np,json,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'photo_turnout'))
from operational_rig.build_rig import source_meshes
from operational_rig.mass_properties import integrate
import cap_geometry as C
rows,source_sha=source_meshes();d={m['name']:m for m in rows};out=[]
for base,top in [('PHOTO_RETAIN_left_ENV_main_rail_1','PHOTO_RETAIN_left_ENV_main_rail_top_1'),('PHOTO_RETAIN_right_ENV_main_rail_1','PHOTO_RETAIN_right_ENV_main_rail_top_1'),('ENV_main_rail_-1','ENV_main_rail_top_-1'),('OP_approach_PHOTO_RETAIN_left_ENV_main_rail_1','OP_approach_PHOTO_RETAIN_left_ENV_main_rail_top_1')]:
 a,b=d[base],d[top];v=np.array(b['vertices']).reshape(-1,3);p=(v.min(0)+v.max(0))/2
 out.append({'material_mesh':base,'overlay_mesh':top,'witness_point_world_m':p.tolist(),'oriented_solid_angle_winding_numbers':[C.witness(a,p),C.witness(b,p)],'material_mesh_mass_at_steel_density_kg':integrate(a,density_kg_m3=7850)['mass_kg'],'overlay_mass_if_erroneously_counted_at_steel_density_kg':integrate(b,density_kg_m3=7850)['mass_kg'],'overlay_current_geometry_role':b.get('geometry_role'),'material_geometry_sha256':a['geometry_sha256'],'overlay_geometry_sha256':b['geometry_sha256']})
t=datetime.datetime.now(datetime.timezone.utc)
r={'schema':'gtsd-independent-source-overlay-audit-v1','completed_utc':t.isoformat(),'completed_beijing':t.astimezone(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),'source_geometry_sha256':source_sha,'method':'Evaluate actual oriented triangle solid-angle sum at a strict shared interior point; winding approximately 1 in both closed source solids proves positive-volume material overlap. This witness does not claim an integrated overlap volume. Report hypothetical standalone overlay mass to identify the mass previously added, not as a measured intersection mass.','witnesses':out}
Path(__file__).with_name('source_alias_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
