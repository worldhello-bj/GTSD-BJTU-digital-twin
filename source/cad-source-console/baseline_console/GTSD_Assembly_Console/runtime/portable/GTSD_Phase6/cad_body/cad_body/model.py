"""Replace body mass geometry and native floor in the existing 79-DOF tree.

Only the residual 26.6 kg body budget is redistributed. It includes fixed
unitemized equipment: mass/volume is a mathematical weighting, NOT material
aluminium density. Existing doors and auxiliaries have already been debited.
"""
import copy,hashlib,json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .geometry import read_sources,resource
from aux_mount.model import parallel_axis,quatmat,_inertial_attributes,_physical_tensor,spatial_mass_properties,vec

def augment_xml(xml,support_spec):
    cad,moments,evidence,contract=read_sources();base=mujoco.MjModel.from_xml_string(xml)
    d=mujoco.MjData(base);mujoco.mj_forward(base,d);root=ET.fromstring(xml)
    if root.find('.//geom[@name="cad_body_visual_0"]') is not None:raise ValueError('CAD body is already augmented')
    bid=base.body('payload_secondary').id;mass=float(base.body_mass[bid])
    if abs(mass-26.6)>1e-12:raise ValueError('Expected once-allocated residual payload budget of 26.6 kg')
    body=root.find('.//body[@name="payload_secondary"]');translation=np.array([0.,0.,contract['body_bottom_z_m']]);R=d.xmat[bid].reshape(3,3)
    if not np.allclose(R,np.eye(3),rtol=0,atol=1e-14):raise ValueError('CAD installation contract currently requires reference-aligned body axes')
    rows=[];first=np.zeros(3);origin=np.zeros((3,3));total=0.
    for part in moments['parts']:
        m=mass*part['volume_m3']/moments['volume_m3'];cw=np.asarray(part['centroid_component_m'])+translation
        cl=R.T@(cw-d.xpos[bid]);ic=R.T@np.asarray(part['unit_density_inertia_com_kgm2'])@R*mass/moments['volume_m3']
        _physical_tensor(ic,part['name']);first+=m*cl;origin+=ic+parallel_axis(m,cl);total+=m
        rows.append(dict(name=part['name'],mass_kg=m,volume_m3=part['volume_m3'],com_body_local_m=cl.tolist(),inertia_com_body_local_kgm2=ic.tolist()))
    center=first/total;inertia=origin-parallel_axis(total,center);inertia=(inertia+inertia.T)/2;eig=_physical_tensor(inertia,'CAD residual body')
    expected_center=R.T@(np.asarray(moments['centroid_component_m'])+translation-d.xpos[bid])
    expected_inertia=R.T@np.asarray(moments['inertia_per_kg_m2'])@R*mass
    if abs(total-mass)>1e-12 or max(abs(center-expected_center))>1e-12 or np.max(abs(inertia-expected_inertia))>1e-12:raise ValueError('Per-part reconstruction differs from CAD aggregate')
    oldR=quatmat(base.body_iquat[bid]);old=dict(mass_kg=mass,com_local_m=base.body_ipos[bid].tolist(),inertia_com_local_kgm2=(oldR@np.diag(base.body_inertia[bid])@oldR.T).tolist())
    inertial=body.find('inertial');inertial.attrib.clear();inertial.attrib.update(_inertial_attributes(mass,center,inertia))
    removed=[]
    for parent in root.iter():
        for e in list(parent):
            if e.tag=='geom' and e.get('name') in ('payload_proxy','roof_reference'):
                removed.append(e.get('name'));parent.remove(e)
    # Exact floor is rectangular, so a box is its true shape, not an outer hull.
    floor=next(m for m in cad['meshes'] if m['name']=='CAD_body_floor_39780');v=np.asarray(floor['vertices']).reshape(-1,3)+translation
    lo=v.min(axis=0);hi=v.max(axis=0);fc=R.T@((lo+hi)/2-d.xpos[bid]);half=(hi-lo)/2
    fg=root.find('.//geom[@name="v5s_BODY_bay_floor"]')
    if fg is None:raise ValueError('Expected existing active floor support')
    fg.set('pos',vec(fc));fg.set('size',vec(half));fg.set('rgba','.21 .40 .60 .15')
    support=copy.deepcopy(support_spec);floor_record=next(x for x in support['supports'] if x['source_mesh']=='BODY_bay_floor')
    floor_record.update(source_mesh=floor['name'],aabb_min_world_m=lo.tolist(),aabb_max_world_m=hi.tolist(),center_local_m=fc.tolist(),half_size_m=half.tolist(),
       collision='EXACT_CAD_RECTANGULAR_FLOOR_SWEEP',mounting_evidence='CAD_SECTION_FLOOR_PLUS_INHERITED_BODY_BOTTOM_DATUM')
    support['cad_floor_resource_sha256']=hashlib.sha256(json.dumps(floor,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    support['source_geometry_role']='Baseline hash applies only to two unchanged legacy troughs; CAD floor has its separate resource hash.'
    support['mass_boundary']='CAD floor share is included once in residual 26.6 kg equivalent; all support geoms add zero mass.'
    # Embed exact triangle visuals in native model as well as in composed web resource.
    # Shells/tubes/ribs have contact disabled; no convex hull is presented as a hollow contact model.
    asset=root.find('asset')
    if asset is None:asset=ET.SubElement(root,'asset')
    native_names=[]
    for i,mesh in enumerate(cad['meshes']):
        name='cad_body_visual_'+str(i);local=(np.asarray(mesh['vertices']).reshape(-1,3)+translation-d.xpos[bid])@R
        ET.SubElement(asset,'mesh',name=name,vertex=vec(local.reshape(-1)),face=' '.join(map(str,mesh['indices'])))
        ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,mass='0',contype='0',conaffinity='0',rgba=vec(mesh['color']))
        native_names.append(name)
    ET.indent(root);result=ET.tostring(root,encoding='unicode');model=mujoco.MjModel.from_xml_string(result);md=mujoco.MjData(model);mujoco.mj_forward(model,md)
    before=spatial_mass_properties(base,d);after=spatial_mass_properties(model,md)
    if abs(before['mass_kg']-after['mass_kg'])>1e-11 or model.nv!=base.nv or model.nq!=base.nq:raise ValueError('CAD body changed total mass or DOFs')
    compiledR=quatmat(model.body_iquat[bid]);compiled=compiledR@np.diag(model.body_inertia[bid])@compiledR.T
    if np.max(abs(compiled-inertia))>1e-11:raise ValueError('Compiled tensor differs from full CAD tensor')
    spec=dict(schema='gtsd-cad-body-inertia-floor-model-v1',mass_boundary='26.6 kg residual equivalent including fixed unitemized equipment distributed by CAD solid volume; not a material-density claim. Four doors (2.4 kg including 0.06 kg bay fan) and other auxiliaries (1.0 kg) remain debited exactly once from original 30 kg budget.',
      cad_original_unit_density_moments=moments,component_to_world_translation_m=translation.tolist(),reference_body_position_m=d.xpos[bid].tolist(),
      original_residual=old,residual_mass_kg=mass,residual_com_body_local_m=center.tolist(),residual_inertia_com_body_local_kgm2=inertia.tolist(),principal_inertias_kgm2=eig.tolist(),
      equivalent_weighting_kg_per_m3=mass/moments['volume_m3'],weighting_is_material_density=False,parts=rows,
      reference_spatial_before=before,reference_spatial_after=after,total_mass_error_kg=after['mass_kg']-before['mass_kg'],
      compiled_full_tensor_max_error_kgm2=float(np.max(abs(compiled-inertia))),removed_legacy_native_geoms=removed,native_visual_geoms=native_names,
      floor_support=copy.deepcopy(floor_record),floor_collision_scope='Only inherited selected bay cable route; shell/doors and other equipment collision are not added.',
      wheel_and_rail_positions_scaled=False,rail_contour_center_distance_m=contract['rail_contour_center_distance_m'],wheel_reference_center_y_m=contract['wheel_center_y_m'],
      cad_width_m=evidence['source_outer_width_mm']/1000,legacy_width_m=contract['body_width_m'],incremental_dof=0,all_source_mounts_migrated=False,
      warning='CAD width/section/mass distribution and floor are integrated. Remaining installation points, door dimensions, pantograph mount, suspension stiffness and real masses remain uncalibrated candidates.')
    return result,spec,support
