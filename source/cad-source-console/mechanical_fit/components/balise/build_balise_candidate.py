#!/usr/bin/env python3
"""Build a reversible, source-constrained fit candidate, not an as-built claim.
Reads supplied source root only. Output files are confined to --output-dir.
"""
import argparse, copy, json, math
from pathlib import Path
import numpy as np
from balise_geometry import load_candidate_geometry, triangle_hash, classify, slab_y_bounds, overlap_chords, closed

HERE=Path(__file__).resolve().parent
ROOT=None  # Supply --project-root; no machine-specific source path is embedded.

def write(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def assembly_names(index):
    return [f'ENV_balise_body_{index:02}',f'ENV_balise_yellow_lid_{index:02}']+['ENV_balise_lid_screw'+('' if j==0 else f'.{j:03}') for j in range(index*4,index*4+4)]

def make(root,config,out):
    seed=json.loads((HERE/'balise_source_seed.json').read_text())
    geo,cat,source_identity=load_candidate_geometry(root,seed); before=dict(geo); after=dict(geo)
    chairs={r['old_chair']:r for r in seed['source_chair_groups']}
    existing_rows={r['replaces_mesh_name']:r for r in seed['chair_source_transforms']}
    if any(not np.isfinite(config[k]) or config[k]<0 for k in ['minimum_sleepers_side_clearance_mm','minimum_rail_side_clearance_mm','minimum_clip_rail_side_clearance_mm','minimum_clip_chair_edge_margin_mm']):raise ValueError('Clearance parameters must be finite and nonnegative')
    if len(config['assemblies'])!=2 or {a['balise_index'] for a in config['assemblies']}!={2,3}:raise ValueError('This profile requires both complete balise02/03 assemblies')
    if config.get('body_dimension_changes_allowed') is not False:raise ValueError('This profile only allows rigid transforms, no size edits')
    if config.get('deck_z_offset_mm') != 0:raise ValueError('Deck mounting hypothesis requires the unchanged inherited deck Z')
    transforms=[];mesh_rows=[];remove=[];assembly_info=[];moved=[]
    main_candidates=[n for n in geo if n == 'ENV_main_rail_1' and geo[n][0].mean(0)[1]>0]
    if len(main_candidates)!=1:raise ValueError('Positive-Y main rail must be unique')
    main=main_candidates[0];branch='DETAIL_BENCH_turnout_inner_right_segment'
    for spec in config['assemblies']:
        idx=spec['balise_index'];names=assembly_names(idx)
        if any(n not in geo for n in names):raise ValueError('Incomplete six-mesh balise assembly')
        body=names[0];bv,bf=geo[body];bc=(bv.min(0)+bv.max(0))/2
        k1,k2=spec['between_source_stations']
        if spec['linked_source_chair_station']!={2:12,3:17}[idx]:raise ValueError('This task has fixed source chair correspondences for balise02 and03')
        if spec['linked_source_chair_station'] not in [k1,k2]:raise ValueError('Only the two adjacent source bays are admitted for this local candidate')
        if k2!=k1+1:raise ValueError('A bay must be bounded by consecutive source sleepers')
        sleepers=[f'DETAIL_BENCH_long_sleeper_{k:02}' for k in [k1,k2]]
        va,vb=[geo[n][0] for n in sleepers]
        # This bounded profile supports the nearby unrotated source sleeper bays only.
        if any(np.ptp(np.unique(np.round(v[:,0],8))) > .081 for v in [va,vb]):raise ValueError('Selected tilted or unusual sleeper needs a dedicated planar fit')
        bay_left=float(va[:,0].max());bay_right=float(vb[:,0].min())
        if bay_right<=bay_left:raise ValueError('Invalid source sleeper ordering')
        x=(bay_left+bay_right)/2
        angle=float(spec['yaw_deg'])
        if angle not in [0,90,180,270]:raise ValueError('Choose one of the explicitly checkable orthogonal orientation candidates')
        ang=np.deg2rad(angle); rot=np.array([[round(float(np.cos(ang)),15),-round(float(np.sin(ang)),15),0],[round(float(np.sin(ang)),15),round(float(np.cos(ang)),15),0],[0,0,1]])
        rotated=(bv-bc)@rot.T;hx=max(abs(rotated[:,0]));hy=max(abs(rotated[:,1]))
        margin=config['minimum_sleepers_side_clearance_mm']/1000
        gap=bay_right-bay_left
        if hx*2+margin*2>gap+1e-9:raise ValueError(f'Balise {idx:02}: body width {hx*2000:.6f} mm plus side clearances will not fit source bay {gap*1000:.6f} mm')
        # Use the entire assembly X/Y envelope conservatively for rail spacing.
        av=np.concatenate([geo[n][0] for n in names]); ar=(av-bc)@rot.T
        ax0=x+ar[:,0].min();ax1=x+ar[:,0].max();ay0=ar[:,1].min();ay1=ar[:,1].max()
        branch_bound=slab_y_bounds(*geo[branch],ax0,ax1)
        main_bound=slab_y_bounds(*geo[main],ax0,ax1)
        railmargin=config['minimum_rail_side_clearance_mm']/1000
        lower=branch_bound[1]-ay0+railmargin;upper=main_bound[0]-ay1-railmargin
        if lower>upper:raise ValueError('No rail corridor for the unchanged complete assembly')
        y=(lower+upper)/2 if spec.get('lateral_center_m') is None else float(spec['lateral_center_m'])
        if not lower-1e-9<=y<=upper+1e-9:raise ValueError('Requested lateral placement violates finite rail triangle corridor')
        dst=bc.copy();dst[0]=x;dst[1]=y
        mat=np.eye(4);mat[:3,:3]=rot;mat[:3,3]=dst-rot@bc
        for n in names:
            v,f=geo[n];nv=v@rot.T+mat[:3,3];after[n]=(nv,f);moved.append(n);remove.append(n)
            newname=f'FIT_BALISE_{idx:02}_'+n.removeprefix('ENV_balise_')
            meta={'geometry_status':'PHOTO_INFERRED_GEOMETRY','installation_status':'PHOTO_SUPPORTED_MOUNTING_FAMILY_PARAMETER_CANDIDATE','as_built_confirmed':False,'new_collision_enabled':False,'physics_status':'STATIC_GEOMETRY_ONLY_NO_CURRENT_SOLVER_BINDING','candidate_profile':config['profile_id'],'replaces_mesh_name':n,'source_refs':cat[n]['source_metadata']['source_refs'],'source_body_dimensions_unchanged':True,'source_triangle_sha256':triangle_hash(v,f),'triangle_sha256':triangle_hash(nv,f),'old_baked_world_to_new_baked_world_matrix':mat.tolist()}
            color=[.93,.64,.08,1] if 'yellow_lid' in n else ([.3,.32,.34,1] if 'screw' in n else [.49,.51,.50,1])
            mesh_rows.append({'name':newname,'vertices':nv.reshape(-1).tolist(),'indices':f.reshape(-1).tolist(),'group':'environment','color':color,'metadata':meta})
            transforms.append({'old':n,'new':newname,'matrix':mat.tolist(),'source_triangle_sha256':meta['source_triangle_sha256'],'triangle_sha256':meta['triangle_sha256']})
        chair_name=f"DETAIL_BENCH_branch_chair_{spec['linked_source_chair_station']:02}_2"
        cg=chairs[chair_name]
        clip_seating=[]
        for n in cg['old_mesh_names']:
            v,f=geo[n];r=existing_rows[n];remove.append(n)
            clip_delta=0.0
            if 'fastener' in n:
                side=-1 if n.endswith('_-1') else 1
                railbounds=slab_y_bounds(*geo[branch],float(v[:,0].min()),float(v[:,0].max()))
                cm=config['minimum_clip_rail_side_clearance_mm']/1000
                clip_delta=max(0.0,railbounds[1]+cm-float(v[:,1].min())) if side==1 else min(0.0,railbounds[0]-cm-float(v[:,1].max()))
                v=v.copy();v[:,1]+=clip_delta;after[n]=(v,f);moved.append(n)
                cv=geo[chair_name][0];margin=config['minimum_clip_chair_edge_margin_mm']/1000
                edge_margin=min(float(v[:,1].min()-cv[:,1].min()),float(cv[:,1].max()-v[:,1].max()),float(v[:,0].min()-cv[:,0].min()),float(cv[:,0].max()-v[:,0].max()))
                if edge_margin<margin-1e-9:raise ValueError('Outboard clip candidate would leave the existing chair footprint')
                # All clip projection vertices sit well within the central full-width
                # rectangular portion of the bevelled chair (verified at mesh level below).
                footprint_points=np.unique(v[:,:2],axis=0)
                from balise_geometry import inside
                chair_mid_z=float((cv[:,2].min()+cv[:,2].max())/2)
                if not all(inside(*geo[chair_name],np.array([q[0],q[1],chair_mid_z])) for q in footprint_points):raise ValueError('Actual clip projection is not fully inside the chair material section')
                actual_gap=float(v[:,1].min()-railbounds[1]) if side==1 else float(railbounds[0]-v[:,1].max())
                clip_seating.append({'name':n,'side':side,'world_Y_delta_from_source_group_mm':clip_delta*1000,'rail_triangle_slab_Y_bounds_m':railbounds,'actual_rail_side_gap_mm':actual_gap*1000,'minimum_chair_plan_AABB_edge_margin_mm':edge_margin*1000,'every_clip_XY_vertex_inside_actual_chair_material_midsection':True,'as_built_confirmed':False})
            meta={'geometry_status':'CAD_SOURCE_POSITION_WITH_INHERITED_PROXY_FORM','as_built_confirmed':False,'new_collision_enabled':False,'physics_status':'STATIC_GEOMETRY_ONLY_NO_CURRENT_SOLVER_BINDING','candidate_profile':config['profile_id'],'replaces_mesh_name':n,'source_sleeper_handles':cg['source_sleeper_handles'],'source_rail_handles':cg['source_rail_handles'],'old_baked_world_to_new_baked_world_matrix':r['old_baked_world_to_new_baked_world_matrix'],'source_triangle_sha256':r['old_triangle_sha256'],'triangle_sha256':triangle_hash(v,f)}
            if 'fastener' in n:
                meta['geometry_status']='INHERITED_CLIP_SHAPE_WITH_SOURCE_RAIL_CONSTRAINED_OUTBOARD_SEATING_CANDIDATE'
                meta['clip_outboard_delta_from_source_group_mm']=clip_delta*1000
                matclip=np.array(r['old_baked_world_to_new_baked_world_matrix']);matclip[1,3]+=clip_delta
                meta['old_baked_world_to_new_baked_world_matrix']=matclip.tolist()
            newname='FIT_BALISE_SOURCE_'+n.removeprefix('DETAIL_BENCH_')
            mesh_rows.append({'name':newname,'vertices':v.reshape(-1).tolist(),'indices':f.reshape(-1).tolist(),'group':'environment','color':[.44,.46,.48,1] if 'chair' in n else [.19,.21,.23,1],'metadata':meta})
        assembly_info.append({'balise_index':idx,'clip_seating_candidates':clip_seating,'names':names,'linked_chair_names':cg['old_mesh_names'],'source_station_handles':cg['source_sleeper_handles'],'source_rail_handles':cg['source_rail_handles'],'between_source_sleepers':sleepers,'body_dimensions_before_mm':((bv.max(0)-bv.min(0))*1000).tolist(),'body_dimensions_after_mm':((after[body][0].max(0)-after[body][0].min(0))*1000).tolist(),'source_bay_open_bounds_X_m':[bay_left,bay_right],'body_each_side_geometric_gap_mm':((gap-2*hx)/2)*1000,'yaw_deg':angle,'center_before_m':bc.tolist(),'center_after_m':dst.tolist(),'center_delta_mm':((dst-bc)*1000).tolist(),'rail_corridor_center_Y_bounds_m':[lower,upper],'rail_constraints_meshes':[branch,main],'finite_triangle_slab_X_m':[ax0,ax1],'actual_assembly_branch_rail_Y_gap_mm':(y+ay0-branch_bound[1])*1000,'actual_assembly_main_rail_Y_gap_mm':(main_bound[0]-y-ay1)*1000,'deck_Z_unchanged':bool(np.array_equal(bv[:,2],after[body][0][:,2])),'matrix':mat.tolist(),'as_built_confirmed':False})
    # Global moved-vs-all check. The other 1,921 meshes are never suppressed.
    pairs=[];seen=set()
    for n in moved:
        for other in geo:
            if n==other:continue
            key='|'.join(sorted([n,other]))
            if key in seen:continue
            seen.add(key)
            def broad(state):
                a=state[n][0];b=state[other][0]
                return bool(np.all(np.minimum(a.max(0),b.max(0))-np.maximum(a.min(0),b.min(0))>1e-9))
            if broad(before) or broad(after):
                pairs.append({'pair':key,'before':classify(before[n],before[other]),'after':classify(after[n],after[other])})
    unresolved=[r for r in pairs if r['after']['status'] not in ['NO_POSITIVE_VOLUME_AABB','TRIANGULATED_MESHES_SEPARATED']]
    # Recompute exact historical three witnesses from actual triangles.
    history={'new_positive_overlap_pairs':seed['original_pending_targets']}
    targets=[]
    for r in history['new_positive_overlap_pairs']:
        if '|ENV_balise_body_' not in r['pair']:continue
        a,b=r['pair'].split('|');ch=overlap_chords(before[a],before[b],r['witness_world_m'])
        targets.append({'pair':r['pair'],'before':{'status':'POSITIVE_MATERIAL_OVERLAP','witness_world_m':r['witness_world_m'],'shared_material_chords':ch},'after':classify(after[a],after[b])})
    if len(targets)!=3:raise ValueError('Expected exactly three originally pending balise material overlaps')
    invariants=[]
    for t in transforms:
        v,f=before[t['old']];nv,nf=after[t['old']];m=np.array(t['matrix']);rot=m[:3,:3]
        invariants.append({'name':t['old'],'closed_mesh':closed(v,f),'triangles_unchanged':bool(np.array_equal(f,nf)),'Z_vertices_unchanged':bool(np.array_equal(v[:,2],nv[:,2])),'orthogonality_error':float(np.max(abs(rot.T@rot-np.eye(3)))),'rotation_determinant':float(np.linalg.det(rot)),'vertex_transform_residual_m':float(np.max(abs(v@rot.T+m[:3,3]-nv)))})
    report={'schema':'balise-fit-real-triangle-validation-v1','passes_geometric_candidate_gate':not unresolved,'source_input_identity':source_identity,'scope':'Current default scene assembled from hash-checked GLB, scene extras and active source_replacements, plus the two pending source-position chair/clip groups; 12 balise assembly meshes rigidly transformed plus four inherited clip heads seated outboard against the source rail footprint. Not whole-vehicle physical clearance certification.','target_pair_count':3,'targets':targets,'all_moved_vs_all_pairs_with_positive_AABB_before_or_after':pairs,'remaining_after_review_pairs':unresolved,'assemblies':assembly_info,'rigid_geometry_invariants':invariants,'triangle_check_tolerance_m':1e-8,'method':'Positive AABB broadphase; actual triangle internal-point parity witness with XYZ ray exits; otherwise exhaustive triangle AABB, six edge-triangle tests, coplanar 2D SAT and every in-bounds unique-vertex containment. No negative grid result is accepted as separation.','no_source_rails_sleepers_or_chair_shapes_changed':True,'four_clip_shapes_unchanged_outboard_seating_candidate':True,'no_hardware_or_physics_certification':True}
    profile={'schema':'gtsd-mechanical-fit-profile-v1','id':config['profile_id'],'label':'应答器枕间安装候选（保尺寸90°转向）','default_enabled':False,'as_built_confirmed':False,'configuration':config,'remove_names':remove,'add_meshes':mesh_rows,'additional_mutually_exclusive_names':[existing_rows[n]['name'] for a in assembly_info for n in a['linked_chair_names']],'group_moves_atomic':True,'groups':assembly_info,'review_warning':'保留照片推定尺寸；枕间/轨间安装族有照片依据，但02/03身份、确切bay、横向坐标和90°方向尚未实测。四个旧推定扣头按原轨底边外侧0.5mm参数就座，形状不变；此相对偏置未确认实装。几何通过不等于实装确认。','physics_new_collision':False,'physics_added_mass_kg':0,'physics_added_bodies':0,'support_z_policy':'Inherited body bottom retained at deck top; not raised onto sleepers.','source_dimensions_changed':False,'source_CAD_rails_sleepers_and_native_fasteners_moved_or_reshaped':False,'validation_file':'balise_fit_validation.json'}
    out.mkdir(parents=True,exist_ok=True)
    write(out/'balise_fit_profile.json',profile);write(out/'balise_fit_validation.json',report)
    write(out/'balise_transforms.json',{'schema':'balise-rigid-transform-v1','transforms':transforms,'as_built_confirmed':False})
    write(out/'balise_installation_contract.json',{'schema':'balise-installation-contract-v1','config':config,'assemblies':assembly_info,'source_relocation_dependency':'38 track/sleeper revisions; chair groups 12_2 and 17_2 source positions','remove_names':remove,'replacements':[{'old':r['metadata']['replaces_mesh_name'],'new':r['name']} for r in mesh_rows],'group_moves_atomic':True,'full_scene_gate_pass':not unresolved,'as_built_confirmed':False,'not_verified':['Exact balise 02/03 identity or unique station in photographs','90 degree mounting orientation','Manufacturer dimensions and tolerances','Brackets, mounting holes, cable access and service clearances','Electromagnetic placement and passing-vehicle dynamic envelope']})
    if unresolved:raise ValueError(f'{len(unresolved)} after pairs require review; profile is not admissible')
    print(json.dumps({'pass':True,'replace_meshes':len(remove),'candidate_meshes':len(mesh_rows),'target_pairs_resolved_geometrically':len(targets),'before_positive_pairs':sum(p['before']['status']=='POSITIVE_MATERIAL_OVERLAP' for p in pairs),'after_positive_or_unresolved_pairs':len(unresolved),'assemblies':assembly_info},ensure_ascii=False,indent=2))
    return profile,report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project-root','--source-root',dest='source_root',type=Path,required=True);p.add_argument('--config',type=Path,default=HERE/'fit_config.json');p.add_argument('--output-dir',type=Path,default=HERE);a=p.parse_args()
    make(a.source_root,json.loads(a.config.read_text()),a.output_dir)
