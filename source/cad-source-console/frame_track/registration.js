/* Additional reviewed visual ownership. No old-name/equality fields are forged. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDReviewedRegistration=api;})(globalThis,function(){
'use strict';
const vector=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(Number.isFinite);
async function witness(mesh){
  const bytes=new ArrayBuffer(4*(mesh.vertices.length+mesh.indices.length)),view=new DataView(bytes);
  mesh.vertices.forEach((x,i)=>view.setFloat32(i*4,x===0?0:x,true));
  mesh.indices.forEach((x,i)=>view.setUint32((mesh.vertices.length+i)*4,x,true));
  return Array.from(new Uint8Array(await globalThis.crypto.subtle.digest('SHA-256',bytes)),x=>x.toString(16).padStart(2,'0')).join('');
}
function pose(p){if(!p||!vector(p.position,3)||!vector(p.quaternion,4)||Math.abs(Math.hypot(...p.quaternion)-1)>1e-7)throw Error('Invalid registered body pose');return p;}
async function prepare(A,cache,catalog,bundle,addon){
  if(addon?.schema!=='gtsd-cad-frame-track-registration-addendum-v1'||!Array.isArray(addon.rows)||addon.rows.length!==235)throw Error('Unexpected registration addendum');
  if(addon.baseline.glb_sha256!==catalog.base_glb.sha256||addon.baseline.native_sha256!==catalog.native_sha256)throw Error('Source assembly identity mismatch');
  const accepted=addon.baseline.accepted_solver_identity,current=bundle.contract.accepted_identity;
  for(const key of ['model_identity_sha256','source_geometry_sha256','variant'])if(accepted[key]!==current[key])throw Error('Registration requires explicit new-identity acceptance');
  const by=new Map(bundle.meshes.map(x=>[x.name,x])),registered=new Map();
  for(const row of addon.rows){
    const mesh=cache.get(row.name),old=by.get(row.name);
    if(!mesh||registered.has(row.name)||old?.category!=='UNBOUND_SOURCE_OR_CANDIDATE'||old.latest_world_triangle_sha256!==row.source_triangle_sha256)throw Error('Registration mesh/source record mismatch');
    if(row.new_collision_allowed!==false||row.added_mass_kg!==0||row.added_dof!==0||!['frame','world'].includes(row.body_name))throw Error('Registration exceeds visual-only authority');
    if(row.mode!==(row.body_name==='frame'?'reviewed_visual_rigid_only':'reviewed_world_fixed_display_only'))throw Error('Registration mode/body mismatch');
    if(await witness(mesh)!==row.geometry_witness_sha256)throw Error('Registration vertex/index witness mismatch');
    // Private frozen copy makes later caller-array edits unable to alter vetted source geometry.
    registered.set(row.name,{row:Object.freeze({...row}),mesh:{...mesh,vertices:Object.freeze([...mesh.vertices]),indices:Object.freeze([...mesh.indices])}});
  }
  const reference=pose(addon.baseline.frame_reference_pose);
  if(JSON.stringify(reference)!==JSON.stringify(bundle.contract.reference_poses.frame))throw Error('Frame reference mismatch');
  return function follow(resource,state){
    const base=A.follow(resource,bundle,catalog,state);
    const now=pose(state.body_poses.frame),qi=[reference.quaternion[0],...reference.quaternion.slice(1).map(x=>-x)];
    let dynamic=0,world=0;
    const meshes=base.resource.meshes.map(mesh=>{
      const item=registered.get(mesh.name);if(!item)return mesh;
      const source=item.mesh,row=item.row;
      if(row.body_name==='world'){world++;return {...source,reviewed_registration:row.mode,physics_contact_active:false};}
      const vertices=new Array(source.vertices.length);
      for(let i=0;i<vertices.length;i+=3){
        const local=A.rotate(qi,source.vertices.slice(i,i+3).map((x,k)=>x-reference.position[k])),p=A.rotate(now.quaternion,local);
        for(let k=0;k<3;k++)vertices[i+k]=p[k]+now.position[k];
      }
      dynamic++;return {...source,vertices,visual_body:'frame',reviewed_registration:row.mode,physics_contact_active:false};
    });
    return {resource:{...base.resource,meshes},summary:{...base.summary,
      dynamic_visual_meshes:base.summary.dynamic_visual_meshes+dynamic,
      world_static_meshes:base.summary.world_static_meshes+world,
      unbound_or_static_reference_meshes:base.summary.unbound_or_static_reference_meshes-dynamic-world,
      newly_reviewed_dynamic_visual_meshes:dynamic,newly_reviewed_world_display_meshes:world,
      newly_registered_native_collision_meshes:0,newly_registered_mass_kg:0}};
  };
}
return{prepare,witness};
});
