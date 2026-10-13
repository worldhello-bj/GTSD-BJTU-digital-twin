/* Source-backed carrier visuals and coils driven only by current native sites. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDBogieRegistration=api;})(globalThis,function(){
'use strict';
const finite=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(Number.isFinite),dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),sub=(a,b)=>a.map((x,i)=>x-b[i]),add=(a,b)=>a.map((x,i)=>x+b[i]),scale=(a,s)=>a.map(x=>x*s),norm=a=>Math.hypot(...a),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function position(site){if(site?.source!=='mujoco.site_xpos'||!finite(site.position,3))throw Error('Current native site pose required');return site.position;}
function pose(value){if(!value||!finite(value.position,3)||!finite(value.quaternion,4)||Math.abs(norm(value.quaternion)-1)>1e-7)throw Error('Finite native rigid pose required');return value;}
function deform(A,vertices,referenceLower,referenceUpper,lower,upper){
  const rv=sub(referenceUpper,referenceLower),cv=sub(upper,lower),rl=norm(rv),cl=norm(cv);
  if(!(rl>1e-8&&cl>1e-8))throw Error('Collapsed coil endpoints');
  const u=scale(rv,1/rl),v=scale(cv,1/cl),d=Math.max(-1,Math.min(1,dot(u,v)));
  let q;if(d>-1+1e-10){q=[1+d,...cross(u,v)];q=scale(q,1/norm(q));}else{let axis=cross(u,Math.abs(u[0])<.9?[1,0,0]:[0,1,0]);axis=scale(axis,1/norm(axis));q=[0,...axis];}
  const out=new Array(vertices.length);
  for(let i=0;i<vertices.length;i+=3){const delta=sub(vertices.slice(i,i+3),referenceLower),t=dot(delta,u)/rl,radial=sub(delta,scale(rv,t)),p=add(add(lower,scale(cv,t)),A.rotate(q,radial));for(let k=0;k<3;k++)out[i+k]=p[k];}
  return out;
}
async function prepare(A,W,cache,catalog,addon,acceptedIdentity,options={}){
  if(addon?.schema!=='gtsd-bogie-cad-visual-registration-v1'||addon.baseline.glb_sha256!==catalog.base_glb.sha256)throw Error('Bogie source identity mismatch');
  if(options.enableUncalibratedCarrierVisuals!==true)throw Error('Explicit research carrier visual opt-in required');
  if(!acceptedIdentity||['model_identity_sha256','source_geometry_sha256','variant'].some(k=>typeof acceptedIdentity[k]!=='string'||!acceptedIdentity[k]))throw Error('Explicit accepted native identity required');
  if(['model_identity_sha256','source_geometry_sha256','variant'].some(k=>addon.baseline.accepted_solver_identity?.[k]!==acceptedIdentity[k]))throw Error('Addendum admission identity does not match explicit native identity');
  const meshes=new Map();
  for(const row of [...addon.rigid_rows,...addon.coil_rows]){const name=row.name||row.mesh_name,m=cache.get(name);
    if(!m||meshes.has(name)||row.new_collision_allowed!==false||row.added_mass_kg!==0||row.added_dof!==0||await W.witness(m)!==row.geometry_witness_sha256)throw Error('Bogie mesh witness/scope failure');
    meshes.set(name,{...m,vertices:Object.freeze([...m.vertices]),indices:Object.freeze([...m.indices])});
  }
  if(options.enableUncalibratedCadSeatFit===true)for(const row of addon.coil_rows)for(const [site,key] of [[row.site_lower,'new_lower_world_m'],[row.site_upper,'new_upper_world_m']]){
    if(norm(sub(position(options.referenceSitePoses?.[site]),row[key]))>1e-10)throw Error('Reference native sites do not match applied CAD seat fit');
  }
  return function apply(resource,state){
    if(!A.runtimeStatus(state)||['model_identity_sha256','source_geometry_sha256','variant'].some(k=>state[k]!==acceptedIdentity[k]))throw Error('Unaccepted native identity or stale state');
    const present=new Set(resource.meshes.map(x=>x.name)),updates=new Map();let movingCarrier=0,movingFrame=0;
    for(const row of addon.rigid_rows){if(!present.has(row.name))continue;
      const source=meshes.get(row.name),ref=pose(options.referenceBodyPoses?.[row.body_name]),now=pose(state.body_poses?.[row.body_name]),qi=[ref.quaternion[0],...ref.quaternion.slice(1).map(x=>-x)],vertices=new Array(source.vertices.length);
      for(let i=0;i<vertices.length;i+=3){const p=add(now.position,A.rotate(now.quaternion,A.rotate(qi,sub(source.vertices.slice(i,i+3),ref.position))));for(let k=0;k<3;k++)vertices[i+k]=p[k];}
      updates.set(row.name,{...source,vertices,visual_body:row.body_name,physics_contact_active:false,reviewed_registration:'source_supported_visual_rigid_candidate'});
      if(row.body_name==='frame')movingFrame++;else movingCarrier++;
    }
    let movingCoils=0,staticCoils=0;
    for(const row of addon.coil_rows){if(!present.has(row.mesh_name))continue;
      const source=meshes.get(row.mesh_name),application=state.cad_seat_application;
      const enabled=options.enableUncalibratedCadSeatFit===true&&application?.schema==='gtsd-near-side-cad-spring-seat-application-v1'&&application.uncalibrated_research_opt_in===true;
      if(enabled&&options.cadSeatContractSha256&&application.contract_sha256!==options.cadSeatContractSha256)throw Error('Native seat application contract mismatch');
      const lo=state.native_spring_sites?.[row.site_lower],up=state.native_spring_sites?.[row.site_upper];
      if(!enabled||!lo||!up){staticCoils++;updates.set(row.mesh_name,{...source,color:[1,.55,.15,.5],physics_contact_active:false,reviewed_registration:'static_coil_requires_accepted_cad_seat_application_and_native_sites'});continue;}
      const lower=position(lo),upper=position(up);movingCoils++;
      updates.set(row.mesh_name,{...source,vertices:deform(A,source.vertices,row.new_lower_world_m,row.new_upper_world_m,lower,upper),visual_endpoint_sites:[row.site_lower,row.site_upper],physics_contact_active:false,reviewed_registration:'two_native_site_endpoint_deformable_visual_only'});
    }
    return {resource:{...resource,meshes:resource.meshes.map(m=>updates.get(m.name)||m)},summary:{new_carrier_visual_meshes:movingCarrier,new_frame_visual_meshes:movingFrame,native_site_coil_visual_meshes:movingCoils,static_review_coil_meshes:staticCoils,new_collision_meshes:0,new_mass_kg:0}};
  };
}
return{prepare,deform};
});
