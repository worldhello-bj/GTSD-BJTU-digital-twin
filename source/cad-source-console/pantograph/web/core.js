(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDPantoCandidate=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const MODE='H1_DUAL_FOURBAR_PLANAR';
function pose(p){if(!p||!Array.isArray(p.position)||p.position.length!==3||!p.position.every(Number.isFinite)||!Array.isArray(p.quaternion)||p.quaternion.length!==4||!p.quaternion.every(Number.isFinite)||Math.abs(Math.hypot(...p.quaternion)-1)>1e-6)throw Error('Candidate pose invalid');return p;}
function rotate(q,v){const[w,x,y,z]=q,[a,b,c]=v,tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);return[a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx];}
function resource(catalog,cache,metadata,state){
 if(metadata?.schema!=='gtsd-pantograph-candidate-metadata-v1'||metadata.production_allowed!==false)throw Error('Research metadata required');
 const review=metadata.review,binding=metadata.binding,rows=new Map(review.meshes.map(r=>[r.mesh_name,r])),moving=new Map(binding.meshes.map(r=>[r.mesh_name,r]));
 if(review.source_glb_gzip_sha256!==catalog.base_glb.gzip_sha256)throw Error('Candidate source GLB identity mismatch');
 const active=state?.initialized===true;
 if(active&&(state.mode!==MODE||state.schema!=='gtsd-pantograph-candidate-session-v1'||state.production_allowed!==false||state.model_identity_sha256!==metadata.candidate_model_identity_sha256||!['paused','stepping','error'].includes(state.phase)))throw Error('Wrong candidate solver identity');
 let counts={moving:0,base:0,unknown:0,legacy:0};
 const meshes=catalog.meshes.filter(m=>m.group==='pantograph').map(spec=>{
  const m=cache.get(spec.name),r=rows.get(spec.name);if(!m||!r)throw Error('Source mesh/review missing '+spec.name);let out={...m};
  if(moving.has(spec.name)){
   counts.moving++;out.color=[.13,.77,.88,1];const row=moving.get(spec.name);if(!row.body_name.startsWith('cad_panto_'))throw Error('Old solver namespace rejected');
   if(active){const ref=pose(binding.reference_poses[row.body_name]),now=pose(state.body_poses?.[row.body_name]);const qi=[ref.quaternion[0],-ref.quaternion[1],-ref.quaternion[2],-ref.quaternion[3]];out.vertices=new Array(m.vertices.length);for(let i=0;i<m.vertices.length;i+=3){const local=rotate(qi,m.vertices.slice(i,i+3).map((v,k)=>v-ref.position[k])),v=rotate(now.quaternion,local);for(let k=0;k<3;k++)out.vertices[i+k]=v[k]+now.position[k];}out.visual_body=row.body_name;}
  }else if(r.assignment_status==='SOURCE_STATIC_SUPPORT_CANDIDATE'){counts.base++;out.color=[.34,.47,.58,1];}
  else if(r.assignment_status==='BASELINE_RIGID_BINDING_RETAINED'){counts.legacy++;out.color=[.40,.40,.43,1];}
  else{counts.unknown++;out.color=[1,.57,.12,.68];}
  return out;
 });
 if(meshes.length!==74||counts.moving!==20||counts.base!==17||counts.unknown!==26||counts.legacy!==11)throw Error('74-part audit mismatch');
 return {resource:{schema:'gtsd-new-geometry-view-v1',units:'m',registry:[],meshes,new_physics_enabled:false,research_candidate_active:active},counts};
}
function command(s,action,args,id){if(s?.schema!=='gtsd-pantograph-candidate-session-v1'||!s.initialized||s.mode!==MODE||typeof s.session_id!=='string')throw Error('Initialize H1 first');if(!['step','pause','reset','checkpoint','restore'].includes(action))throw Error('Unbounded/unknown action refused');if(action==='step'&&(!Number.isInteger(args.steps)||args.steps<1||args.steps>50||!Number.isFinite(args.torque_Nm)||Math.abs(args.torque_Nm)>.1))throw Error('Step/torque bounds');return{action,session_id:s.session_id,expected_revision:s.revision,request_id:id,...args};}
return{MODE,resource,command,rotate};
});
