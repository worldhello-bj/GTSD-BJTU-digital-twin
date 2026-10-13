(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDResearchRig=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const SCHEMA='gtsd-research-rig-session-v1';
const ACK='UNCALIBRATED_RESEARCH_NOT_AS_BUILT_NOT_WHOLE_CONTACT_ACCEPTANCE';
function rotate(q,v){const[w,x,y,z]=q,[a,b,c]=v,tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);return[a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx];}
function pose(position,q){if(!Array.isArray(position)||position.length!==3||!position.every(Number.isFinite)||!Array.isArray(q)||q.length!==4||!q.every(Number.isFinite)||Math.abs(Math.hypot(...q)-1)>1e-6)throw Error('Invalid native pose');return{position,q};}
function transform(vertices,reference,now){const qi=[reference.q[0],-reference.q[1],-reference.q[2],-reference.q[3]],out=new Array(vertices.length);for(let i=0;i<vertices.length;i+=3){const local=rotate(qi,vertices.slice(i,i+3).map((x,k)=>x-reference.position[k])),v=rotate(now.q,local);for(let k=0;k<3;k++)out[i+k]=v[k]+now.position[k];}return out;}
function resource(geometry,metadata,state,filter='all'){
 if(geometry?.schema!=='gtsd-research-rig-geometry-v1'||geometry.units!=='m'||metadata?.schema!=='gtsd-research-rig-metadata-v1'||state?.schema!==SCHEMA)throw Error('Research schemas required');
 if(state.as_built!==false||state.whole_contact_acceptance!==false||state.hardware_connected!==false)throw Error('Research-only boundary required');
 if(typeof state.server_generation_id!=='string'||!state.server_generation_id||geometry.server_generation_id!==state.server_generation_id||metadata.server_generation_id!==state.server_generation_id)throw Error('Stale server generation; refresh required');
 const active=state.initialized,contract=metadata.contract;
 if(active&&(!contract||geometry.session_id!==state.session_id||metadata.session_id!==state.session_id||geometry.identity_sha256!==state.identity_sha256||contract.parameters.mode!==state.mode))throw Error('Stale source/solver/session identity; refresh required');
 const bindings=new Map((active?contract.bindings:[]).map(x=>[x.mesh_name,x]));
 if(active&&bindings.size!==contract.bindings.length)throw Error('Duplicate bindings refused');
 const names=new Set(geometry.meshes.map(m=>m.name));for(const n of bindings.keys())if(!names.has(n))throw Error('Bound source geometry missing: '+n);
 let counts={bound:0,background:0,visible:0};
 const meshes=geometry.meshes.map(m=>{
   const row=bindings.get(m.name);let out={...m,id:m.name,group:'environment'};
   if(row){
     if(row.source_geometry_sha256!==m.geometry_sha256)throw Error('Source geometry identity mismatch: '+m.name);
     const native=state.body_poses[row.body_name];if(!native)throw Error('Native body pose missing: '+row.body_name);
     out.vertices=transform(m.vertices,pose(row.body_reference_position_m,row.body_reference_quaternion_wxyz),pose(native.position,native.quaternion_wxyz));
     out.visual_body=row.body_name;out.binding_status='native_pose_bound';counts.bound++;
   }else{out.vertices=m.vertices.slice();out.binding_status='static_unbound_background';out.color=[.66,.52,.22,.7];counts.background++;}
   return out;
 }).filter(m=>filter==='all'||m.branch===filter||((filter==='50DF4'||filter==='51409')&&m.name.includes(filter)));
 if(!meshes.length)throw Error('No source meshes in selected branch');counts.visible=meshes.length;
 return{resource:{schema:'gtsd-new-geometry-view-v1',units:'m',registry:[],meshes,as_built:false,research_rig_active:active},counts};
}
function requestIdentity(state,id){if(state?.schema!==SCHEMA||typeof state.server_generation_id!=='string'||!state.server_generation_id||typeof id!=='string'||!id)throw Error('Current research identity required');return{server_generation_id:state.server_generation_id,session_id:state.session_id,expected_revision:state.revision,request_id:id};}
function command(state,action,args={},id){if(!state?.initialized)throw Error('Initialize the research session first');const allowed={step:['steps','loads'],run:['steps','loads'],pause:[],reset:[],checkpoint:[],restore:['checkpoint_id']};if(!allowed[action]||Object.keys(args).some(k=>!allowed[action].includes(k)))throw Error('Unknown command field/action');if(['step','run'].includes(action)){if(!Number.isInteger(args.steps)||(action==='step'?![1,10,50].includes(args.steps):args.steps<1||args.steps>500))throw Error('Bounded native step count required');for(const [name,force] of Object.entries(args.loads||{})){if(!name||!Array.isArray(force)||force.length!==3||!force.every(Number.isFinite)||Math.hypot(...force)>20)throw Error('World force norm must be at most 20 N');}}return{action,...requestIdentity(state,id),...args};}
function initialization(state,mode,parameters,ack,id){if(!ack)throw Error('Acknowledge the stated research assumptions first');if(!['mount_only','pin_core_contact'].includes(mode)||!parameters||typeof parameters!=='object'||Array.isArray(parameters))throw Error('Explicit mode and parameter object required');return{...requestIdentity(state,id),mode,parameters,assumption_ack:ACK};}
function canAdopt(oldState,newState,allowGenerationChange=false){return newState?.schema===SCHEMA&&typeof newState.server_generation_id==='string'&&!!newState.server_generation_id&&Number.isInteger(newState.revision)&&newState.revision>=0&&(!oldState||(newState.server_generation_id===oldState.server_generation_id?newState.revision>=oldState.revision:allowGenerationChange));}
return{SCHEMA,ACK,rotate,pose,transform,resource,command,initialization,canAdopt};
});
