/* Pure command/readiness and live-pose adaptation. No trajectories or solver. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDLiveUI=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
function readiness(state,busy){const ready=!!state&&['paused','running'].includes(state.phase)&&!!state.plant,stepping=!!state?.single_steps_remaining,recoverable=ready||state?.phase==='error'&&!!state.plant;return{run:ready&&!busy&&state.phase==='paused'&&!stepping,pause:ready&&!busy&&(state.phase==='running'||stepping),step:ready&&!busy&&state.phase==='paused'&&!stepping,new_run:recoverable&&!busy,restore_run:recoverable&&!busy&&!!state.previous_run_available,restore_checkpoint:recoverable&&!busy,ports:ready&&!busy,export:ready&&!busy};}
function validateState(s){if(!s||s.schema!=='gtsd-v5-console-state-v1'||typeof s.session_id!=='string'||!['building','paused','running','error','closed'].includes(s.phase))throw new Error('控制台状态格式不匹配');if(s.plant){const p=s.plant;if(p.schema!=='gtsd-v5-live-plant-state-v1'||!Number.isFinite(p.time_s)||p.time_s<0||!p.body_poses||!p.telemetry||!Number.isFinite(p.dt_s)||p.dt_s<=0)throw new Error('缺少真实求解器姿态或时间');}return s;}
function liveResource(source,binding,state,hash,physics){validateState(state);if(!state.plant)throw new Error('模型尚无可显示的求解状态');if(state.plant.source_geometry_sha256!==hash)throw new Error('求解器几何身份与显示源不一致');return physics.atFrame(source,binding,{schema:'gtsd-physics-trajectory-v1',subsystem:state.plant.binding_subsystem||'integrated',frames:[{time_s:state.plant.time_s,body_poses:state.plant.body_poses,telemetry:state.plant.telemetry}]},0,hash);}
function pendingResult(state,pending){if(!pending)return null;return(state.events||[]).find(e=>e.request_id===pending.request_id&&e.session_id===pending.session_id)||null;}
function checkpointFile(value){if(!value||value.schema!=='gtsd-v5-console-checkpoint-v1'||typeof value.sha256!=='string'||!value.plant)throw new Error('不是本控制台的完整检查点；观察快照和旧缓存不能恢复');return value;}
function makeCommand(state,action,args,id){validateState(state);if(!state.plant)throw new Error('模型尚未就绪');return{session_id:state.session_id,request_id:id,action,args};}
function searchSource(source,binding,query){const q=String(query||'').trim().toLocaleLowerCase();if(!q)return[];const byName=new Map((binding?.meshes||[]).map(b=>[b.mesh_name,b]));return(source?.meshes||[]).filter(m=>[m.name,m.registered_component_id].some(v=>String(v||'').toLocaleLowerCase().includes(q))).map(m=>{const b=byName.get(m.name);return{id:m.id,name:m.name,registered_component_id:m.registered_component_id||null,body_name:b?.body_name||null,binding_mode:b?.mode||'unbound',visible_dynamic:b?.mode==='rigid'};});}
function viewPresets(subsystem){const benches={cover_bench:'独立维护台',brake_contact_bench:'独立片盘接触研究台',cables_support:'独立线缆支承研究台',legacy_pantograph_bench:'旧双闭链研究台',legacy_drawbar_bench:'旧双车牵引杆研究台'};return benches[subsystem]?[['bench',benches[subsystem]]]:[['platform','整车＋环境'],['whole','整车'],['bogie','转向架'],['pantograph','受电弓'],['body','门与舱体（定位）'],['cutaway','剖视']];}
function primitiveProxy(p){
 if(!['box','cylinder'].includes(p.kind)||typeof p.body_name!=='string'||!Array.isArray(p.position_world_m)||p.position_world_m.length!==3||!p.position_world_m.every(Number.isFinite)||!Array.isArray(p.rotation_world_matrix)||p.rotation_world_matrix.length!==3||!p.rotation_world_matrix.every(row=>Array.isArray(row)&&row.length===3&&row.every(Number.isFinite))||!Array.isArray(p.size_m)||p.size_m.length!==(p.kind==='box'?3:2)||!p.size_m.every(x=>Number.isFinite(x)&&x>0))throw new Error('真实box/cylinder代理格式无效');
 const R=p.rotation_world_matrix,dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 for(let i=0;i<3;i++)for(let j=0;j<3;j++)if(Math.abs(dot(R[i],R[j])-(i===j?1:0))>1e-7)throw new Error('代理旋转矩阵非正交');
 if(Math.abs(dot(R[0],cross(R[1],R[2]))-1)>1e-7)throw new Error('代理旋转矩阵手性无效');
 const local=[],indices=[];
 if(p.kind==='box'){
  const [x,y,z]=p.size_m;for(const a of [[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])local.push(a);
  indices.push(0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7);
 }else{
  const [r,h]=p.size_m,n=24;for(const z of [-h,h])for(let k=0;k<n;k++)local.push([r*Math.cos(2*Math.PI*k/n),r*Math.sin(2*Math.PI*k/n),z]);
  local.push([0,0,-h],[0,0,h]);for(let k=0;k<n;k++){const next=(k+1)%n;indices.push(k,next,k+n,next,next+n,k+n,2*n,next,k,2*n+1,k+n,next+n);}
 }
 const vertices=local.flatMap(v=>R.map((row,i)=>p.position_world_m[i]+dot(row,v)));
 return{id:'solver-proxy:'+(p.geom_name||p.body_name),name:p.label||p.geom_name||p.body_name,group:p.group||'bogie',display_group:'solver_proxy',color:[1.,.38,.08,1.],vertices,indices,physics_binding:{mode:'solver_proxy',body_name:p.body_name,geom_name:p.geom_name||null,note:'Actual solver '+p.kind+' pose; replaceable uncalibrated engineering geometry, not a confirmed source part.'}};
}
function proxyMeshes(proxies){
 if(!Array.isArray(proxies))throw new Error('求解器代理列表格式错误');
 const meshes=[],seen=new Set(),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>{const n=Math.hypot(...a);if(n<1e-12)throw new Error('代理轴长度无效');return a.map(v=>v/n);};
 for(const p of proxies){
  const key=p.geom_name||p.body_name;if(seen.has(key))throw new Error('重复真实代理');
  if(p.kind==='box'||p.kind==='cylinder'){meshes.push(primitiveProxy(p));seen.add(key);continue;}
  if(p.kind!=='capsule'||typeof p.body_name!=='string'||seen.has(key)||![p.a_world_m,p.b_world_m].every(v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite))||!Number.isFinite(p.radius_m)||p.radius_m<=0)throw new Error('无效或重复的真实capsule代理');
  seen.add(key);const a=p.a_world_m,b=p.b_world_m,u=unit(b.map((v,i)=>v-a[i])),seed=Math.abs(u[2])<.9?[0,0,1]:[0,1,0],v=unit(cross(u,seed)),w=cross(u,v),vertices=[],indices=[],rings=[],segments=12;
  // Triangulate the actual solver capsule for display only. This surface is
  // explicitly tagged as a numerical proxy, never a reconstructed source mesh.
  for(let j=0;j<=4;j++)rings.push({center:a,angle:Math.PI-j*Math.PI/8});
  for(let j=0;j<=4;j++)rings.push({center:b,angle:Math.PI/2-j*Math.PI/8});
  for(const ring of rings)for(let k=0;k<segments;k++){const theta=k*2*Math.PI/segments,radial=p.radius_m*Math.sin(ring.angle),axial=p.radius_m*Math.cos(ring.angle);for(let i=0;i<3;i++)vertices.push(ring.center[i]+u[i]*axial+radial*(v[i]*Math.cos(theta)+w[i]*Math.sin(theta)));}
  for(let j=0;j<rings.length-1;j++)for(let k=0;k<segments;k++){const x=j*segments+k,y=j*segments+(k+1)%segments,z=(j+1)*segments+k,t=(j+1)*segments+(k+1)%segments;indices.push(x,y,z,y,t,z);}
  meshes.push({id:'solver-proxy:'+p.body_name,name:p.label||p.body_name,group:'body',display_group:'solver_proxy',color:[1.,.38,.08,1.],vertices,indices,physics_binding:{mode:'solver_proxy',body_name:p.body_name,note:'Actual solver capsule; not the original flexible source surface.'}});
 }
 return meshes;
}
return{readiness,validateState,liveResource,pendingResult,checkpointFile,makeCommand,proxyMeshes,searchSource,viewPresets};
});
