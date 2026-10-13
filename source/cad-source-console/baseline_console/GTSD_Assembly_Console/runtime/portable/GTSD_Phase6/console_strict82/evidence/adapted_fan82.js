(function(root){
'use strict';
const esc=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const valid=x=>x&&x.identity_match===true&&x.variant==='cad_doors_four_fans_scoped_rollback'&&/^[0-9a-f]{64}$/.test(x.model_identity_sha256)&&/^[0-9a-f]{64}$/.test(x.source_geometry_sha256);
function render(data,catalog){
 if(!valid(data))return '<h2>四门风机原生测点</h2><p role="status">Unavailable：未连接到身份已校验的82DOF快照；不显示旧值。</p>';
 const table=(data.observations||[]);const ids=catalog&&catalog.model_identity_sha256===data.model_identity_sha256&&catalog.current_geometry_sha256===data.source_geometry_sha256?catalog.meshes:[];
 const identity='<p>显式 cad_doors_four_fans_scoped_rollback · '+esc(data.inventory?.rotating_axes)+' 旋转轴 + '+esc(data.inventory?.sliding_axes)+' 活塞 · 模型 '+esc(data.model_identity_sha256)+'<br>当前几何 '+esc(data.source_geometry_sha256)+' · t='+esc(data.sample_time_s)+' s · step='+esc(data.steps)+'</p>';
 const blocks=Object.entries(data.fans||{}).map(([key,fan])=>{
  const mesh=ids.find(m=>m.name===fan.mesh_name);const button=mesh?'<button type="button" data-fan-mesh="'+esc(mesh.id)+'">选择当前定子罩</button>':'<span>几何选择 Unavailable</span>';
  const rows=table.filter(x=>x.fan===key).map(x=>'<tr><th>'+esc(x.field)+'</th><td>'+esc(x.available?Array.isArray(x.value)?x.value.map(n=>Number(n).toPrecision(7)).join(', '):typeof x.value==='number'?x.value.toPrecision(7):x.value:'Unavailable: '+x.availability)+'</td><td>'+esc(x.unit)+'</td></tr>').join('');
  return '<details><summary>'+esc(key)+' · '+esc(fan.rotor)+'</summary>'+button+'<table><tbody>'+rows+'</tbody></table></details>';
 }).join('');
 const work='<p>新增功列（J，按原生名称映射，不重复累加）：'+esc(JSON.stringify(data.extra_work_columns_J))+'；单次计账核对 '+esc(data.extra_work_columns_counted_once)+'</p>';
 return '<h2>四门风机原生测点 · 研究模型未标定</h2>'+identity+blocks+work+'<ul>'+(data.limits||[]).map(s=>'<li>'+esc(s)+'</li>').join('')+'</ul>';
}
function createClient({fetch,draw,select,setTimer,clearTimer}){
 let stopped=false,epoch=0,timer=null,current=null,catalog=null;
 async function refresh(){
  const token=++epoch;
  try{
   const response=await fetch('/api/four-door-fans',{cache:'no-store'});if(!response.ok)throw Error('HTTP '+response.status);const next=await response.json();if(!valid(next))throw Error('Invalid native identity');
   if(!catalog||catalog.current_geometry_sha256!==next.source_geometry_sha256||catalog.model_identity_sha256!==next.model_identity_sha256){const r=await fetch('/api/lineage-registry',{cache:'no-store'});if(!r.ok)throw Error('Catalogue unavailable');const c=await r.json();if(c.current_geometry_sha256!==next.source_geometry_sha256||c.model_identity_sha256!==next.model_identity_sha256)throw Error('Catalogue identity mismatch');if(stopped||token!==epoch)return;catalog=c;}
   if(stopped||token!==epoch)return;current=next;draw(render(current,catalog));
  }catch(e){if(stopped||token!==epoch)return;current=null;catalog=null;draw(render(null,null));}
  if(!stopped&&token===epoch)timer=setTimer(refresh,1500);
 }
 function choose(id){if(!valid(current)||!catalog||catalog.current_geometry_sha256!==current.source_geometry_sha256||!catalog.meshes.some(m=>m.id===id))return false;select([id]);return true;}
 function stop(){stopped=true;++epoch;current=null;catalog=null;if(timer!==null)clearTimer(timer);draw(render(null,null));}
 return {refresh,choose,stop};
}
const api={render,valid,createClient};if(typeof module==='object'&&module.exports)module.exports=api;
if(root&&root.document){const host=root.document.getElementById('four-fan-panel');if(!host)return;const c=createClient({fetch:root.fetch.bind(root),draw:html=>host.innerHTML=html,select:ids=>root.GTSDLineageView?.select(ids),setTimer:root.setTimeout.bind(root),clearTimer:root.clearTimeout.bind(root)});host.addEventListener('click',e=>{const b=e.target.closest?.('[data-fan-mesh]');if(b)c.choose(b.dataset.fanMesh);});root.addEventListener('pagehide',()=>c.stop(),{once:true});c.refresh();}
})(typeof window!=='undefined'?window:null);
