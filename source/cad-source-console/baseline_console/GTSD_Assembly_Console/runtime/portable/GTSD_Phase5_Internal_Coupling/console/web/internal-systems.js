/* Read-only source registration and live observations; no physical commands. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDInternalSystems=api;})(typeof window!=='undefined'?window:globalThis,function(){
'use strict';
const labels={REGISTERED_NAME_WITH_SEPARATE_GEOMETRY_EVIDENCE:'登记名称，几何证据另列',IDEAL_SOLVER_NOT_HARDWARE_CALIBRATED:'理想求解观察，未硬件标定',IMPLEMENTED_SOLVER_TOPOLOGY_NOT_PHYSICAL_PIPING:'已实现求解拓扑，实物管路未确认',SOLVER_EQUIVALENT_UNCALIBRATED:'未标定求解量',PRESENT_UNIT_UNSPECIFIED:'有值但单位未定',MISSING_IN_CURRENT_SNAPSHOT:'当前求解器无此字段',SOURCE_GEOMETRY_MISMATCH:'源几何身份不符，关联停用',UNAVAILABLE_HARDWARE_UNBOUND:'未绑定实物接口',NULL_IN_CURRENT_SNAPSHOT:'当前值为空',INVALID_NONFINITE:'拒绝非有限数值',INVALID_TYPE:'拒绝字段类型不符',confirmed:'来源确认',direct:'来源直接对应',candidate:'候选对应',engineering_assumption:'工程假设',unverified:'未核实',available:'已读取求解值',unavailable:'当前模型无此字段',missing:'当前模型无此字段',not_available:'当前模型无此字段'};
function safe(v){return v===null||v===undefined?'未提供':typeof v==='object'?JSON.stringify(v):String(v);}
function displayValue(row){
  const present=Object.prototype.hasOwnProperty.call(row,'value')&&row.value!==null&&row.value!==undefined;
  const value=present?safe(row.value):'未提供';
  return (row.label||row.path||row.telemetry_path||'观察量')+' = '+value+(row.unit?' '+row.unit:'')+' · '+(labels[row.availability]||row.availability||(present?'求解器观察':'当前无值'));
}
function describe(registry,observed,visibleIds){
  if(!registry||!Array.isArray(registry.items))return null;
  const records=new Map((observed?.observations||[]).map(v=>[v.id,v]));
  const visible=visibleIds instanceof Set?visibleIds:new Set(visibleIds||[]);
  const items=registry.items.map(row=>{
    const record=records.get(row.id),values=record?.values||(record?[record]:[]);
    const targets=observed?.geometry_identity_match===false?[]:(row.geometry_mesh_ids||[]).filter(id=>visible.has(id));
    return {...row,targets,record,values,text:values.length?values.map(displayValue).join('\n'):'当前模型未提供对应求解值',evidence:labels[row.evidence_level]||row.evidence_level||'证据等级未提供'};
  });
  const edgeValues=new Map((observed?.edges||[]).map(e=>[e.id,e]));
  const edges=(registry.edges||[]).map(e=>({...e,...(edgeValues.get(e.id)||{availability:'CURRENT_TOPOLOGY_NOT_VERIFIED',available:false})}));
  return {items,edges,limits:registry.limits||[],sample_time_s:observed?.sample_time_s??null};
}
function edgeText(edge){
  const from=edge.from||edge.source||edge.from_id||'未提供',to=edge.to||edge.target||edge.to_id||'未提供';
  return (edge.label||edge.id||'连接')+'：'+safe(from)+' → '+safe(to)+' · '+(labels[edge.evidence_level]||edge.evidence_level||'求解拓扑；不证明实物管路')+' · '+(edge.available===true?'当前求解器有此连接量':'当前未验证或不可用');
}
function mount(container,registry,observed,{visibleIds=[],onSelect=()=>{},document:doc=globalThis.document}={}){
  const open=new Set(Array.from(container.children||[]).filter(c=>Array.from(c.children||[]).some(x=>x.open)).map(c=>c._registryId)),scroll=container.scrollTop||0;
  const result=describe(registry,observed,visibleIds);container.replaceChildren();
  if(!result)return null;
  for(const row of result.items){
    const card=doc.createElement('section');card.className='internal-card';card._registryId=row.id;
    const heading=doc.createElement('h3');heading.textContent=row.label||row.id;card.append(heading);
    const badge=doc.createElement('p');badge.className='muted';badge.textContent=row.evidence+(row.assumption?' · '+safe(row.assumption):'');card.append(badge);
    const values=doc.createElement('pre');values.textContent=row.text;card.append(values);
    const select=doc.createElement('button');select.textContent=row.targets.length?'定位 '+row.targets.length+' 个当前可见源网格':'无可见、可信几何对应';select.disabled=!row.targets.length;
    select.addEventListener('click',()=>{if(row.targets.length)onSelect([...row.targets]);});card.append(select);
    const refs=doc.createElement('details'),title=doc.createElement('summary'),body=doc.createElement('pre');title.textContent='来源与映射路径';body.textContent=JSON.stringify({id:row.id,source_refs:row.source_refs||[],body_names:row.body_names||[],telemetry_paths:row.telemetry_paths||[]},null,2);refs.open=open.has(row.id);refs.append(title);refs.append(body);card.append(refs);container.append(card);
  }
  container.scrollTop=scroll;
  return result;
}
function topology(container,registry,observed,{onNode=()=>{},document:doc=globalThis.document}={}){
  container.replaceChildren();
  if(!doc.createElementNS)return {drawn:false,reason:'SVG DOM unavailable'};
  const model=describe(registry,observed,[]),all=model?.edges||[];
  const edges=all.filter(e=>e.available===true&&e.kind==='pneumatic_solver_edge'&&typeof e.from==='string'&&typeof e.to==='string');
  const ids=[...new Set(edges.flatMap(e=>[e.from,e.to]))].sort();
  if(!ids.length)return {drawn:false,reason:'No available native gas topology in this snapshot'};
  const ns='http://www.w3.org/2000/svg',svg=doc.createElementNS(ns,'svg'),byId=new Map((registry.items||[]).map(r=>[r.id,r]));
  function el(tag,attrs){const e=doc.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs||{}))e.setAttribute(k,String(v));return e;}
  svg.setAttribute('viewBox','0 0 760 440');svg.append(el('rect',{x:0,y:0,width:760,height:440,fill:'#0b1118'}));svg.setAttribute('role','img');svg.setAttribute('aria-label','当前求解器有效气路拓扑示意，非实物管路');
  const defs=el('defs'),marker=el('marker',{id:'internal-flow-arrow',viewBox:'0 0 10 10',refX:10,refY:5,markerWidth:5,markerHeight:5,orient:'auto'}),tip=el('path',{d:'M 0 0 L 10 5 L 0 10 z',fill:'#759db5'});marker.append(tip);defs.append(marker);svg.append(defs);
  const positions=new Map(ids.map((id,i)=>{const angle=2*Math.PI*i/ids.length-Math.PI/2;return[id,[380+290*Math.cos(angle),220+160*Math.sin(angle)]];}));
  const pairCounts=new Map();
  for(const edge of edges){const a=positions.get(edge.from),b=positions.get(edge.to),key=[edge.from,edge.to].sort().join('|'),n=pairCounts.get(key)||0;pairCounts.set(key,n+1);const dx=b[0]-a[0],dy=b[1]-a[1],length=Math.max(1,Math.hypot(dx,dy)),bend=(n%2?1:-1)*(16+14*Math.floor(n/2)),cx=(a[0]+b[0])/2-dy/length*bend,cy=(a[1]+b[1])/2+dx/length*bend;
    const path=el('path',{d:`M ${a[0]+dx/length*19} ${a[1]+dy/length*19} Q ${cx} ${cy} ${b[0]-dx/length*20} ${b[1]-dy/length*20}`,fill:'none',stroke:'#486a85','stroke-width':1.3,'marker-end':'url(#internal-flow-arrow)'}),title=el('title');title.textContent=edgeText(edge)+(Number.isFinite(edge.flow_kg_s)?'\n'+edge.flow_kg_s+' kg/s':'');path.append(title);svg.append(path);}
  for(const id of ids){const row=byId.get(id),[x,y]=positions.get(id),g=el('g',{tabindex:0,role:'button','aria-label':(row?.label||id)+'，定位登记和源几何'}),circle=el('circle',{cx:x,cy:y,r:17,fill:'#173b4c',stroke:'#7ac7d4','stroke-width':2}),label=el('text',{x,y:y+32,'text-anchor':'middle',fill:'#cee5ef','font-size':11});let short=row?.label||id;const brake=id.match(/brake_axle_(\d)_(minus|plus)$/);if(brake)short='轴'+brake[1]+' '+(brake[2]==='minus'?'−':'+')+'侧制动缸（假设）';if(id==='solver.ambient')short='大气边界（模型）';if(short.length>16)short=short.slice(0,15)+'…';label.textContent=short;const full=el('title');full.textContent=row?.label||id;g.append(full);g.append(circle);g.append(label);g.addEventListener('click',()=>onNode(id));g.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();onNode(id);}});svg.append(g);}
  container.append(svg);return{drawn:true,node_count:ids.length,edge_count:edges.length,excluded_edge_count:all.length-edges.length};
}
return {displayValue,describe,edgeText,mount,topology};
});
