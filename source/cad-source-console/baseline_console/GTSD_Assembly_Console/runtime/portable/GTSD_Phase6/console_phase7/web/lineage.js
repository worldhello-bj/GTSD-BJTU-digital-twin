/* Read-only semantic selection: source geometry evidence and native values stay separate. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDLineage=api;})(typeof window!=='undefined'?window:globalThis,function(){
'use strict';
const text=v=>v===null||v===undefined?'未提供':typeof v==='object'?JSON.stringify(v):String(v);
function timeText(o){return '求解 t='+text(o.sample_time_s)+' s · '+text(o.phase)+' · 快照年龄 '+text(o.snapshot_age_s)+' s。分母是软件信号定义，不是实物覆盖率。';}
function validate(c,o){if(c?.schema!=='gtsd-semantic-lineage-registry-v1'||o?.schema!=='gtsd-semantic-lineage-observations-v1')throw new Error('谱系格式未识别');return o.identity_match===true&&c.catalogue_sha256===o.catalogue_sha256&&c.current_geometry_sha256===o.current_geometry_sha256&&c.model_identity_sha256===o.model_identity_sha256;}
function truthLink(c,mesh,truth){
 const row=(truth?.rows||[]).find(r=>r.id===mesh.registered_component_id);if(!row)return{row:null,status:'NO_REGISTERED_COMPONENT_TRUTH_ROW'};
 const hash=v=>typeof v==='string'&&/^[0-9a-f]{64}$/.test(v);
 if(!hash(truth?.source_geometry_sha256))return{row:null,status:'UNAVAILABLE_TRUTH_SOURCE_IDENTITY'};
 const lists=[row.model_mapping?.direct_mesh_ids,row.model_mapping?.candidate_mesh_ids],selectors=lists.flatMap(v=>Array.isArray(v)?v.filter(id=>typeof id==='string'):[]);
 const retained=mesh.lineage==='EXACT_BASELINE_MESH_RETAINED'&&hash(c.baseline_registry_source_geometry_sha256)&&truth.source_geometry_sha256===c.baseline_registry_source_geometry_sha256;
 const current=hash(c.current_geometry_sha256)&&truth.source_geometry_sha256===c.current_geometry_sha256;
 if(!selectors.includes(mesh.id)||!retained&&!current)return{row:null,status:'UNAVAILABLE_TRUTH_GEOMETRY_OR_EXACT_SELECTOR'};
 return{row,status:retained?'BASELINE_ROW_THROUGH_VERIFIED_RETAINED_MESH':'CURRENT_GEOMETRY_ROW_EXACT_MESH_SELECTOR'};
}
function selection(c,o,ids,truth){
 const valid=validate(c,o),byId=new Map((o.observations||[]).map(r=>[r.id,r])),defs=new Map(c.signals.map(r=>[r.id,r])),items=new Map(c.items.map(r=>[r.id,r]));
 return {valid,rows:ids.map(id=>{const m=c.meshes.find(m=>m.id===id);if(!m)return{id,unknown:true};const item=items.get('mesh.'+id),link=truthLink(c,m,truth);return{...m,truth_link_status:link.status,values:[...new Set([...(item?.signal_ids||[]),...(c.mesh_direct_signal_groups?.[item?.direct_signal_group]||[]),...(c.body_context_signal_groups?.[item?.body_context_signal_group]||[])])].map(id=>({...defs.get(id),...(byId.get(id)||{available:false,value:null,availability:'MISSING_IN_CURRENT_SNAPSHOT'}),available:valid&&byId.get(id)?.available===true})),truth:link.row};})};
}
function guards(t){const rows=(t.semantic_guards||[]).map(x=>x.text).filter(x=>typeof x==='string');if(t.id==='DTB-C0020')rows.push('黑筒身份/升弓力路仍为候选，外形映射不证明气缸身份。');if(['DTB-C0073','DTB-C0074'].includes(t.id))rows.push('3000RPM为速度输出候选；禁止4–20mA默认角度换算，轴号/量程未核实。');if(['DTB-C0013','DTB-C0067'].includes(t.id))rows.push('517.833 mm为轨轮廓中心距候选；478.333 mm为39.5 mm轨头假设算例，162.25/152.35 mm是半径。');return rows;}
function elem(doc,tag,t,cls){const e=doc.createElement(tag);if(t!==undefined)e.textContent=t;if(cls)e.className=cls;return e;}
function mount(container,c,o,{document:doc=globalThis.document,selected=[],truth=null,query='',visibleIds=[],onSelect=()=>{},onRefresh=()=>{}}={}){
 const valid=validate(c,o);container.replaceChildren();container.append(elem(doc,'h2','新几何谱系 · 当前原生测点'));
 container.append(elem(doc,'p','逐件网格谱系、原生求解读数与实物真值分开。点击画面或列表只定位，不施加命令；旧登记expected hash未修改。','lineage-warning'));
 container.append(elem(doc,'p','辅助气压峰差与切换事件仍未证明全状态收敛。门锁扣力为独立端点 mj_forward 观察，叶侧世界XYZ力的框侧反力取负；不得另记区间输入功。软件响应和短程恢复通过不代表全系统认证。','lineage-warning'));
 const status=elem(doc,'p',valid?'当前模型/几何/注册身份一致，读数仍未标定':'当前身份或采样注册不匹配，关联暂停','lineage-status');container.append(status);
 container.append(elem(doc,'p',timeText(o),'lineage-time'));
 container.append(elem(doc,'pre',JSON.stringify(o.denominators||{},null,2)));
 const bar=elem(doc,'div',undefined,'toolbar'),input=elem(doc,'input'),refresh=elem(doc,'button','刷新只读证据');input.type='search';input.value=query;input.placeholder='网格名称、部件ID或body名字';input.setAttribute('aria-label','谱系网格筛选');refresh.addEventListener('click',()=>onRefresh());bar.append(input,refresh);container.append(bar);
 const results=elem(doc,'div',undefined,'lineage-results'),detail=elem(doc,'div',undefined,'lineage-selected');container.append(results,detail);
 const visible=new Set(visibleIds);
 function list(){results.replaceChildren();const q=input.value.trim().toLocaleLowerCase(),rows=c.meshes.filter(m=>!q||[m.id,m.name,m.body_name,m.registered_component_id].join(' ').toLocaleLowerCase().includes(q));results.append(elem(doc,'p','匹配 '+rows.length+' 条资产记录，最多显示60条；不是实物数量。'));
 for(const m of rows.slice(0,60)){const button=elem(doc,'button',m.name+' · '+m.lineage);button.disabled=!valid||!visible.has(m.id)||!m.visible_in_dynamic_view;button.addEventListener('click',()=>{if(!button.disabled)onSelect([m.id]);});results.append(button);}}
 input.addEventListener('input',list);list();
 const picked=selection(c,o,selected,truth);
 if(!selected.length)detail.append(elem(doc,'p','在三维画面单击可见网格，或按名称筛选后定位。拖动仍只调整视角。'));
 for(const row of picked.rows){const box=elem(doc,'section');detail.append(box);box.append(elem(doc,'h3',row.name||row.id));if(row.unknown){box.append(elem(doc,'p','此对象没有已验证源网格谱系；可能是研究代理。无实物测点映射。'));continue;}
 box.append(elem(doc,'p','外形谱系：'+row.lineage+' · 原生刚体 '+text(row.body_name)+' · 安装状态 '+text(row.mounting_status)));
 box.append(elem(doc,'p','实装验证 false · 硬件标定 false。刚体共有读数是运动上下文，不是该网格上有传感器。','lineage-warning'));
 box.append(elem(doc,'pre','当前几何 '+row.current_geometry_sha256+'\n逐件内容 '+row.canonical_mesh_sha256));
 const values=elem(doc,'pre');values.textContent=row.values.length?row.values.map(v=>(v.label||v.id)+' = '+(v.available?text(v.value)+' '+(v.unit||''):'Unavailable ('+(valid?v.availability:'IDENTITY_MISMATCH')+')')).join('\n'):'Unavailable (NO_SEMANTIC_SIGNAL_MAPPING)';box.append(values);
 if(row.truth){const t=row.truth;box.append(elem(doc,'h4','原109登记部件证据：'+t.id+' · '+(t.name||'')));box.append(elem(doc,'p','身份等级 '+text(t.id==='DTB-C0020'?'CANDIDATE_IDENTITY':t.display_identity_status||t.identity_status||t.physical_identity_status)+'；实装验收 '+text(t.as_built_1to1_verified)));
 for(const g of guards(t))box.append(elem(doc,'p',g,'lineage-warning'));
 for(const[k,v]of Object.entries(t.fields||{}))box.append(elem(doc,'p',k+'：'+text(v.status)));
 box.append(elem(doc,'pre',JSON.stringify({identity:t.identity_warning,source_mapping:t.model_mapping,references:t.references},null,2)));
 }else box.append(elem(doc,'p','没有确认的109部件身份连接。设计来源/刚体绑定不能补成实物身份。'+row.truth_link_status));
 box.append(elem(doc,'pre',JSON.stringify(row.source_refs||[],null,2)));}
 return{input,refresh,results,detail,status};
}
function pick(view,x,y,width,height,G){
 if(!view?.resource||view.unavailable||!Number.isFinite(x)||!Number.isFinite(y)||!(width>0&&height>0)||x<0||y<0||x>width||y>height)return null;
 if(typeof view.draw==='function')view.draw();if(!view.gl&&view.softwareStats?.blocked)return null;const camera=view.lastCamera;if(!camera)return null;
 const px=x/width*2-1,py=1-y/height*2;let hit=null;
 for(const m of view.resource.meshes){if(!G.visibleMesh(m,view.view)||(m.color?.[3]??1)<=0)continue;
 const v=m.vertices,p=new Float64Array(v.length);for(let k=0;k<v.length;k+=3){const a=v[k],b=v[k+1],c=v[k+2];p[k]=camera[0]*a+camera[4]*b+camera[8]*c+camera[12];p[k+1]=camera[1]*a+camera[5]*b+camera[9]*c+camera[13];p[k+2]=camera[2]*a+camera[6]*b+camera[10]*c+camera[14];}
 for(let k=0;k<m.indices.length;k+=3){const a=m.indices[k]*3,b=m.indices[k+1]*3,c=m.indices[k+2]*3,den=(p[b+1]-p[c+1])*(p[a]-p[c])+(p[c]-p[b])*(p[a+1]-p[c+1]);if(Math.abs(den)<1e-14)continue;
 const u=((p[b+1]-p[c+1])*(px-p[c])+(p[c]-p[b])*(py-p[c+1]))/den,w=((p[c+1]-p[a+1])*(px-p[c])+(p[a]-p[c])*(py-p[c+1]))/den;if(u< -1e-10||w< -1e-10||u+w>1+1e-10)continue;
 const z=u*p[a+2]+w*p[b+2]+(1-u-w)*p[c+2];if(z< -1||z>1||hit&&z>=hit.depth)continue;hit={id:m.id,name:m.name,depth:z,source:'CURRENT_RENDERED_TRIANGLE',proxy:m.physics_binding?.mode==='solver_proxy'||m.display_group==='solver_proxy'};}}
 return hit;
}
function gesture(a,b){return !!a&&a.button===0&&b.button===0&&a.pointerId===b.pointerId&&Math.hypot(a.x-b.x,a.y-b.y)<=4;}
return{validate,selection,mount,pick,gesture,text,guards,timeText,truthLink};
});
