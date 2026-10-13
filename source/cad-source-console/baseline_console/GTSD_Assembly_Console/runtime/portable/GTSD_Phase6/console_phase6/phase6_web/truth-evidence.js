/* Pure read-only evidence presentation. No fetch, solver commands or hardware conversion. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDTruthEvidence=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const FIELDS=['dimensions','position','connection','quantity','material','mass','motion_relation'];
const LABELS={dimensions:'尺寸',position:'位置',connection:'连接',quantity:'数量',material:'材料',mass:'质量',motion_relation:'运动关系'};
function validate(value){
 if(!value||value.schema!=='gtsd-readonly-truth-overlay-v1'||!Array.isArray(value.rows)||value.rows.length!==109)throw new Error('证据矩阵格式或109登记节点数不匹配');
 const ids=new Set();for(const row of value.rows){if(typeof row.id!=='string'||ids.has(row.id)||!row.fields||FIELDS.some(k=>!row.fields[k]||typeof row.fields[k].status!=='string'))throw new Error('登记ID或七字段证据不完整');ids.add(row.id);}
 if(value.counts?.as_built_coverage_percentage!==null)throw new Error('禁止以登记节点/网格生成实物覆盖百分比');
 return value;
}
function textValue(value){if(value===null||value===undefined)return'未提供';if(typeof value==='object')return JSON.stringify(value,null,2);return String(value);}
function identity(row){return row.id==='DTB-C0020'?'CANDIDATE_IDENTITY':row.display_identity_status||row.identity_status||'未提供';}
function guards(row){
 const values=(row.semantic_guards||[]).map(g=>g.text).filter(x=>typeof x==='string');
 if(row.id==='DTB-C0020'&&!values.some(x=>x.includes('力路')))values.push('黑筒外形映射不证明气缸身份或升弓力路；身份仍为候选。');
 if(['DTB-C0073','DTB-C0074'].includes(row.id)&&!values.some(x=>x.includes('3000RPM')))values.push('3000RPM铭牌观察；模拟量速度输出候选，量程/轴号未确认，禁止4–20mA默认角度换算。');
 if(['DTB-C0013','DTB-C0067'].includes(row.id)&&!values.some(x=>x.includes('517.833')))values.push('517.833 mm轨轮廓中心距候选；478.333 mm仅为39.5 mm轨头假设下的算例。162.25/152.35 mm为半径，不是直径。');
 return values;
}
function describe(value,query=''){
 validate(value);const q=String(query).trim().toLocaleLowerCase();
 return{revision:value.evidence_revision_sha256,identity_match:value.geometry_identity_match,
  summary:'109 个登记节点 × 7 个独立真值字段 = 763 条字段记录；不是实物总数，也不是1:1覆盖率。',
  warning:value.coverage_warning,rows:value.rows.filter(r=>!q||JSON.stringify(r).toLocaleLowerCase().includes(q))};
}
function element(doc,tag,text,className){const e=doc.createElement(tag);if(text!==undefined)e.textContent=text;if(className)e.className=className;return e;}
function mount(container,value,{document:doc=globalThis.document,query='',onRefresh=()=>{}}={}){
 const description=describe(value,query);container.replaceChildren();
 container.append(element(doc,'h2','实物证据与模型差异（只读）'));
 container.append(element(doc,'p',description.summary,'truth-warning'));
 container.append(element(doc,'p',value.parameter_truth_warning||'外形映射、身份、连接及参数真值分别判断。','muted'));
 const match=description.identity_match===true?'当前solver源几何匹配':description.identity_match===false?'源几何不匹配；不得把本证据对应到当前模型':'尚无solver源几何可核对';
 container.append(element(doc,'p',match+' · 证据版本 '+String(description.revision),'truth-identity'));
 const candidates=element(doc,'details',undefined,'truth-design-candidates');candidates.append(element(doc,'summary','当前运行源几何与新CAD设计候选（未重绑定）'));
 const cadContext=value.current_solver_context?.variant==='cad_body_mounted';
 candidates.children[0].textContent=cadContext?'当前CAD部分迁移分支与独立设计资产身份':'当前运行源几何与新CAD设计候选（未重绑定）';
 candidates.append(element(doc,'p','当前运行源几何 SHA-256：'+textValue(value.current_running_geometry_identity)+(cadContext?'。当前是另行选择的CAD部分迁移分支，已使用新合成几何/完整惯量/底板支承；门、辅机安装等仍待迁移。旧109矩阵不因这个入口而改hash。':'。以下设计候选均未接入当前75/79DOF，不会自动修改质量、惯量、碰撞或检查点身份。')));
 for(const text of value.current_solver_context?.cad_migration_display_limits||[])candidates.append(element(doc,'p',text,'truth-warning'));
 for(const row of value.independent_design_candidates||[]){const item=element(doc,'section');item.append(element(doc,'h3',row.label||row.id));item.append(element(doc,'p','独立资产状态：'+textValue(row.status)+' · 通过证据面板自动绑定：false'+(cadContext&&row.id==='BODY-CAD-CROSS-SECTION-CANDIDATE'?'；当前CAD分支已使用其迁移派生资产，剩余安装未认证。':'')));item.append(element(doc,'p','独立候选几何 SHA-256：'+textValue(row.geometry_sha256)));item.append(element(doc,'pre',textValue(row)));candidates.append(item);}
 const review=value.encoder_independent_review;
 if(review?.available&&review.current_solver_binding===false&&review.enabled_by_default===false){const link=element(doc,'a','打开独立编码器候选审阅（不会替换当前模型）');link.href='/candidate-review/encoder';link.target='_blank';link.rel='noopener noreferrer';candidates.append(link);}
 else if(review?.error)candidates.append(element(doc,'p','独立编码器审阅不可用：'+review.error));
 const bodyReview=value.body_independent_review;
 if(bodyReview?.available&&bodyReview.current_solver_binding===false&&bodyReview.enabled_by_default===false){const link=element(doc,'a','打开新CAD车体独立审阅（局部坐标，待物理重绑定）');link.href='/candidate-review/body';link.target='_blank';link.rel='noopener noreferrer';candidates.append(element(doc,'p','新车体局部Z=0在车体底面，不当作原world坐标。'),link);}
 else if(bodyReview?.error)candidates.append(element(doc,'p','独立CAD车体审阅不可用：'+bodyReview.error));
 const pantoReview=value.pantograph_independent_review;
 if(pantoReview?.available&&pantoReview.allow_dynamics===false&&pantoReview.current_solver_binding===false){const link=element(doc,'a','打开受电弓局部CAD审阅（Z仅显示、配准未通过）');link.href='/candidate-review/pantograph';link.target='_blank';link.rel='noopener noreferrer';candidates.append(element(doc,'p','受电弓局部候选仅XY轮廓；禁止自动整车安装或闭链动力学。'),link);}
 else if(pantoReview?.error)candidates.append(element(doc,'p','受电弓局部候选不可用：'+pantoReview.error));
 container.append(candidates);
 const toolbar=element(doc,'div',undefined,'toolbar'),input=element(doc,'input');input.type='search';input.value=query;input.placeholder='部件ID、名称、证据状态或关键词';input.setAttribute('aria-label','只读证据筛选');
 const refresh=element(doc,'button','刷新证据');refresh.addEventListener('click',()=>onRefresh(input.value));toolbar.append(input,refresh);container.append(toolbar);
 const status=element(doc,'p',undefined,'muted'),list=element(doc,'div',undefined,'truth-evidence-list');container.append(status,list);
 function draw(){const rows=describe(value,input.value).rows;status.textContent='显示 '+rows.length+' / 109 个登记节点；刷新不会修改求解器或检查点身份。';list.replaceChildren();
  for(const row of rows){const card=element(doc,'details',undefined,'truth-row');card.append(element(doc,'summary',row.id+' · '+(row.name||'未命名')));
   card.append(element(doc,'p','外形映射：'+textValue(row.model_mapping?.geometry_status_from_registry)+'（仅显示绑定）'));
   card.append(element(doc,'p','实物身份：'+identity(row)+(row.identity_warning?' · '+row.identity_warning:'')));
   card.append(element(doc,'p','实物连接：'+row.fields.connection.status+' · 端点完备：'+textValue(row.fields.connection.physical_endpoints_complete)));
   for(const text of guards(row))card.append(element(doc,'p',text,'truth-warning'));
   const fields=element(doc,'div',undefined,'truth-fields');for(const key of FIELDS){const detail=element(doc,'details');detail.append(element(doc,'summary',LABELS[key]+'：'+row.fields[key].status));detail.append(element(doc,'pre',textValue(row.fields[key])));fields.append(detail);}card.append(fields);
   card.append(element(doc,'p','逐件实装验收标记：'+textValue(row.as_built_1to1_verified)+'；此标记不产生全装置覆盖率。'));
   const sources=element(doc,'details');sources.append(element(doc,'summary','来源与只读差异建议'));sources.append(element(doc,'pre',textValue({references:row.references,proposals:row.difference_proposals,next_evidence:row.minimum_next_evidence})));card.append(sources);list.append(card);
  }
 }
 input.addEventListener('input',draw);draw();return{queryInput:input,refreshButton:refresh,status,list};
}
function mountAuxiliary(container,value,{document:doc=globalThis.document}={}){
 if(!value||value.schema!=='gtsd-phase6-named-auxiliary-observations-v1')throw new Error('辅机命名观察格式不匹配');
 container.hidden=!value.mounted_variant;if(container.hidden)return;container.replaceChildren();
 container.append(element(doc,'h2','79DOF辅机原生命名观察'));
 for(const text of value.cad_migration_display_limits||[])container.append(element(doc,'p',text,'truth-warning'));
 container.append(element(doc,'p','只读真实命名端点；旧75DOF数组索引不复用。原生转角不是C0073/C0074硬件角度换算。','truth-warning'));
 container.append(element(doc,'p','状态 '+textValue(value.phase)+' · 求解 t='+textValue(value.sample_time_s)+' s · 快照年龄 '+textValue(value.snapshot_age_s)+' s'));
 const lines=(value.observations||[]).map(row=>row.id+' = '+(row.available?textValue(row.value)+' '+row.unit:'未提供（'+row.availability+'）'));
 container.append(element(doc,'pre',lines.join('\n')));
 const details=element(doc,'details');details.append(element(doc,'summary','安装反力与取样时刻（原生虚拟测点）'));details.append(element(doc,'p','内部安装传递诊断，不是另一份全局能量输入；尚无接受积分段时不伪造反力。'));details.append(element(doc,'pre',textValue(value.mount_sensor_records)));container.append(details);
 const world=element(doc,'details');world.append(element(doc,'summary','转子/定子/空气世界角速度（当前端点）'));world.append(element(doc,'p','rad/s，世界坐标向量；相对转速按真实轴投影。这里是瞬时端点观察，不能代替已接受积分区间的功。'));world.append(element(doc,'pre',textValue(value.world_endpoint_observations)));container.append(world);
}
return{FIELDS,LABELS,validate,textValue,identity,guards,describe,mount,mountAuxiliary};
});
