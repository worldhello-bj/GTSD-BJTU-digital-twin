/* Read-only projection of accepted solver records; no commands or force law. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDContactSections=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const number=x=>Number.isFinite(x)?x:null;
const sum=values=>values.every(Number.isFinite)?values.reduce((a,b)=>a+b,0):null;
const fmt=(x,unit)=>x===null?'未提供':Number(x).toExponential(6)+' '+unit;
function describe(plant){
 if(plant?.variant!=='contact-sections-research')return null;
 const c=plant.telemetry?.wheel_rail_contact;if(!c)return{error:'缺少实际轮轨记录；没有估算替代值。'};
 const current=c.current_state_evaluation,applied=c.applied_last_interval,old=c.actual_old_state_comparison;
 if(!current?.records||!applied?.records||!old?.records)return{error:'接触端点/已施加区间记录不完整。'};
 const interval=c.applied_interval_exists===true,groups=[],expected=[];
 for(const axle of [1,2])for(const side of [-1,1]){
  const names=[`wheel_${axle}_${side}_section_-1`,`wheel_${axle}_${side}_section_1`,`candidate_flange_${axle}_${side}`];
  const points=names.map((name,index)=>{
   expected.push(name);const r=current.records[name],a=applied.records[name],o=old.records[name];
   if(!r||!a||!o)return{name,label:index<2?'踏面截面 '+(index===0?'-1':'+1'):'候选轮缘',missing:true};
   const wn=interval?number(a.actual_midpoint_normal_work_J):null,wf=interval?number(a.actual_midpoint_friction_work_J):null;
   const u=number(r.storage_J),u0=number(o.storage_J),power=number(a.normal_dissipation_W),dt=number(plant.dt_s);
   const remainder=[wn,u,u0,power,dt].every(Number.isFinite)?wn+u-u0-power*dt:null;
   return{name,label:index<2?'踏面截面 '+(index===0?'-1':'+1'):'候选轮缘',active:r.active===true,
    normal_force_N:number(r.normal_force_N),gap_m:r.gap_is_active_penetration===true?number(r.gap_m):null,
    gap_status:r.gap_is_active_penetration===true?'当前接触穿透，负值':'本条记录未提供分离间隙；记录中的0是占位',
    current_storage_J:u,last_normal_work_J:wn,last_friction_work_J:wf,
    last_normal_remainder_J:remainder,normal_remainder_is_derived_read_only:true};
  });
  groups.push({wheel:`轴 ${axle} / 侧 ${side>0?'+1':'-1'}`,points,
   current_tread_normal_N:sum(points.slice(0,2).map(p=>p.normal_force_N)),
   current_candidate_flange_normal_N:points[2].normal_force_N??null,
   last_normal_work_J:sum(points.map(p=>p.last_normal_work_J)),
   last_normal_remainder_J:sum(points.map(p=>p.last_normal_remainder_J))});
 }
 const column=name=>{const i=c.work_columns_J?.indexOf(name);return i>=0?number(c.cumulative_work_J?.[i]):null;};
 return{time_s:number(plant.time_s),dt_s:number(plant.dt_s),evaluation_time_s:number(current.time_s),
  last_interval_start_s:interval?number(applied.time_s):null,last_interval_exists:interval,
  groups,unexpected_record_names:Object.keys(current.records).filter(n=>!expected.includes(n)),
  cumulative_normal_work_J:column('actual_midpoint_normal'),
  cumulative_normal_dissipation_work_J:column('evaluated_velocity_normal_dissipation'),
  storage_change_J:number(c.storage_change_J),normal_remainder_J:number(c.normal_integration_remainder_J),
  last_normal_remainder_J:number(c.last_normal_and_friction_integration_remainder_J?.[0]),
  cumulative_friction_work_J:column('actual_midpoint_friction'),
  native_wheel_contact_count:number(c.native_wheel_contact_count),
  midpoint_residual:number(c.midpoint_corrector?.actual_midpoint_residual),
  footprint_assumption:c.footprint_assumption||'未提供'};
}
function summaryText(v){
 if(v.error)return v.error;
 return ['当前端点 t = '+fmt(v.evaluation_time_s,'s'),
  v.last_interval_exists?'上一已接受区间 ['+fmt(v.last_interval_start_s,'s')+', '+fmt(v.time_s,'s')+']':'尚无已接受积分区间',
  '累计实际法向功：'+fmt(v.cumulative_normal_work_J,'J'),
  '累计有符号法向耗散功：'+fmt(v.cumulative_normal_dissipation_work_J,'J'),
  '接触储能变化：'+fmt(v.storage_change_J,'J'),
  '累计法向数值余项：'+fmt(v.normal_remainder_J,'J'),
  '上一段法向数值余项：'+fmt(v.last_normal_remainder_J,'J'),
  '累计实际摩擦功：'+fmt(v.cumulative_friction_work_J,'J'),
  '原生轮轨重复接触数：'+(v.native_wheel_contact_count??'未提供'),
  '实际中点速度残差：'+fmt(v.midpoint_residual,'rad/s 或 m/s（各广义自由度原生单位）'),
  '完成步进、零warning或余项符号均不表示全状态收敛、严格逐步被动或实机安全。',
  '法向余项 = 法向功 + 储能变化 - 有符号法向耗散功。以下逐点余项由当前保存量只读计算，不是新增累计账本。',
  v.unexpected_record_names.length?'另有未分组记录：'+v.unexpected_record_names.join(', '):'4轮 × 双踏面截面 + 候选轮缘，共12个登记点。'].join('\n');
}
function groupText(g){return ['踏面合计 '+fmt(g.current_tread_normal_N,'N')+'；候选轮缘 '+fmt(g.current_candidate_flange_normal_N,'N'),...g.points.map(p=>p.missing?p.label+'：记录缺失':[
 p.label+' · '+(p.active?'当前接触':'当前未接触'),
 '  N = '+fmt(p.normal_force_N,'N')+'；g = '+(p.gap_m===null?p.gap_status:fmt(p.gap_m,'m')),
 '  当前储能 = '+fmt(p.current_storage_J,'J'),
 '  上一段法向功 = '+fmt(p.last_normal_work_J,'J')+'；摩擦功 = '+fmt(p.last_friction_work_J,'J'),
 '  上一段法向余项 = '+fmt(p.last_normal_remainder_J,'J')].join('\n'))].join('\n');}
return{describe,summaryText,groupText};
});
