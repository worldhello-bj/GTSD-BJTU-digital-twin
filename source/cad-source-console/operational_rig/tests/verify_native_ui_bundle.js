'use strict';
// Read-only acceptance of captured native data in the shipped HTML/DOM.
// Renderer and transport are doubles. This is not browser visual acceptance.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),crypto=require('node:crypto'),{JSDOM}=require('jsdom'),C=require('../web/core.js');
const folder=process.argv[2];
if(!folder)throw Error('Usage: node tests/verify_native_ui_bundle.js /path/to/native-ui-bundle');
const hash=bytes=>crypto.createHash('sha256').update(bytes).digest('hex');
const raw=Object.fromEntries(['metadata','geometry','status'].map(k=>[k,fs.readFileSync(path.join(folder,k+'.json'))]));
const bundle=Object.fromEntries(Object.entries(raw).map(([k,v])=>[k,JSON.parse(v)]));
const original=hash(JSON.stringify(bundle.geometry));
const result=C.resource(bundle.geometry,bundle.metadata,bundle.status);
assert.equal(result.counts.bound,bundle.metadata.contract.bindings.length);
assert(result.resource.meshes.every(m=>m.vertices.every(Number.isFinite)));
assert.equal(hash(JSON.stringify(bundle.geometry)),original);
const views={};
for(const view of ['all','wheel_rail','catenary','pin_50DF4']){
 const r=C.resource(bundle.geometry,bundle.metadata,bundle.status,view);
 views[view]={mesh_count:r.resource.meshes.length,focus_bounds:C.focusBounds(r.resource.meshes,view,bundle.metadata.contract.parameters)};
}
let posts=0;
const doms=[];
function boot(page){
 const dom=new JSDOM(fs.readFileSync(path.join(__dirname,'../web',page+'.html'),'utf8'),{url:'http://127.0.0.1:8176/operational-rig',runScripts:'outside-only'}),w=dom.window;
 doms.push(dom);
 w.fetch=async(url,opts={})=>{if(opts.method==='POST'){posts++;throw Error('Native bundle acceptance is read-only');}const key=url.replace('/api/operational-rig/','');return{ok:true,json:async()=>bundle[key]};};
 w.AbortSignal.timeout=()=>new w.AbortController().signal;
 class ModelView{constructor(){this.yaw=0;this.pitch=0;this.zoom=1;}setResource(r){this.resource=r;}setView(){}select(){}requestDraw(){}focusSelection(){}resetCamera(){}snapshot(){return{visibleMeshes:this.resource.meshes.length};}}
 w.GTSDReconstruction={ModelView};
 w.eval(fs.readFileSync(path.join(__dirname,'../web/core.js'),'utf8'));
 w.eval(fs.readFileSync(path.join(__dirname,'../web',page==='index'?'app.js':'parameters.js'),'utf8'));
 return w;
}
const flush=async()=>{for(let i=0;i<12;i++)await new Promise(r=>setImmediate(r));};
(async()=>{
 const w=boot('index');await flush();const e=id=>w.document.getElementById(id);
 assert.equal(e('error').textContent,'');
 assert.equal(e('branch').value,'wheel_rail');
 assert.equal(e('displacement').textContent,C.format(C.telemetry(bundle.status).displacement));
 assert.equal(e('contact-force').textContent,C.format(C.telemetry(bundle.status).contactForce));
 assert.equal(e('energy').textContent,C.format(bundle.status.energy.total_J,4));
 assert.equal(e('step').disabled,false);
 if(bundle.status.performance){
  assert(e('run-detail').textContent.includes('墙钟 '+C.format(bundle.status.performance.elapsed_wall_s,1)+' s'));
  assert(e('run-detail').textContent.includes(C.format(bundle.status.performance.simulated_s_per_wall_s,4)+'×实时'));
 }
 const accounting=bundle.metadata.contract.mass_accounting||{};
 for(const[key,label]of [['dynamic_source_solids','动力实体'],['fixed_context_meshes','固定参照'],['visual_overlay_meshes','同件显示层']])if(Number.isInteger(accounting[key]))assert(e('binding-status').textContent.includes(`${accounting[key]} ${label}`));
 assert(e('part-facts').textContent.includes('刚体组合质量'));
 assert.equal([...e('part-facts').querySelectorAll('dt')].some(x=>x.textContent==='质量'),false);
 for(const[id,preset]of [['view-wheel','wheel_rail'],['view-wire','catenary'],['view-pin','pin_50DF4'],['view-all','all']]){
  e(id).click();assert.equal(e('error').textContent,'');assert.equal(e('branch').value,preset);
  assert.equal(w.GTSDOperationalRigDebug.renderer().visibleMeshes,views[preset].mesh_count);
 }
 const p=boot('parameters');await flush();
 assert.equal(p.document.getElementById('parameters-error').textContent,'');
 assert.equal(p.document.getElementById('save-parameters').disabled,false);
 const sources=new Map(bundle.metadata.parameter_sources.map(row=>[row.key,row])),parameterCoverage=[];
 for(const key of Object.keys(bundle.metadata.defaults)){
  const input=p.document.getElementById('parameter-'+key),source=sources.get(key);
  assert(input,'Missing visible default parameter: '+key);assert(source,'Missing source row: '+key);assert(source.unit,'Missing unit: '+key);
  const row=input.closest('.parameter-row');
  assert(row.querySelector('.parameter-key').textContent.includes(source.unit),'Missing visible unit: '+key);
  assert(row.querySelector('.parameter-source').textContent.includes(source.level),'Missing visible provenance level: '+key);
  assert(row.querySelector('.parameter-source').textContent.includes(source.source||source.description),'Missing visible source: '+key);
  if(key==='mode')assert.equal(input.readOnly,true);
  parameterCoverage.push({key,unit:source.unit,source_level:source.level,editable:key!=='mode'});
 }
 const budget=p.document.getElementById('run-budget').textContent;
 for(const key of ['demo_steps_max','run_steps_max','total_steps_max'])assert(budget.includes(bundle.metadata.limits[key].toLocaleString('en-US')));
 assert.equal(posts,0);
 const transportPath=path.join(folder,'report.json'),transport=fs.existsSync(transportPath)?JSON.parse(fs.readFileSync(transportPath,'utf8')):null;
 if(transport){assert.equal(transport.passed,true);assert.equal(transport.identity_sha256,bundle.status.identity_sha256);}
 const now=new Date();
 const report={
  schema:'gtsd-operational-native-ui-contract-check-v1',completed_at_utc:now.toISOString(),completed_at_beijing:new Date(now.getTime()+8*60*60*1000).toISOString().replace('Z','+08:00'),result:'pass',
  scope:transport?'Captured native HTTP responses verified in jsdom with renderer/transport doubles; browser graphics and downloads are not exercised.':'Captured native data verified in jsdom with renderer/transport doubles; HTTP and browser graphics are not exercised.',
  checks:{source_hashes_and_all_native_bindings:true,all_transformed_vertices_finite:true,source_geometry_unchanged:true,actual_telemetry_display:true,four_view_shortcuts:true,visible_source_mass_roles:true,grouped_mass_labeled_explicitly:true,parameter_default_source_and_unit_coverage:true,distinct_demo_manual_and_session_budgets:true,read_only_dom_verification:true},
  http_evidence:transport?{schema:transport.schema,passed:transport.passed,passed_check_count:Object.values(transport.checks).filter(Boolean).length,completed_at_utc:transport.finished_utc,report_sha256:hash(fs.readFileSync(transportPath))}:null,
  native_steps:bundle.status.steps,native_time_s:bundle.status.time_s,identity_sha256:bundle.status.identity_sha256,
  source_mass_accounting:accounting,bound_meshes:result.counts.bound,total_meshes:result.resource.meshes.length,post_count:posts,
  telemetry:C.telemetry(bundle.status),display:{displacement_mm:e('displacement').textContent,contact_force_N:e('contact-force').textContent,energy_J:e('energy').textContent},
  budgets:bundle.metadata.limits,budget_display:budget,parameter_coverage:parameterCoverage,views,
  bundle_sha256:Object.fromEntries(Object.entries(raw).map(([key,bytes])=>[key+'.json',hash(bytes)])),
  web_sha256:Object.fromEntries(['index.html','app.js','core.js','style.css','parameters.html','parameters.js'].map(name=>['web/'+name,hash(fs.readFileSync(path.join(__dirname,'../web',name)))])),
  browser_visual_tested:false,browser_download_completion_tested:false
 };
 console.log(JSON.stringify(report,null,2));
})().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>doms.forEach(dom=>dom.window.close()));
