'use strict';
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),test=require('node:test'),crypto=require('node:crypto');
const realU=require('../web/lineage.js');
const client=fs.readFileSync(require('node:path').join(__dirname,'../web/lineage-client.js'),'utf8');
const client_sha256=crypto.createHash('sha256').update(client).digest('hex');
const C={schema:'gtsd-semantic-lineage-registry-v1',catalogue_sha256:'catalog',current_geometry_sha256:'geometry',model_identity_sha256:'model'};
const O={schema:'gtsd-semantic-lineage-observations-v1',catalogue_sha256:'catalog',current_geometry_sha256:'geometry',model_identity_sha256:'model',identity_match:true,session_id:'session',steps:1,phase:'paused',sample_time_s:0.1,observations:[{id:'pressure',value:123,available:true}]};
async function setup(){
 const events={},timers=[],mounts=[],observations=[structuredClone(O)],truths=[],catalogues=[];
 const box={textContent:'',scrollTop:0,querySelector:()=>null,append:function(p){this.textContent+='\n'+p.textContent;}};
 let controls;
 const window={GTSDLineage:{...realU,mount:(b,c,o,opts)=>{controls=opts;mounts.push({valid:realU.validate(c,o),value:o.observations[0].value,available:o.observations[0].available,truth:opts.truth});b.textContent=JSON.stringify(mounts.at(-1));return{};}},GTSDLineageView:{visible:()=>['mesh']},addEventListener:(type,fn)=>events[type]=fn};
 const document={getElementById:id=>id==='lineage-panel'?box:null,activeElement:{},createElement:()=>({})};
 const respond=async value=>{if(value instanceof Error)throw value;return{ok:true,json:async()=>value};};
 const fetch=async path=>{if(path==='/api/truth-evidence')return respond(truths.length?truths.shift():{evidence_revision_sha256:'truth',rows:[{id:'part',as_built_1to1_verified:false}]});if(path==='/api/lineage-registry')return respond(catalogues.length?catalogues.shift():structuredClone(C));return respond(observations.shift());};
 vm.runInNewContext(client,{window,document,fetch,setTimeout:fn=>timers.push(fn)});
 const flush=()=>new Promise(r=>setImmediate(r));await flush();
 return{events,timers,mounts,observations,truths,catalogues,box,flush,refreshTruth:()=>controls.onRefresh(),select:()=>events['gtsd-lineage-selection']({detail:{ids:['mesh']}}),poll:async value=>{observations.push(value);await timers.shift()();}};
}
test('failed poll stays unavailable through selection and successful truth refresh; fresh poll recovers',async()=>{
 const h=await setup();await h.poll(new Error('HTTP 409 model unavailable'));assert.match(h.box.textContent,/暂不可用/);const n=h.mounts.length;h.select();await h.refreshTruth();assert.equal(h.mounts.length,n);assert.match(h.box.textContent,/暂不可用/);await h.poll(structuredClone(O));assert.equal(h.mounts.at(-1).valid,true);assert.equal(h.mounts.length,n+1);
});
test('failed catalogue refresh cannot resurrect prior observation via selection',async()=>{
 const h=await setup();h.catalogues.push(new Error('catalog unavailable'));await h.poll({...structuredClone(O),catalogue_sha256:'new'});const n=h.mounts.length;h.select();await h.refreshTruth();assert.equal(h.mounts.length,n);assert.match(h.box.textContent,/暂不可用/);
});
test('identity flag and model/geometry hash changes invalidate a same-step frame',async()=>{
 for(const patch of [{identity_match:false},{model_identity_sha256:'wrong'},{current_geometry_sha256:'wrong'}]){const h=await setup();await h.poll({...structuredClone(O),...patch});assert.equal(h.mounts.length,2);assert.equal(h.mounts.at(-1).valid,false);}
});
test('same-step value and availability changes re-render',async()=>{
 const h=await setup();await h.poll({...structuredClone(O),observations:[{id:'pressure',value:999,available:true}]});assert.equal(h.mounts.at(-1).value,999);await h.poll({...structuredClone(O),observations:[{id:'pressure',value:null,available:false}]});assert.equal(h.mounts.at(-1).available,false);assert.equal(h.mounts.length,3);
});
test('failed truth refresh clears independent evidence and selection cannot revive it',async()=>{
 const h=await setup();assert.ok(h.mounts.at(-1).truth);h.truths.push(new Error('truth unavailable'));await h.refreshTruth();assert.equal(h.mounts.at(-1).truth,null);assert.match(h.box.textContent,/只读证据刷新失败/);h.select();assert.equal(h.mounts.at(-1).truth,null);await h.refreshTruth();assert.ok(h.mounts.at(-1).truth);
});
test('snapshot-age-only updates do not force a mount',async()=>{
 const h=await setup();await h.poll({...structuredClone(O),snapshot_age_s:3});assert.equal(h.mounts.length,1);
});
test('record exact client hash and test scope',()=>console.log(JSON.stringify({client_sha256,scope:'Node VM state-machine tests with actual client and real validate; instrumented DOM, no browser/GPU acceptance'})));
