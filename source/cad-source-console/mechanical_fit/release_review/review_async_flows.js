'use strict';
// Read-only behavioral review. Pass the console root as the first argument.
const fs=require('node:fs'),path=require('node:path'),Module=require('node:module'),assert=require('node:assert/strict');
const root=path.resolve(process.argv[2]),harnessFile=path.join(root,'integration/tests/app_harness.js');
let source=fs.readFileSync(harnessFile,'utf8');
source=source.replace("if(url==='/api/mechanical-fit/profiles')return response(options.fitData||{});","if(url==='/api/mechanical-fit/profiles'){if(options.fitResponse)return options.fitResponse(opts);return response(options.fitData||{});}");
const mod=new Module(harnessFile);mod.filename=harnessFile;mod.paths=Module._nodeModulePaths(path.dirname(harnessFile));mod._compile(source,harnessFile);
const {boot,fresh}=mod.exports;
const mapping={schema:'gtsd-assembly-console-mapping-v1',contract:{accepted_identity:{model_identity_sha256:'solver',source_geometry_sha256:'source',variant:'strict'},latest_assembly_identity:{native_sha256:'native',default_glb_uncompressed_sha256:'glb'},reference_poses:{world:{position:[0,0,0],quaternion:[1,0,0,0]}}},meshes:['part','alternative'].map(name=>({name,category:'STATIC_WORLD_RIGID_DISPLAY_ONLY',old_binding_body:'world',old_binding_mode:'rigid',may_follow_old_native_body_pose:true,world_triangle_float32_equal:true,oriented_triangle_float32_equal:true,new_collision_allowed:false}))};
const fitData={schema:'gtsd-mechanical-fit-profiles-v1',units:'m',source_display_revision_sha256:'fixture-revision',base_native_sha256:'native',base_glb_sha256:'glb',solver_mapping_unchanged:true,profiles:[{id:'clamps',title:'test candidate',supported_scene_ids:['one'],as_built:false,requires_explicit_selection:true,groups:[{id:'g',remove:[{name:'part',geometry_witness_sha256:'valid',physics_binding_changed:false}],add_meshes:[{name:'fit_mesh',id:'fit_mesh',vertices:[0,0,0,1,0,0,0,1,0],indices:[0,1,2],group:'body',color:[1,.5,0,1],geometry_witness_sha256:'valid',mechanical_fit_candidate:true,as_built:false,physics_binding:false,collision_enabled:false,added_mass_kg:0,body_name:null}]}]}]};
const response=x=>({ok:true,status:200,json:async()=>structuredClone(x)});
(async()=>{
 const checks=[];
 let resolve;
 let h=await boot(fresh,{mapping,fitResponse:()=>new Promise(r=>resolve=r)});
 h.e('fit-profile').value='clamps';const pending=h.e('fit-profile').onchange();assert.equal(h.e('fit-profile').disabled,true);
 h.e('scene').value='two';h.e('scene').onchange();resolve(response(fitData));await pending;
 assert.equal(h.window.GTSDAssemblyDebug.status().mechanical_fit_profile,'source');assert.equal(h.window.GTSDAssemblyDebug.view().resource.assembly_scene,'two');assert.equal(h.window.GTSDAssemblyDebug.view().resource.meshes[0].name,'alternative');assert.equal(h.e('fit-profile').disabled,false);assert.match(h.e('error').textContent,/未核验/);checks.push('Scene changed during first profile load: unsupported candidate rejected; current scene retained.');
 h.e('scene').value='one';h.e('scene').onchange();h.e('fit-profile').value='clamps';await h.e('fit-profile').onchange();assert.equal(h.window.GTSDAssemblyDebug.view().resource.meshes[0].name,'fit_mesh');h.e('scene').value='two';h.e('scene').onchange();assert.equal(h.e('fit-profile').value,'source');assert.equal(h.window.GTSDAssemblyDebug.view().resource.meshes[0].name,'alternative');checks.push('Previously loaded clamp candidate resets to source on unsupported scene change.');
 assert.equal(h.calls.filter(x=>x.opts?.method==='POST').length,0);checks.push('Both transitions issued zero solver POST commands.');
 console.log(JSON.stringify({ok:true,checks,browser_visual_acceptance:false},null,2));
})().catch(e=>{console.error(e);process.exitCode=1;});
