'use strict';
// Compose independent source-based profiles. Historical profiles remain unchanged.
const fs=require('node:fs'),path=require('node:path'),z=require('node:zlib');
const args=process.argv.slice(2),at=args.indexOf('--source');
if(at<0||!args[at+1])throw Error('Usage: node build_source_profiles.js --source CONSOLE_ROOT');
const source=path.resolve(args[at+1]),root=__dirname;
const W=require(path.join(source,'frame_track/registration.js'));
const prior=JSON.parse(fs.readFileSync(path.join(source,'mechanical_fit/profiles.json')));
const base=prior.profiles.find(p=>p.id==='combined');if(!base)throw Error('Expected PR5 combined profile');
async function profile(folder,id,title){
 const manifest=JSON.parse(fs.readFileSync(path.join(root,folder,'change_manifest.json')));
 const payload=JSON.parse(z.gunzipSync(fs.readFileSync(path.join(root,folder,'candidate_meshes.json.gz'))));
 const remove=new Map(base.groups.flatMap(g=>g.remove).map(r=>[r.name,r]));
 const add=new Map(base.groups.flatMap(g=>g.add_meshes).map(m=>[m.name,m]));
 for(const r of manifest.remove){
  if(add.has(r.name)){
   if(await W.witness(add.get(r.name))!==r.expected_geometry_witness_sha256)throw Error('Combined candidate identity mismatch '+r.name);
   add.delete(r.name);
  }else{
   if(remove.has(r.name))throw Error('Already absent from combined profile '+r.name);
   remove.set(r.name,{name:r.name,geometry_witness_sha256:r.expected_geometry_witness_sha256,physics_binding_changed:false});
  }
 }
 for(const m0 of payload.meshes){
  const m={...m0,id:m0.name,mechanical_fit_candidate:true,as_built:false,physics_binding:false,collision_enabled:false,added_mass_kg:0,body_name:null};
  m.geometry_witness_sha256=await W.witness(m);m.metadata={...m.metadata,as_built_confirmed:false,physics_binding:false,collision_enabled:false};
  if(add.has(m.name))throw Error('Duplicate add '+m.name);add.set(m.name,m);
 }
 return {id,title,as_built:false,requires_explicit_selection:true,supported_scene_ids:base.supported_scene_ids,evidence:{status:'STATIC_RESEARCH_NOT_AS_BUILT_OR_PHYSICS_CERTIFIED',base_profile:'PR5 combined, flattened to original source removals',manifest:folder+'/change_manifest.json',material_audit:folder+'/material_audit.json',historical_profiles_unchanged:true},groups:[{id:id+'_complete_source_replacement',remove:[...remove.values()],add_meshes:[...add.values()]}]};
}
(async()=>{
 const profiles=[await profile('build','photo_frog_proxy_review','照片支持的辙叉与分层支承 · 保留旧扣件代理审查'),await profile('hardware_revision','photo_frog_hold_down_research','照片支持的辙叉与完整外置压板扣件 · 研究候选')];
 function clean(v){if(Array.isArray(v))return v.map(clean);if(v&&typeof v==='object')return Object.fromEntries(Object.entries(v).map(([k,x])=>[k,clean(x)]));if(typeof v==='string')return v.replace(/[^\s\"'<>;,，;]+\.(jpg|jpeg|png|heic|webp)\b/gi,'private photo observation');return v;}
 const out=clean({...prior,profiles});fs.writeFileSync(path.join(root,'photo_source_profiles.json'),JSON.stringify(out)+'\n');
 fs.writeFileSync(path.join(root,'profile_mapping.json'),JSON.stringify({schema:'gtsd-photo-source-profile-mapping-v1',base_native_sha256:prior.base_native_sha256,base_glb_sha256:prior.base_glb_sha256,source_display_revision_sha256:prior.source_display_revision_sha256,integration:'Append these two profiles to existing profiles array. Do not apply on top of combined or mutate old profiles. Both are already flattened source-based alternatives.',profiles:profiles.map(p=>({id:p.id,remove:p.groups[0].remove,add_ids:p.groups[0].add_meshes.map(m=>m.name)}))},null,2)+'\n');
 console.log(profiles.map(p=>({id:p.id,source_remove:p.groups[0].remove.length,source_add:p.groups[0].add_meshes.length})));
})().catch(e=>{console.error(e);process.exitCode=1;});
