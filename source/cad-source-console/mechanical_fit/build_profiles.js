/* Normalize the audited component alternatives against the exact public model. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),z=require('node:zlib'),crypto=require('node:crypto');
const R=path.resolve(__dirname,'..'),B=R+'/baseline_console/GTSD_Assembly_Console';
const A=require(B+'/app/web/core.js'),W=require(R+'/frame_track/registration.js'),S=require(R+'/integration/web/source-replacements.js'),F=require('./static_profiles.js');
const read=p=>JSON.parse(fs.readFileSync(p));
function protectedNames(){const a=read(R+'/frame_track/registration_addendum.json'),b=read(R+'/frame_track/bogie_registration_addendum.json');return [...a.rows.filter(r=>r.body_name!=='world'),...b.rigid_rows,...b.coil_rows].map(r=>r.name||r.mesh_name);}
function scrub(v){
 if(typeof v==='string')return v.replace(/\/workspace\/[^\s"']+/g,'LOCAL_STAGING_PATH_OMITTED').replace(/\/tmp\/[^\s"']+/g,'LOCAL_STAGING_PATH_OMITTED');
 if(Array.isArray(v))return v.map(scrub);
 if(v&&typeof v==='object')return Object.fromEntries(Object.entries(v).map(([k,x])=>[k,scrub(x)]));
 return v;
}
async function main(){
 const c=read(B+'/app/data/catalog.json'),mapping=read(B+'/app/data/mapping_bundle.json'),raw=z.gunzipSync(fs.readFileSync(B+'/assets/whole.glb.gz')),cache=A.decodeGLB(raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength),c);
 A.addExtras(cache,JSON.parse(z.gunzipSync(fs.readFileSync(B+'/app/data/scene_extras.json.gz'))),c);
 const source=read(R+'/integration/data/source_replacements.json'),replace=await S.prepare(cache,c,mapping,source,W.witness),available=new Map(cache);
 for(const m of replace.meshes)available.set(m.name,m);
 const specs=[['track_support','track_support/profile_conforming_hull.json'],['track_support_flat','track_support/profile_flat_top_blank.json'],['balise','balise/balise_fit_profile.json'],['clamps','clamps/clamp_fit_profile.json'],['clamps_messenger','clamps/clamp_messenger_datum_profile.json'],['clamps_b535_review','clamps/clamp_b535_review_profile.json']],profiles=[];
 for(const [id,file]of specs){
  const input=path.join(__dirname,'inputs',file),bytes=fs.readFileSync(input),p=read(input),remove=[];
  for(const name of p.remove_names){const m=available.get(name);if(!m)throw Error('No source geometry '+name);remove.push({name,geometry_witness_sha256:await W.witness(m),physics_binding_changed:false});}
  const meshes=[];
  for(const row of p.add_meshes){const m={...scrub(row),id:row.name,mechanical_fit_candidate:true,as_built:false,physics_binding:false,collision_enabled:false,added_mass_kg:0,body_name:null};m.geometry_witness_sha256=await W.witness(m);meshes.push(m);}
  profiles.push({id,title:p.label||p.label_zh||id,requires_explicit_selection:true,as_built:false,supported_scene_ids:id.startsWith('clamps')?[c.default_scene]:Object.keys(c.scenes),evidence:{input:file,input_sha256:crypto.createHash('sha256').update(bytes).digest('hex'),configuration:scrub(p.configuration||p.config||{}),source_inference_not_hardware_certification:true},groups:[{id,remove,add_meshes:meshes}]});
 }
 profiles.push({id:'combined',title:'全部已核机械配合工程候选',requires_explicit_selection:true,as_built:false,supported_scene_ids:[c.default_scene],evidence:{combines:['track_support','balise','clamps'],independent_assembly_validation_required:true},groups:profiles.filter(p=>['track_support','balise','clamps'].includes(p.id)).flatMap(p=>p.groups)});
 const data={schema:'gtsd-mechanical-fit-profiles-v1',units:'m',base_native_sha256:c.native_sha256,base_glb_sha256:c.base_glb.sha256,source_display_revision_sha256:replace.revision,solver_mapping_unchanged:true,profiles};
 const fit=await F.prepare(data,c,available,W.witness,mapping,replace.revision,protectedNames()),hidden=new Set(read(R+'/integration/data/source_overlays.json').display_review_contract.source_review_suppress_exact),checks=[];
 for(const p of profiles)for(const scene of p.supported_scene_ids){const b=A.sceneResource(c,cache,scene,'all'),r=fit.apply(replace.apply({...b,meshes:b.meshes.filter(m=>!hidden.has(m.name))}),p.id);if(r.mechanical_fit_incomplete_groups.length)throw Error('Incomplete groups '+p.id);checks.push({profile:p.id,scene,mesh_count:r.meshes.length,removed:r.mechanical_fit_removed,added:r.mechanical_fit_added});}
 fs.writeFileSync(__dirname+'/profiles.json',JSON.stringify(data)+'\n');fs.writeFileSync(__dirname+'/evidence/profile_identity_validation.json',JSON.stringify({ok:true,checks,solver_mapping_unchanged:true},null,2)+'\n');console.log(JSON.stringify(checks));
}
module.exports={main,scrub};if(require.main===module)main().catch(e=>{console.error(e);process.exitCode=1;});
