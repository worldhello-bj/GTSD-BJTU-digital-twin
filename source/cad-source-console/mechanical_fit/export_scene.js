'use strict';
const fs=require('node:fs'),path=require('node:path'),z=require('node:zlib');
const R=path.resolve(__dirname,'..'),B=R+'/baseline_console/GTSD_Assembly_Console';
const A=require(B+'/app/web/core.js'),W=require(R+'/frame_track/registration.js'),S=require(R+'/integration/web/source-replacements.js'),F=require('./static_profiles.js');
const read=p=>JSON.parse(fs.readFileSync(p));
function protectedNames(){const a=read(R+'/frame_track/registration_addendum.json'),b=read(R+'/frame_track/bogie_registration_addendum.json');return [...a.rows.filter(r=>r.body_name!=='world'),...b.rigid_rows,...b.coil_rows].map(r=>r.name||r.mesh_name);}
// Historical metadata sometimes serializes another metadata object as text.
// Preserve source handles/filenames without exposing producer machine paths.
function publicMetadata(v){
 if(Array.isArray(v))return v.map(publicMetadata);
 if(v&&typeof v==='object')return Object.fromEntries(Object.entries(v).map(([k,x])=>[k,publicMetadata(x)]));
 if(typeof v!=='string')return v;
 try{const p=JSON.parse(v);if(p&&typeof p==='object')return JSON.stringify(publicMetadata(p));}catch{}
 return v.replace(/\/(?:workspace|tmp|home)\/[^\s"'\\]+/g,s=>'source-artifact/'+s.split('/').pop());
}
async function load(){
 const catalog=read(B+'/app/data/catalog.json'),mapping=read(B+'/app/data/mapping_bundle.json'),raw=z.gunzipSync(fs.readFileSync(B+'/assets/whole.glb.gz'));
 const cache=A.decodeGLB(raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength),catalog);
 A.addExtras(cache,JSON.parse(z.gunzipSync(fs.readFileSync(B+'/app/data/scene_extras.json.gz'))),catalog);
 const replace=await S.prepare(cache,catalog,mapping,read(R+'/integration/data/source_replacements.json'),W.witness);
 const available=new Map(cache);for(const m of replace.meshes)available.set(m.name,m);
 const fit=await F.prepare(read(__dirname+'/profiles.json'),catalog,available,W.witness,mapping,replace.revision,protectedNames());
 const hidden=new Set(read(R+'/integration/data/source_overlays.json').display_review_contract.source_review_suppress_exact);
 function resource(scene=catalog.default_scene,profile='source'){
  const base=A.sceneResource(catalog,cache,scene,'all');
  return fit.apply(replace.apply({...base,meshes:base.meshes.filter(m=>!hidden.has(m.name))}),profile);
 }
 return {catalog,mapping,cache,available,replace,fit,resource};
}
async function main(){
 const {catalog,replace,fit,resource}=await load(),out=path.resolve(process.argv[2]||(__dirname+'/build'));
 fs.mkdirSync(out,{recursive:true});const profiles=['source',...fit.profiles.keys()],scenes=[];
 for(const profile of profiles){
  const r=resource(catalog.default_scene,profile),rows=[];
  for(const m of r.meshes)rows.push({name:m.name,vertices:m.vertices,indices:m.indices,color:m.color,group:m.group,geometry_witness_sha256:await W.witness(m),metadata:publicMetadata(m.metadata||catalog.meshes.find(x=>x.name===m.name)?.source_metadata||{}),mechanical_fit_candidate:!!m.mechanical_fit_candidate});
  const data={schema:'gtsd-fit-scene-export-v1',units:'m',source_base_glb_sha256:catalog.base_glb.sha256,source_native_sha256:catalog.native_sha256,source_display_revision_sha256:replace.revision,source_scene:catalog.default_scene,profile,new_physics:false,as_built:false,source_hidden_historical_interpretations:[...new Set(read(R+'/integration/data/source_overlays.json').display_review_contract.source_review_suppress_exact)],meshes:rows};
  // Store one whole source scene and only candidate mesh/replacement data for
  // the other scenes; Blender shares untouched source meshes across scenes.
  if(profile!=='source'){data.mesh_refs=rows.map(m=>({name:m.name,geometry_witness_sha256:m.geometry_witness_sha256}));data.meshes=rows.filter(m=>m.mechanical_fit_candidate);}
  const file=profile+'.json.gz';fs.writeFileSync(path.join(out,file),z.gzipSync(JSON.stringify(data),{level:6}));
  scenes.push({profile,file,mesh_count:r.meshes.length,triangles:r.meshes.reduce((n,m)=>n+m.indices.length/3,0),added:r.mechanical_fit_added||0,removed:r.mechanical_fit_removed||0,incomplete_groups:r.mechanical_fit_incomplete_groups||[]});
 }
 fs.writeFileSync(path.join(out,'scenes.json'),JSON.stringify({scenes},null,2)+'\n');console.log(JSON.stringify(scenes));
}
module.exports={load,publicMetadata};if(require.main===module)main().catch(e=>{console.error(e);process.exitCode=1;});
