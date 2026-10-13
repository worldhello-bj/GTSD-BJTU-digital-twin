/* Explicit mechanical-fit design alternatives. Never alter solver state or mass. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GTSDMechanicalFit=api;})(globalThis,function(){'use strict';
async function prepare(data,catalog,available,witness,mapping,sourceRevision,protectedDynamicNames=[]){
 if(!Array.isArray(protectedDynamicNames)||!protectedDynamicNames.every(n=>typeof n==='string'))throw Error('Invalid reviewed dynamic protection set');
 const protectedDynamic=new Set(protectedDynamicNames);
 if(data?.schema!=='gtsd-mechanical-fit-profiles-v1'||data.units!=='m'||data.base_native_sha256!==catalog.native_sha256||data.base_glb_sha256!==catalog.base_glb.sha256||data.source_display_revision_sha256!==sourceRevision||data.solver_mapping_unchanged!==true||!Array.isArray(data.profiles))throw Error('Mechanical fit source identity mismatch');
 const sourceRows=new Map((mapping?.meshes||[]).map(m=>[m.name,m]));
 const profiles=new Map();
 for(const p of data.profiles){
  if(typeof p.id!=='string'||!/^[a-z0-9_]+$/.test(p.id)||profiles.has(p.id)||p.as_built!==false||p.requires_explicit_selection!==true||!Array.isArray(p.groups))throw Error('Invalid explicit fit profile');
  const names=new Set(),added=new Set(),groups=[];
  for(const g of p.groups){
   if(!Array.isArray(g.remove)||!g.remove.length||!Array.isArray(g.add_meshes)||!g.add_meshes.length)throw Error('Incomplete fit group');
   for(const row of g.remove){const m=available.get(row.name);if(names.has(row.name)||!m||row.physics_binding_changed!==false||await witness(m)!==row.geometry_witness_sha256)throw Error('Fit source geometry mismatch '+row.name);const old=sourceRows.get(row.name),staticReplacement=m.static_source_replacement===true&&m.physics_binding===false;const allowed=staticReplacement||old&&(old.category==='UNBOUND_SOURCE_OR_CANDIDATE'||old.category==='STATIC_WORLD_RIGID_DISPLAY_ONLY'&&old.old_binding_body==='world'||old.category==='PRESERVE_WORLD_STATIC_MAP'&&old.old_binding_body==='world');if(!allowed||protectedDynamic.has(row.name))throw Error('Fit cannot replace native or reviewed dynamic geometry '+row.name);names.add(row.name);}
   const meshes=[];
   for(const m of g.add_meshes){
    if(m.id!==m.name||available.has(m.name)||added.has(m.name)||m.mechanical_fit_candidate!==true||m.as_built!==false||m.physics_binding!==false||m.collision_enabled!==false||m.added_mass_kg!==0||m.body_name!==null||!Array.isArray(m.vertices)||!m.vertices.length||m.vertices.length%3||!m.vertices.every(Number.isFinite)||!Array.isArray(m.indices)||!m.indices.length||m.indices.length%3||!m.indices.every(i=>Number.isInteger(i)&&i>=0&&i<m.vertices.length/3)||await witness(m)!==m.geometry_witness_sha256)throw Error('Invalid fit candidate '+m.name);
    added.add(m.name);meshes.push(Object.freeze({...m,vertices:Object.freeze([...m.vertices]),indices:Object.freeze([...m.indices])}));
   }
   groups.push(Object.freeze({id:g.id,remove:Object.freeze(g.remove.map(x=>x.name)),meshes:Object.freeze(meshes)}));
  }
  profiles.set(p.id,Object.freeze({id:p.id,title:p.title,evidence:p.evidence,supported_scene_ids:Object.freeze([...(p.supported_scene_ids||Object.keys(catalog.scenes||{}))]),groups:Object.freeze(groups),meshes:Object.freeze(groups.flatMap(g=>g.meshes))}));
 }
 function apply(resource,id){
  if(!id||id==='source')return resource;
  const p=profiles.get(id);if(!p)throw Error('Unknown fit profile');
  if(p.supported_scene_ids.length&&!p.supported_scene_ids.includes(resource.assembly_scene))throw Error('Fit profile is not reviewed for this source scene');
  if(resource.mechanical_fit_profile)throw Error('Fit profile cannot be applied twice');
  const present=new Set(resource.meshes.map(m=>m.name)),remove=new Set(),add=[],skipped=[];
  for(const g of p.groups){const n=g.remove.filter(x=>present.has(x)).length;if(!n)continue;if(n!==g.remove.length){skipped.push(g.id);continue;}g.remove.forEach(x=>remove.add(x));add.push(...g.meshes);}
  return {...resource,meshes:[...resource.meshes.filter(m=>!remove.has(m.name)),...add],mechanical_fit_profile:id,mechanical_fit_added:add.length,mechanical_fit_removed:remove.size,mechanical_fit_removed_names:[...remove],mechanical_fit_incomplete_groups:skipped,mechanical_fit_as_built:false,mechanical_fit_solver_modified:false};
 }
 return Object.freeze({profiles,apply});
}
function merge(base,extra){
 for(const k of ['schema','units','base_native_sha256','base_glb_sha256','source_display_revision_sha256','solver_mapping_unchanged'])if(base?.[k]!==extra?.[k])throw Error('Additional profile source identity mismatch '+k);
 if(!Array.isArray(base.profiles)||!Array.isArray(extra.profiles))throw Error('Additional profile array required');
 const ids=new Set(base.profiles.map(p=>p.id));
 for(const p of extra.profiles){if(ids.has(p.id)||!p.id.startsWith('photo_frog_'))throw Error('Unexpected additional photo profile');ids.add(p.id);}
 return {...base,profiles:[...base.profiles,...extra.profiles]};
}
return {prepare,merge};
});
