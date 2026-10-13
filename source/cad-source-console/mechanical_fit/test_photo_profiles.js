'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {load}=require('./export_scene.js'),F=require('./static_profiles.js'),W=require('../frame_track/registration.js');
const root=path.resolve(__dirname,'..'),read=p=>JSON.parse(fs.readFileSync(path.join(root,p)));
test('photo profiles replace whole source groups without altering legacy data or solver bindings',async()=>{
 const env=await load(),base=read('mechanical_fit/profiles.json'),original=JSON.stringify(base);
 const extra=read('photo_turnout/photo_source_profiles.json'),a=read('frame_track/registration_addendum.json'),b=read('frame_track/bogie_registration_addendum.json');
 const protectedNames=[...a.rows.filter(r=>r.body_name!=='world'),...b.rigid_rows,...b.coil_rows].map(r=>r.name||r.mesh_name);
 const fit=await F.prepare(F.merge(base,extra),env.catalog,env.available,W.witness,env.mapping,env.replace.revision,protectedNames);
 const before=env.resource(),byName=new Map(before.meshes.map(m=>[m.name,m]));
 for(const [id,count,removed,added] of [['photo_frog_proxy_review',1931,47,46],['photo_frog_hold_down_research',2006,67,141]]){
  const after=fit.apply(before,id);assert.equal(after.meshes.length,count);assert.equal(after.mechanical_fit_removed,removed);assert.equal(after.mechanical_fit_added,added);assert.deepEqual(after.mechanical_fit_incomplete_groups,[]);assert.equal(after.mechanical_fit_solver_modified,false);assert.equal(new Set(after.meshes.map(m=>m.name)).size,count);
  for(const m of after.meshes){if(!m.mechanical_fit_candidate)assert.equal(m,byName.get(m.name));else{assert.equal(m.physics_binding,false);assert.equal(m.collision_enabled,false);assert.equal(m.added_mass_kg,0);}}
 }
 assert.equal(JSON.stringify(base),original);assert.equal(env.resource(undefined,'combined').meshes.length,1928);
});
