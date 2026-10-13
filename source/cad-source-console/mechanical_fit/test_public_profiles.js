'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {load}=require('./export_scene.js');
test('real public mesh identities admit every declared profile and preserve unrelated geometry',async()=>{
 const env=await load();let checks=0;
 for(const p of env.fit.profiles.values())for(const scene of p.supported_scene_ids){
  const before=env.resource(scene),after=env.resource(scene,p.id),old=new Map(before.meshes.map(m=>[m.name,m]));
  assert.equal(after.mechanical_fit_incomplete_groups.length,0);
  assert.equal(after.meshes.length,before.meshes.length-after.mechanical_fit_removed+after.mechanical_fit_added);
  assert.equal(new Set(after.meshes.map(m=>m.name)).size,after.meshes.length);
  for(const m of after.meshes)if(!m.mechanical_fit_candidate)assert.equal(m,old.get(m.name));
  for(const m of after.meshes.filter(m=>m.mechanical_fit_candidate)){assert.equal(m.collision_enabled,false);assert.equal(m.added_mass_kg,0);assert.equal(m.as_built,false);}
  checks++;
 }
 assert.equal(checks,19);
 const source=env.resource(),combined=env.resource(undefined,'combined');
 assert.equal(source.meshes.length,1932);assert.equal(combined.meshes.length,1928);
 assert.equal(combined.mechanical_fit_removed,37);assert.equal(combined.mechanical_fit_added,33);
 assert.equal(env.resource().meshes.length,1932);
});
