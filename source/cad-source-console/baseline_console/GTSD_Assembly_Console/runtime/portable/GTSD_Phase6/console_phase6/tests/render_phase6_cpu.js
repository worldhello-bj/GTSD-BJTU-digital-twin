/* Actual source bytes plus saved accepted solver poses. CPU evidence, not browser QA. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),zlib=require('node:zlib');
const root=path.resolve(__dirname,'..'),sourceRoot=path.resolve(process.argv[2]||path.join(root,'../gtsd-physics-phase5-1012'));
const cadMode=process.argv[3]==='cad',prefix=cadMode?'cad_body':'phase6';
const U=require(path.join(sourceRoot,'console/web/ui-model.js')),P=require(path.join(sourceRoot,'web/physics-model.js')),G=require(path.join(sourceRoot,'web/render-model.js'));
const bytes=cadMode?fs.readFileSync(0):fs.readFileSync(path.join(sourceRoot,'web/geometry.json')),hash=crypto.createHash('sha256').update(bytes).digest('hex'),source=JSON.parse(bytes);
const state=JSON.parse(fs.readFileSync(path.join(root,'evidence/'+prefix+'_accepted_state.json'))),binding=JSON.parse(fs.readFileSync(path.join(root,'evidence/'+prefix+'_accepted_binding.json')));
const originalState=JSON.stringify(state),originalSource=JSON.stringify(source),mapping=U.liveResource(source,binding,state,hash,P),proxies=U.proxyMeshes(state.plant.solver_proxies);
if(proxies.length!==21)throw new Error('Expected 17 existing + 4 explicit equivalent-inertia research proxies');
const aux=state.plant.solver_proxies.filter(p=>p.geom_name?.startsWith('phase6-display-only:'));
if(aux.length!==4||aux.some(p=>p.source_mapping_confirmed!==false||p.collision_geometry!==false))throw new Error('Auxiliary display provenance is not explicit');
let pixels;
const ctx={createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),putImageData:img=>{pixels=img;},fillRect(){},fillText(){},clearRect(){},setTransform(){}};
const canvas={style:{},width:800,height:600,clientWidth:800,clientHeight:600,getContext:name=>name==='2d'?ctx:null,addEventListener(){},removeEventListener(){},focus(){}};
const view=new G.ModelView(canvas,{forceCanvas:true});view.setResource({...mapping.resource,meshes:[...mapping.resource.meshes,...proxies]});view.setView('cutaway');view.frameBounds=G.boundsOf(view.resource.meshes.filter(m=>!['environment','display'].includes(m.group)));view.yaw=.72;view.pitch=.35;view.draw();
if(!pixels||view.unavailable||view.softwareStats.blocked)throw new Error('CPU raster unavailable');
let changed=0;for(let i=0;i<pixels.data.length;i+=4)if(pixels.data[i]!==11||pixels.data[i+1]!==17||pixels.data[i+2]!==24)changed++;
if(changed<1000)throw new Error('Raster has no credible visible geometry');
if(JSON.stringify(state)!==originalState||JSON.stringify(source)!==originalSource)throw new Error('Read-only rendering mutated its inputs');
function crc32(data){let c=0xffffffff;for(const b of data){c^=b;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;}return(c^0xffffffff)>>>0;}
function chunk(type,data){const t=Buffer.from(type),length=Buffer.alloc(4),crc=Buffer.alloc(4);length.writeUInt32BE(data.length);crc.writeUInt32BE(crc32(Buffer.concat([t,data])));return Buffer.concat([length,t,data,crc]);}
const ihdr=Buffer.alloc(13);ihdr.writeUInt32BE(canvas.width,0);ihdr.writeUInt32BE(canvas.height,4);ihdr[8]=8;ihdr[9]=6;
const raw=Buffer.alloc(canvas.height*(1+canvas.width*4));for(let y=0;y<canvas.height;y++)Buffer.from(pixels.data.buffer,pixels.data.byteOffset+y*canvas.width*4,canvas.width*4).copy(raw,y*(canvas.width*4+1)+1);
fs.writeFileSync(path.join(root,'evidence/'+prefix+'_actual_pose_cpu.png'),Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]),chunk('IHDR',ihdr),chunk('IDAT',zlib.deflateSync(raw)),chunk('IEND',Buffer.alloc(0))]));
const evidence={ok:true,kind:'CPU depth raster of accepted source-bound 79DOF observation; not browser/GPU acceptance',simulation_time_s:state.plant.time_s,steps:state.plant.steps,model_identity_sha256:state.plant.model_identity_sha256,source_geometry_sha256:hash,changed_pixels:changed,solver_proxy_count:21,auxiliary_equivalent_inertia_display_proxies:4,source_or_checkpoint_mutated:false,explicit_CAD_body_partial_migration:cadMode,encoder_or_pantograph_candidates_bound:false,mapping:mapping.summary,renderer:view.snapshot()};
fs.writeFileSync(path.join(root,'evidence/'+prefix+'_cpu_render.json'),JSON.stringify(evidence,null,2));process.stdout.write(JSON.stringify(evidence,null,2)+'\n');
