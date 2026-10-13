'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
test('finite rail review keeps every source curve when highlighting either route',()=>{
 const html=fs.readFileSync(__dirname+'/rail_web/index.html','utf8'),code=fs.readFileSync(__dirname+'/rail_web/app.js','utf8');
 const handles=[...html.matchAll(/data-handle="([^"]+)"/g)].map(m=>m[1]);assert.equal(handles.length,57);assert.equal(new Set(handles).size,57);
 assert.equal((html.match(/class="band"/g)||[]).length,5);
 assert(!/\son(?:click|change)=/.test(html));assert(!/<script>/.test(html));
 const edges=['main','branch','detail','other'].map(type=>({style:{},classList:{contains:x=>x===type}}));
 const buttons=['all','crossing','right'].map(value=>({dataset:{view:value},addEventListener(name,fn){this[name]=fn;}}));
 const mode={addEventListener(name,fn){this[name]=fn;}},plot={setAttribute(k,v){this[k]=v;}};
 const document={querySelectorAll:q=>q==='[data-view]'?buttons:edges,getElementById:id=>id==='plot'?plot:mode};
 vm.runInNewContext(code,{document});buttons[0].click();assert.equal(plot.viewBox,'-9550 -1050 11050 1550');buttons[1].click();assert.equal(plot.viewBox,'-9520 -400 1270 260');
 for(const value of ['straight_research','branch_research','source_only']){mode.change({target:{value}});assert(edges.every(e=>Number(e.style.opacity)>0));}
 assert(edges.every(e=>e.style.opacity==='1'));
});
