/* GTSD read-only reconstruction viewer. Geometry is consumed unchanged from
 * Blender evaluated meshes. No geometry generators, dynamics or external APIs. */
(function (root) {
  'use strict';
  const SCHEMA = 'gtsd-new-geometry-view-v1';
  const VIEWS = ['whole', 'platform', 'cutaway', 'bogie', 'pantograph', 'encoder'];
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const sub = (a, b) => a.map((x, i) => x - b[i]);
  const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const normal = a => { const n = Math.hypot(...a); return n > 1e-14 ? a.map(x => x/n) : [0, 0, 1]; };
  const textOf = value => value === undefined || value === null ? '' : typeof value === 'string' ? value : JSON.stringify(value, null, 2);

  function validateResource(r) {
    if (!r || r.schema !== SCHEMA) throw new Error('不是本重建查看器的 geometry.json（schema 不匹配）');
    if (r.units !== 'm') throw new Error('网格单位必须明确为 m');
    if (!Array.isArray(r.meshes) || !Array.isArray(r.registry)) throw new Error('缺少 meshes 或 registry');
    if (!r.meshes.length) throw new Error('该文件没有可显示的实际网格；不会生成替代模型');
    const ids = new Set();
    for (const m of r.meshes) {
      if (!m || typeof m.id !== 'string' || !m.id || ids.has(m.id)) throw new Error('网格 ID 缺失或重复');
      ids.add(m.id);
      if (!Array.isArray(m.vertices) || m.vertices.length < 9 || m.vertices.length % 3 || !m.vertices.every(Number.isFinite)) throw new Error('网格顶点无效：' + m.id);
      if (!Array.isArray(m.indices) || !m.indices.length || m.indices.length % 3 || !m.indices.every(i => Number.isInteger(i) && i >= 0 && i < m.vertices.length/3)) throw new Error('网格三角索引无效：' + m.id);
      if (!Array.isArray(m.color) || m.color.length < 3 || m.color.length > 4 || !m.color.every(x => Number.isFinite(x) && x >= 0 && x <= 1)) throw new Error('网格颜色无效：' + m.id);
      if (!['body', 'bogie', 'pantograph', 'display', 'environment'].includes(m.group)) throw new Error('网格分组无效：' + m.id);
    }
    const nodeIds = new Set();
    for (const n of r.registry) {
      if (!n || typeof n.id !== 'string' || !n.id || nodeIds.has(n.id)) throw new Error('部件登记节点 ID 缺失或重复');
      nodeIds.add(n.id);
    }
    return r;
  }
  function visibleMesh(m, view) {
    if (view === 'platform') return true;
    if (view === 'encoder') return m.group === 'bogie' && m.display_group !== 'encoder_cover';
    if (view === 'bogie' || view === 'pantograph') return m.group === view;
    const vehicle = ['body','bogie','pantograph'].includes(m.group);
    if (view === 'cutaway') return vehicle && m.display_group !== 'shell';
    return vehicle;
  }

  function boundsOf(meshes) {
    const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    for (const m of meshes) for (let k = 0; k < m.vertices.length; k += 3) for (let j = 0; j < 3; j++) {
      min[j] = Math.min(min[j], m.vertices[k+j]); max[j] = Math.max(max[j], m.vertices[k+j]);
    }
    if (!Number.isFinite(min[0])) return null;
    return {min, max, center: min.map((x, i) => (x+max[i])/2), size: min.map((x, i) => max[i]-x)};
  }
  function cameraMatrix(target, yaw, pitch, scale, aspect, depth) {
    const right = [Math.cos(yaw), Math.sin(yaw), 0];
    const up = [-Math.sin(yaw)*Math.sin(pitch), Math.cos(yaw)*Math.sin(pitch), Math.cos(pitch)];
    const front = cross(right, up), sx = scale*aspect/2, sy = scale/2, d = depth || 100;
    return [right[0]/sx, up[0]/sy, -front[0]/d, 0, right[1]/sx, up[1]/sy, -front[1]/d, 0, right[2]/sx, up[2]/sy, -front[2]/d, 0, -dot(right,target)/sx, -dot(up,target)/sy, dot(front,target)/d, 1];
  }
  const transform = (m, p) => [m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12], m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13], m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
  function fitScale(bounds, yaw, pitch, aspect) {
    // Fit projected source bounds rather than a much larger enclosing sphere.
    // 25% margin retains all rail/pantograph extremities in every orbit angle.
    const right = [Math.cos(yaw), Math.sin(yaw), 0];
    const up = [-Math.sin(yaw)*Math.sin(pitch), Math.cos(yaw)*Math.sin(pitch), Math.cos(pitch)];
    const width = right.reduce((n, x, i) => n+Math.abs(x)*bounds.size[i], 0);
    const height = up.reduce((n, x, i) => n+Math.abs(x)*bounds.size[i], 0);
    return Math.max(.04,height,width/aspect)*1.25;
  }
  function registryTree(registry) {
    const byId = new Map(registry.map(n => [n.id, {...n, children: []}])), roots = [];
    // Invalid or cyclic parent links are promoted to root rather than hiding data.
    for (const n of byId.values()) {
      let p = byId.get(n.parent || n.parent_id), cyclic = false;
      const seen = new Set([n.id]); let cursor = p;
      while (cursor) { if (seen.has(cursor.id)) {cyclic = true; break;} seen.add(cursor.id); cursor = byId.get(cursor.parent || cursor.parent_id); }
      if (p && !cyclic) p.children.push(n); else roots.push(n);
    }
    return {roots, byId};
  }
  function descendantIds(registry, id) {
    const ids = new Set([id]); let changed = true;
    while (changed) { changed = false; for (const n of registry) if (ids.has(n.parent || n.parent_id) && !ids.has(n.id)) { ids.add(n.id); changed = true; } }
    return ids;
  }
  function componentMeshIds(resource, id) {
    const ids = descendantIds(resource.registry, id);
    return resource.meshes.filter(m => ids.has(m.registered_component_id)).map(m => m.id);
  }
  function candidateMeshIds(resource, id) {
    return resource.meshes.filter(m => textOf(m.semantic_part_id).split(/[^A-Za-z0-9_-]+/).includes(id)).map(m => m.id);
  }
  const CPU_LIMITS = Object.freeze({maxPixels:480000,maxWidth:1000,maxHeight:800,maxTriangles:350000});
  const LITTLE_ENDIAN = new Uint8Array(new Uint32Array([0x01020304]).buffer)[0] === 4;
  function packRGBA(r,g,b,a) {return LITTLE_ENDIAN ? ((a<<24)|(b<<16)|(g<<8)|r)>>>0 : ((r<<24)|(g<<16)|(b<<8)|a)>>>0;}
  function softwareSize(width,height) {
    const scale=Math.min(1,CPU_LIMITS.maxWidth/width,CPU_LIMITS.maxHeight/height,Math.sqrt(CPU_LIMITS.maxPixels/(width*height)));
    return {width:Math.max(1,Math.floor(width*scale)),height:Math.max(1,Math.floor(height*scale)),scale};
  }
  function rasterTarget(width,height) {
    const pixels=new Uint8ClampedArray(width*height*4);
    const target={width,height,pixels,rgba32:new Uint32Array(pixels.buffer),depth:new Float64Array(width*height)};
    clearRaster(target); return target;
  }
  function clearRaster(target) {target.depth.fill(Infinity);target.rgba32.fill(packRGBA(11,17,24,255));}
  // Orthographic, per-pixel barycentric Z-buffer. Smaller camera z is nearer.
  // Incremental edge/depth planes avoid per-pixel arrays or allocation.
  function rasterTriangle(target,p,a,b,c,color,alpha) {
    const x0=p[a],y0=p[a+1],z0=p[a+2],x1=p[b],y1=p[b+1],z1=p[b+2],x2=p[c],y2=p[c+1],z2=p[c+2];
    const den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2);
    if(Math.abs(den)<1e-12)return 0;
    const minX=Math.max(0,Math.ceil(Math.min(x0,x1,x2)-.5)),maxX=Math.min(target.width-1,Math.floor(Math.max(x0,x1,x2)-.5));
    const minY=Math.max(0,Math.ceil(Math.min(y0,y1,y2)-.5)),maxY=Math.min(target.height-1,Math.floor(Math.max(y0,y1,y2)-.5));
    if(minX>maxX||minY>maxY)return 0;
    const inv=1/den,ax=(y1-y2)*inv,ay=(x2-x1)*inv,bx=(y2-y0)*inv,by=(x0-x2)*inv;
    let ar=((y1-y2)*(minX+.5-x2)+(x2-x1)*(minY+.5-y2))*inv;
    let br=((y2-y0)*(minX+.5-x2)+(x0-x2)*(minY+.5-y2))*inv;
    const dz0=z0-z2,dz1=z1-z2,zx=ax*dz0+bx*dz1,zy=ay*dz0+by*dz1;
    let zr=z2+ar*dz0+br*dz1,covered=0;
    const depth=target.depth,rgba=target.rgba32,pixels=target.pixels,transparent=alpha!==undefined&&alpha<.999;
    let red,green,blue;if(transparent){const bytes=new Uint8Array(new Uint32Array([color]).buffer);red=bytes[0];green=bytes[1];blue=bytes[2];}
    const cx=-ax-bx,invAx=ax?1/ax:0,invBx=bx?1/bx:0,invCx=cx?1/cx:0,rowWidth=maxX-minX;
    for(let y=minY;y<=maxY;y++){
      // Solve the three linear edge inequalities once per scanline. Iterating
      // only the covered span avoids giant empty bounding boxes at close zoom.
      let lo=0,hi=rowWidth;const cr=1-ar-br;
      if(ax>0)lo=Math.max(lo,Math.ceil((-1e-9-ar)*invAx));else if(ax<0)hi=Math.min(hi,Math.floor((-1e-9-ar)*invAx));else if(ar<-1e-9)hi=-1;
      if(bx>0)lo=Math.max(lo,Math.ceil((-1e-9-br)*invBx));else if(bx<0)hi=Math.min(hi,Math.floor((-1e-9-br)*invBx));else if(br<-1e-9)hi=-1;
      if(cx>0)lo=Math.max(lo,Math.ceil((-1e-9-cr)*invCx));else if(cx<0)hi=Math.min(hi,Math.floor((-1e-9-cr)*invCx));else if(cr<-1e-9)hi=-1;
      let z=zr+lo*zx,index=y*target.width+minX+lo;
      for(let x=lo;x<=hi;x++,index++,z+=zx){
        if(z>=-1&&z<=1&&z<depth[index]){
          if(transparent){const q=index*4;pixels[q]=Math.round(red*alpha+pixels[q]*(1-alpha));pixels[q+1]=Math.round(green*alpha+pixels[q+1]*(1-alpha));pixels[q+2]=Math.round(blue*alpha+pixels[q+2]*(1-alpha));}
          else{depth[index]=z;rgba[index]=color;}
          covered++;
        }
      }
      ar+=ay;br+=by;zr+=zy;
    }

    return covered;
  }
  function prepareMesh(record) {
    const v = record.vertices, ix = record.indices, normals = new Float32Array(ix.length);
    const packed = new Float32Array(ix.length * 6);
    for (let k = 0; k < ix.length; k += 3) {
      const p = [0,1,2].map(j => v.slice(ix[k+j]*3, ix[k+j]*3+3));
      const n = normal(cross(sub(p[1], p[0]), sub(p[2], p[0]))); normals.set(n, k);
      for (let j = 0; j < 3; j++) { packed.set(p[j], (k+j)*6); packed.set(n, (k+j)*6+3); }
    }
    const color = [record.color[0], record.color[1], record.color[2], record.color[3] === undefined ? 1 : record.color[3]];
    const sun = normal([.25,-.65,1]);
    const shades = Array.from({length:32}, (_, j) => 'rgba('+color.slice(0,3).map(x => Math.round(255*Math.pow(clamp(x*(.42+.58*j/31),0,1),1/2.2))).join(',')+','+color[3]+')');
    const shadeIndices = new Uint8Array(ix.length/3);
    for (let k = 0; k < ix.length; k += 3) shadeIndices[k/3] = Math.round(31*Math.abs(dot(Array.from(normals.slice(k,k+3)),sun)));
    const pixelColors=Array.from({length:32},(_,j)=>packRGBA(...color.slice(0,3).map(x=>Math.round(255*Math.pow(clamp(x*(.42+.58*j/31),0,1),1/2.2))),255));
    return {record, normals, packed, color, shades, pixelColors, shadeIndices, projected: new Float64Array(v.length), bounds: boundsOf([record])};
  }

  class ModelView {
    constructor(canvas, options) {
      this.canvas = canvas; this.options = options || {}; this.resource = null; this.meshes = []; this.meshById = new Map(); this.selectedIds = new Set();
      this.view = 'whole'; this.yaw = .52; this.pitch = .38; this.zoom = 1; this.frameBounds = null; this.renderCount = 0; this.destroyed = false;
      this.onPointerDown = e => {
        if (e.button !== undefined && e.button !== 0) return;
        this.drag = {x:e.clientX, y:e.clientY, yaw:this.yaw, pitch:this.pitch};
        this.canvas.focus({preventScroll:true});
        if (this.canvas.setPointerCapture) try {this.canvas.setPointerCapture(e.pointerId);} catch (_) {}
      };
      this.onPointerMove = e => {if (!this.drag) return; this.yaw = this.drag.yaw-(e.clientX-this.drag.x)*.007; this.pitch = clamp(this.drag.pitch+(e.clientY-this.drag.y)*.005,-1.45,1.45); this.requestDraw();};
      this.onPointerUp = () => {this.drag = null;};
      this.onWheel = e => {e.preventDefault(); this.zoomBy(Math.exp(clamp(e.deltaY,-300,300)*.0015));};
      this.onKeyDown = e => {
        let handled = true;
        switch (e.key) {
          case '+': case '=': this.zoomBy(.8); break;
          case '-': this.zoomBy(1.25); break;
          case 'Home': this.resetCamera(); break;
          case 'Escape': this.select([]); break;
          case 'ArrowLeft': this.yaw -= .1; this.requestDraw(); break;
          case 'ArrowRight': this.yaw += .1; this.requestDraw(); break;
          case 'ArrowUp': this.pitch = clamp(this.pitch+.08,-1.45,1.45); this.requestDraw(); break;
          case 'ArrowDown': this.pitch = clamp(this.pitch-.08,-1.45,1.45); this.requestDraw(); break;
          default: handled = false;
        }
        if (handled) e.preventDefault();
      };
      this.onLost = e => {e.preventDefault(); this.fallback('WebGL 上下文已丢失'); this.requestDraw();};
      this.onCreationError = e => {this.creationError = String(e.statusMessage || '').slice(0,200);};
      this.resizeObserver = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(() => this.requestDraw()) : null;
      canvas.addEventListener('webglcontextcreationerror', this.onCreationError);
      try {
        if (!this.options.forceCanvas) this.gl = canvas.getContext('webgl', {alpha:false,antialias:true,preserveDrawingBuffer:true});
        if (!this.gl) this.fallback(this.options.forceCanvas ? '已手动选择兼容模式' : '浏览器没有可用的 WebGL'+(this.creationError ? '：'+this.creationError : ''));
        else {this.setupGL(); this.mode = 'WebGL 实体渲染';}
      } catch (e) {this.fallback('WebGL 初始化失败：'+e.message);}
      this.bind(); this.notify();
    }
    bind() {
      const c = this.canvas;
      for (const [event,fn] of [['pointerdown',this.onPointerDown],['pointermove',this.onPointerMove],['pointerup',this.onPointerUp],['pointercancel',this.onPointerUp],['lostpointercapture',this.onPointerUp],['keydown',this.onKeyDown],['webglcontextlost',this.onLost],['webglcontextcreationerror',this.onCreationError]]) c.addEventListener(event,fn);
      c.addEventListener('wheel',this.onWheel,{passive:false}); c.style.touchAction = 'none'; c.tabIndex = 0;
      if (this.resizeObserver) this.resizeObserver.observe(c);
    }
    unbind() {
      const c = this.canvas;
      for (const [event,fn] of [['pointerdown',this.onPointerDown],['pointermove',this.onPointerMove],['pointerup',this.onPointerUp],['pointercancel',this.onPointerUp],['lostpointercapture',this.onPointerUp],['keydown',this.onKeyDown],['webglcontextlost',this.onLost],['webglcontextcreationerror',this.onCreationError],['wheel',this.onWheel]]) c.removeEventListener(event,fn);
      if (this.resizeObserver) this.resizeObserver.unobserve(c);
    }
    releaseGL() {
      if (!this.gl) return;
      try {for (const m of this.meshes) if (m.buffer) this.gl.deleteBuffer(m.buffer); if (this.program) this.gl.deleteProgram(this.program);} catch (_) {}
      this.program = null; for (const m of this.meshes) m.buffer = null;
    }
    fallback(reason) {
      this.releaseGL(); this.gl = null; this.rendererNotice = reason; this.mode = 'Canvas CPU 深度渲染'; this.drag = null;
      let ctx; try {ctx = this.canvas.getContext('2d');} catch (_) {}
      if (!ctx && this.canvas.parentNode && this.canvas.cloneNode) {
        const old = this.canvas, next = old.cloneNode(false), focused = old.ownerDocument.activeElement === old;
        this.unbind(); old.parentNode.replaceChild(next,old); this.canvas = next; this.bind();
        if (focused) next.focus({preventScroll:true});
        try {ctx = next.getContext('2d');} catch (_) {}
      }
      this.ctx = ctx; this.unavailable = !ctx;
      if (!ctx) this.rendererNotice += '；Canvas 2D 也不可用，保留部件和来源信息';
      this.notify();
    }
    setupGL() {
      const gl = this.gl, shaders = [];
      try {
        const compile = (kind, source) => {const s = gl.createShader(kind); if (!s) throw new Error('无法创建着色器'); shaders.push(s); gl.shaderSource(s,source); gl.compileShader(s); if (!gl.getShaderParameter(s,gl.COMPILE_STATUS)) throw new Error(String(gl.getShaderInfoLog(s)).slice(0,180)); return s;};
        const vs = compile(gl.VERTEX_SHADER,'attribute vec3 aPosition; attribute vec3 aNormal; uniform mat4 uCamera; varying float vLight; void main(){ vLight=.42+.58*abs(dot(normalize(aNormal),normalize(vec3(.25,-.65,1.)))); gl_Position=uCamera*vec4(aPosition,1.); }');
        const fs = compile(gl.FRAGMENT_SHADER,'precision mediump float; uniform vec4 uColor; varying float vLight; void main(){gl_FragColor=vec4(pow(clamp(uColor.rgb*vLight,0.,1.),vec3(1./2.2)),uColor.a);}');
        const p = gl.createProgram(); this.program = p; if (!p) throw new Error('无法创建着色程序');
        gl.attachShader(p,vs); gl.attachShader(p,fs); gl.linkProgram(p); if (!gl.getProgramParameter(p,gl.LINK_STATUS)) throw new Error(String(gl.getProgramInfoLog(p)).slice(0,180));
        this.locations = {position:gl.getAttribLocation(p,'aPosition'),normal:gl.getAttribLocation(p,'aNormal'),camera:gl.getUniformLocation(p,'uCamera'),color:gl.getUniformLocation(p,'uColor')};
        if (this.locations.position < 0 || this.locations.normal < 0 || this.locations.camera === null || this.locations.color === null) throw new Error('着色器接口不完整');
        gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE); gl.clearColor(.043,.065,.094,1); gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);
      } finally {for (const s of shaders) gl.deleteShader(s);}
    }
    setResource(resource) {
      validateResource(resource);
      if (this.gl) for (const m of this.meshes) if (m.buffer) this.gl.deleteBuffer(m.buffer);
      this.resource = resource; this.meshes = resource.meshes.map(prepareMesh); this.meshById = new Map(this.meshes.map(m => [m.record.id,m])); this.selectedIds = new Set();
      if (this.gl) try {
        for (const m of this.meshes) {m.buffer = this.gl.createBuffer(); if (!m.buffer) throw new Error('无法创建网格缓冲区'); this.gl.bindBuffer(this.gl.ARRAY_BUFFER,m.buffer); this.gl.bufferData(this.gl.ARRAY_BUFFER,m.packed,this.gl.STATIC_DRAW);}
        if (this.gl.getError() !== this.gl.NO_ERROR) throw new Error('几何上传返回 WebGL 错误');
      } catch (e) {this.fallback('WebGL 几何上传失败：'+e.message);}
      // Keep the exact source arrays for software rendering. GPU expansion changes
      // only storage layout; it does not generate or reposition geometry.
      for (const m of this.meshes) delete m.packed;
      this.setView('whole'); return resource;
    }
    async load(url) {
      const response = await fetch(url || 'geometry.json',{cache:'no-cache'});
      if (!response.ok) throw new Error('HTTP '+response.status);
      return this.setResource(await response.json());
    }
    setView(view) {
      if (!VIEWS.includes(view)) return false;
      this.view = view; this.frameBounds = boundsOf(this.resource ? this.resource.meshes.filter(m => visibleMesh(m,view)) : []);
      this.yaw = .52; this.pitch = view === 'pantograph' ? .22 : .38; this.zoom = 1; this.requestDraw(); return true;
    }
    resetCamera() {this.setView(this.view);}
    zoomBy(factor) {if (!Number.isFinite(factor) || factor <= 0) return; this.zoom = clamp(this.zoom*factor,.15,8); this.requestDraw();}
    select(ids) {this.selectedIds = new Set(ids.filter(id => this.meshById.has(id))); this.requestDraw();}
    focusSelection() {
      const selection = this.resource && this.resource.meshes.filter(m => this.selectedIds.has(m.id));
      if (!selection || !selection.length) return false;
      if (selection.some(m => !visibleMesh(m,this.view))) this.view = selection.some(m => !visibleMesh(m,'whole')) ? 'platform' : 'whole';
      this.frameBounds = boundsOf(selection); this.zoom = 1; this.requestDraw(); return true;
    }
    requestDraw() {
      if (this.destroyed || this.framePending) return;
      if (typeof requestAnimationFrame === 'undefined') {this.draw(); return;}
      this.framePending = requestAnimationFrame(() => {this.framePending = null; this.draw();});
    }
    camera(width,height) {
      const b = this.frameBounds || boundsOf(this.resource.meshes), aspect = width/height;
      const radius = Math.max(.02,Math.hypot(...b.size)/2);
      return cameraMatrix(b.center,this.yaw,this.pitch,fitScale(b,this.yaw,this.pitch,aspect)*this.zoom,aspect,Math.max(100,radius*20));
    }
    drawGL(visible,camera) {
      const gl = this.gl, l = this.locations;
      gl.viewport(0,0,this.canvas.width,this.canvas.height); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT); gl.useProgram(this.program); gl.uniformMatrix4fv(l.camera,false,new Float32Array(camera));
      const opaque = visible.filter(m => m.color[3] >= .999 || this.selectedIds.has(m.record.id));
      const transparent = visible.filter(m => m.color[3] < .999 && !this.selectedIds.has(m.record.id));
      transparent.sort((a,b) => transform(camera,b.bounds.center)[2]-transform(camera,a.bounds.center)[2]);
      const draw = m => {
        gl.bindBuffer(gl.ARRAY_BUFFER,m.buffer); gl.enableVertexAttribArray(l.position); gl.vertexAttribPointer(l.position,3,gl.FLOAT,false,24,0); gl.enableVertexAttribArray(l.normal); gl.vertexAttribPointer(l.normal,3,gl.FLOAT,false,24,12);
        gl.uniform4fv(l.color,new Float32Array(this.selectedIds.has(m.record.id) ? [1,.58,.075,1] : m.color)); gl.drawArrays(gl.TRIANGLES,0,m.record.indices.length);
      };
      gl.disable(gl.BLEND); gl.depthMask(true); for (const m of opaque) draw(m);
      gl.enable(gl.BLEND); gl.depthMask(false); for (const m of transparent) draw(m); gl.depthMask(true); gl.disable(gl.BLEND);
      if (gl.isContextLost() || gl.getError() !== gl.NO_ERROR) throw new Error('WebGL 绘制错误或上下文丢失');
    }
    drawSoftware(visible,camera) {
      const ctx=this.ctx,w=this.canvas.width,h=this.canvas.height,sourceTriangles=visible.reduce((n,m)=>n+m.record.indices.length/3,0);
      const started=typeof performance!=='undefined'?performance.now():Date.now();
      if(!this.raster||this.raster.width!==w||this.raster.height!==h){this.raster=rasterTarget(w,h);this.imageData=ctx.createImageData(w,h);}else clearRaster(this.raster);
      if(sourceTriangles>CPU_LIMITS.maxTriangles){
        this.imageData.data.set(this.raster.pixels);ctx.putImageData(this.imageData,0,0);
        this.softwareStats={drawn:0,sourceTriangles,candidates:sourceTriangles,detailLimited:false,blocked:true,width:w,height:h,triangleLimit:CPU_LIMITS.maxTriangles,reason:'当前预设超过 CPU '+CPU_LIMITS.maxTriangles+' 三角形安全上限，请选子组件或在支持 WebGL 的浏览器打开。'};
        return;
      }
      let submitted=0,coveredTriangles=0,pixelWrites=0;const transparent=[];
      const highlight=packRGBA(255,194,91,255);
      for(const m of visible){
        const v=m.record.vertices,ix=m.record.indices,p=m.projected;
        for(let k=0;k<v.length;k+=3){const x=v[k],y=v[k+1],z=v[k+2];p[k]=(camera[0]*x+camera[4]*y+camera[8]*z+camera[12]+1)*w/2;p[k+1]=(1-camera[1]*x-camera[5]*y-camera[9]*z-camera[13])*h/2;p[k+2]=camera[2]*x+camera[6]*y+camera[10]*z+camera[14];}
        const selected=this.selectedIds.has(m.record.id);
        for(let k=0;k<ix.length;k+=3){
          const a=ix[k]*3,b=ix[k+1]*3,c=ix[k+2]*3,color=selected?highlight:m.pixelColors[m.shadeIndices[k/3]];
          submitted++;
          if(m.color[3]<.999&&!selected){transparent.push({p,a,b,c,color,alpha:m.color[3],z:(p[a+2]+p[b+2]+p[c+2])/3});continue;}
          const writes=rasterTriangle(this.raster,p,a,b,c,color,1);if(writes)coveredTriangles++;pixelWrites+=writes;
        }
      }
      // Current reconstruction is entirely opaque. For future translucent
      // materials, retain opaque Z rejection and sort only the transparent pass.
      transparent.sort((a,b)=>b.z-a.z);
      for(const t of transparent){const writes=rasterTriangle(this.raster,t.p,t.a,t.b,t.c,t.color,t.alpha);if(writes)coveredTriangles++;pixelWrites+=writes;}
      this.imageData.data.set(this.raster.pixels);ctx.putImageData(this.imageData,0,0);
      const ended=typeof performance!=='undefined'?performance.now():Date.now();
      this.softwareStats={drawn:submitted,candidates:sourceTriangles,sourceTriangles,coveredTriangles,pixelWrites,detailLimited:false,blocked:false,width:w,height:h,pixelLimit:CPU_LIMITS.maxPixels,triangleLimit:CPU_LIMITS.maxTriangles,elapsedMs:Math.round((ended-started)*10)/10,transparentTriangles:transparent.length};
    }

    draw() {
      if (this.destroyed || !this.resource) return;
      if (this.unavailable) {this.notify(); return;}
      const width = Math.max(1,this.canvas.clientWidth || 1000), height = Math.max(1,this.canvas.clientHeight || 600), dpr = Math.min(2,typeof devicePixelRatio === 'number' ? devicePixelRatio : 1);
      const size=this.gl?{width:Math.round(width*dpr),height:Math.round(height*dpr)}:softwareSize(width,height);
      if (this.canvas.width !== size.width || this.canvas.height !== size.height) {this.canvas.width=size.width; this.canvas.height=size.height;}
      const visible = this.meshes.filter(m => visibleMesh(m.record,this.view)), camera = this.camera(width,height); this.lastCamera = camera;
      try {if (this.gl) this.drawGL(visible,camera); else if (this.ctx) this.drawSoftware(visible,camera);}
      catch (e) {if (this.gl) {this.fallback('WebGL 绘制失败：'+e.message); if (this.ctx) {this.draw();return;}} else {this.rendererNotice='Canvas CPU 渲染失败：'+e.message; this.unavailable=true;}}
      this.renderCount++; this.notify();
    }
    snapshot() {
      const meshes = this.resource ? this.resource.meshes.filter(m => visibleMesh(m,this.view)) : [];
      return {mode:this.mode,notice:this.rendererNotice || '',unavailable:!!this.unavailable,view:this.view,visibleMeshes:meshes.length,visibleTriangles:meshes.reduce((n,m) => n+m.indices.length/3,0),selectedVisible:meshes.filter(m => this.selectedIds.has(m.id)).length,selectedCount:this.selectedIds.size,renderCount:this.renderCount,softwareStats:this.softwareStats || null};
    }
    notify() {if (typeof this.options.onStatus === 'function') this.options.onStatus(this.snapshot());}
    destroy() {this.destroyed=true; this.unbind(); if (this.resizeObserver) this.resizeObserver.disconnect(); if (this.framePending && typeof cancelAnimationFrame !== 'undefined') cancelAnimationFrame(this.framePending); this.releaseGL();}
  }
  const api = {SCHEMA,VIEWS,CPU_LIMITS,softwareSize,rasterTarget,clearRaster,rasterTriangle,packRGBA,validateResource,visibleMesh,boundsOf,cameraMatrix,transform,fitScale,registryTree,descendantIds,componentMeshIds,candidateMeshIds,prepareMesh,textOf,ModelView};
  if (typeof module !== 'undefined' && module.exports) module.exports=api;
  root.GTSDReconstruction=api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
