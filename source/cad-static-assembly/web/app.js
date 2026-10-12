/* UI has no upload, mutation, physics or remote-service operation. */
(function () {
  'use strict';
  const G = window.GTSDReconstruction, $ = id => document.getElementById(id);
  const groupNames = {body:'车体与车载设备',bogie:'转向架与走行部件',pantograph:'受电弓',display:'展示参照',environment:'全台架、接触网与控制台'};
  const viewNames = {whole:'整车',platform:'全台架',cutaway:'剖开壳体',bogie:'仅转向架',pantograph:'仅受电弓',encoder:'打开编码器罩'};
  const presetNotes = {
    whole:'整车预设仅显示车体、转向架与受电弓，隐藏全台架和展示参照；切到“全台架”检查轨道、接触网与控制台。',
    platform:'全台架：显示全部导出网格。轨轮廓中心距是 CAD 图面候选值，轨头/轨底断面解释、接触对齐及部分台架布置仍为工程显示假设。',
    cutaway:'剖开壳体：仅显示车辆，并隐藏 display_group = shell 的完整网格。没有生成切面，也不意味着内部布置已由 CAD 验证。',
    bogie:'仅显示新装配中 group = bogie 的网格；轴距受 CAD 约束，横向轮距与空间布置仍含照片推断。',
    pantograph:'仅显示新装配中 group = pantograph 的网格；静态几何不代表升降弓运动学或物理已经验证。',
    encoder:'仅隐藏照片可见编码器候选轴端的展示深杯罩；轴号、罩深与安装尺寸为照片推断，不代表已经确定实际传感器编号。'
  };
  let resource = null, tab = 'mesh', selected = null, meshButtons = [], nodeButtons = [], busy = false;
  function element(tag, cls, text) {const e=document.createElement(tag); if(cls)e.className=cls; if(text!==undefined)e.textContent=text; return e;}
  function setText(id,text) {$(id).textContent=text;}
  function formatNumber(n) {return Number(n).toLocaleString('zh-CN');}
  function addField(dl,label,value) {dl.append(element('dt','',label),element('dd','',G.textOf(value) || '未提供'));}
  function status(s) {
    const warning=$('renderer-warning');
    warning.hidden = !s.notice;
    const cpu=s.softwareStats;
    warning.textContent = s.notice ? s.notice + (s.unavailable ? '。' : '。当前为 Canvas CPU 逐像素深度渲染（Z-buffer），不是 WebGL；像素分辨率受限以保持响应。'+(cpu&&cpu.blocked?' '+cpu.reason:'')+(cpu&&cpu.transparentTriangles?' 透明材质仍使用排序混合，交叠透明面可能有误差。':'')) : '';
    setText('render-status',s.mode + (resource ? ' · '+formatNumber(s.visibleMeshes)+' 网格 · '+formatNumber(s.visibleTriangles)+' 源三角形'+(s.mode==='Canvas CPU 深度渲染' && cpu ? ' · '+cpu.width+'×'+cpu.height+' px · '+(cpu.blocked?'超出安全预算':'完整深度测试 · '+cpu.elapsedMs+' ms') : '') : ' · 等待网格') + (s.selectedCount ? ' · 选中 '+s.selectedVisible+'/'+s.selectedCount+' 在当前预设' : ''));
    setText('view-label',viewNames[s.view]); setText('preset-note',presetNotes[s.view]);
    document.querySelectorAll('[data-view]').forEach(b=>{const active=b.dataset.view===s.view;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});
    $('software-mode').disabled = !view || !view.gl;
    if (resource && s.visibleMeshes === 0) {showOverlay('该预设没有可显示的网格','导出文件中没有该分组的实体；不会用占位形状代替。',false);} else if (resource) $('load-overlay').hidden=true;
  }
  let view;
  try {view = new G.ModelView($('model-canvas'),{onStatus:status,forceCanvas:new URLSearchParams(location.search).get('renderer')==='canvas'});}
  catch (e) {showOverlay('查看器初始化失败',e.message,true); return;}
  window.GTSD_VIEWER = {get view(){return view;},get resource(){return resource;},get selection(){return selected;}};
  function showOverlay(title,description,canChoose) {setText('load-title',title);setText('load-description',description);$('load-overlay').hidden=false;document.querySelector('.file-choice').hidden=!canChoose;}
  function showResourceInfo() {
    const d=resource.dimensions || {}, mm=(id,key)=>{setText(id,Number.isFinite(d[key])?(d[key]*1000).toLocaleString('zh-CN',{minimumFractionDigits:2,maximumFractionDigits:2})+' mm':'源数据未提供');};
    mm('dim-length','body_length_m');mm('dim-height','body_height_m');mm('dim-axle','axle_distance_m');setText('dim-width',Number.isFinite(d.body_width_m)?(d.body_width_m*1000).toLocaleString('zh-CN',{minimumFractionDigits:6,maximumFractionDigits:6})+' mm':'源数据未提供');
    setText('dim-rail-centers',Number.isFinite(d.rail_contour_center_distance_m)?(d.rail_contour_center_distance_m*1000).toLocaleString('zh-CN',{minimumFractionDigits:6,maximumFractionDigits:6})+' mm':'源数据未提供');
    const railMm=key=>Number.isFinite(d[key])?(d[key]*1000).toLocaleString('zh-CN',{maximumFractionDigits:3})+' mm':'源数据未提供';
    setText('rail-section-note','轨断面显示解释（工程假设）：'+railMm('display_rail_head_width_m')+' 内轮廓暂作轨头，'+railMm('display_rail_base_width_m')+' 外轮廓暂作轨底；二维轮廓本身不能确认这些断面角色，轨中心距也不是标准轨距。');
    const count=resource.registry.filter(n=>n.level==='COMPONENT').length;
    setText('registry-count',resource.registry.length+' 节点 · '+count+' 逻辑部件');setText('mesh-count',resource.meshes.length);setText('node-count',resource.registry.length);
    const dl=$('resource-info');dl.replaceChildren();
    addField(dl,'源版本',resource.source_commit);addField(dl,'坐标与单位',resource.coordinate_system+' · '+resource.units);
    addField(dl,'实际网格实例',resource.meshes.length);addField(dl,'三角形',resource.meshes.reduce((n,m)=>n+m.indices.length/3,0));
    addField(dl,'登记层级',resource.registry.length+' 节点 / '+count+' 逻辑部件');
    addField(dl,'登记直接关联的节点',new Set(resource.meshes.map(m=>m.registered_component_id).filter(Boolean)).size+' 个（网格实例与登记部件不是一一对应）');
    if(resource.schema)addField(dl,'数据版本',resource.schema);
    setText('commit-label','源版本 '+String(resource.source_commit || '未提供').slice(0,12));
    setText('assumptions',G.textOf(resource.assumptions || d.evidence || '源数据未提供假设说明。'));
    if(resource.registry.length!==109) addField(dl,'登记完整性提醒','本文件实际提供 '+resource.registry.length+' 节点；预期登记表为 109 节点。');
  }
  function searchable(value) {return G.textOf(value).toLocaleLowerCase();}
  function buildLists() {
    const query=$('component-search').value.trim().toLocaleLowerCase();meshButtons=[];nodeButtons=[];
    const meshList=$('mesh-list');meshList.replaceChildren();
    let foundMeshes=0,foundNodes=0;
    for(const group of Object.keys(groupNames)) {
      const meshes=resource.meshes.filter(m=>m.group===group && (!query || searchable([m.id,m.name,m.registered_component_id,m.source_refs,m.geometry_status,m.semantic_part_id,m.geometry_assumptions]).includes(query)));
      if(!meshes.length)continue;
      meshList.append(element('div','list-group-label',groupNames[group]+' · '+meshes.length));
      for(const m of meshes) {
        foundMeshes++;const b=element('button','mesh-row'),swatch=element('span','swatch'),main=element('span','row-main');
        swatch.style.background='rgb('+m.color.slice(0,3).map(c=>Math.round(255*Math.pow(c,1/2.2))).join(',')+')';
        main.append(element('span','row-name',m.name || m.id),element('span','row-meta',m.registered_component_id || '未关联登记节点'));
        b.append(swatch,main);b.type='button';b.dataset.meshId=m.id;b.setAttribute('aria-pressed',String(!!selected && selected.type==='mesh' && selected.id===m.id));
        b.classList.toggle('selected',!!selected && selected.type==='mesh' && selected.id===m.id);b.addEventListener('click',()=>selectMesh(m.id));meshButtons.push(b);meshList.append(b);
      }
    }
    const tree=G.registryTree(resource.registry), list=$('registry-list');list.replaceChildren();
    const matched=new Set(resource.registry.filter(n=>!query || searchable(n).includes(query)).map(n=>n.id));
    if(query)for(const n of resource.registry)if(matched.has(n.id)){let p=tree.byId.get(n.parent),visited=new Set();while(p&&!visited.has(p.id)){visited.add(p.id);matched.add(p.id);p=tree.byId.get(p.parent);}}
    function appendNode(n,parent) {
      if(!matched.has(n.id))return;
      foundNodes++;const b=element('button','registry-row node-'+String(n.level || '').toLowerCase()),main=element('span','row-main'),ids=G.componentMeshIds(resource,n.id);
      main.append(element('span','row-name',n.name || n.id),element('span','row-meta',n.id));b.append(main,element('span','row-count',ids.length?ids.length+' 网格':G.candidateMeshIds(resource,n.id).length?'候选未归属':'未建模'));
      b.type='button';b.dataset.nodeId=n.id;b.classList.toggle('unmodeled',!ids.length);b.classList.toggle('selected',!!selected&&selected.type==='node'&&selected.id===n.id);b.setAttribute('aria-pressed',String(!!selected&&selected.type==='node'&&selected.id===n.id));
      b.addEventListener('click',()=>selectNode(n.id));nodeButtons.push(b);parent.append(b);
      if(n.children.length){const children=element('div','tree-children');for(const c of n.children)appendNode(c,children);if(children.childNodes.length)parent.append(children);}
    }
    for(const n of tree.roots)appendNode(n,list);
    $('no-results').hidden=(tab==='mesh'?foundMeshes:foundNodes)>0;
  }
  function updateButtons() {
    for(const b of meshButtons){const active=!!selected&&selected.type==='mesh'&&selected.id===b.dataset.meshId;b.classList.toggle('selected',active);b.setAttribute('aria-pressed',String(active));}
    for(const b of nodeButtons){const active=!!selected&&selected.type==='node'&&selected.id===b.dataset.nodeId;b.classList.toggle('selected',active);b.setAttribute('aria-pressed',String(active));}
  }
  function startDetail(name,ids) {
    setText('selection-title',name);$('selection-hint').hidden=true;$('clear-selection').hidden=false;
    $('selection-detail').replaceChildren();$('focus-selection').hidden=!ids.length;
    view.select(ids);updateButtons();const dl=element('dl');$('selection-detail').append(dl);return dl;
  }
  function sourceBlock(label,data) {const detail=$('selection-detail');detail.append(element('div','source-label',label),element('pre','source-text',G.textOf(data)||'该条目未提供来源引用；不能视为来源已验证。'));}
  function selectMesh(id) {
    const m=resource.meshes.find(x=>x.id===id);if(!m)return;selected={type:'mesh',id};
    const dl=startDetail(m.name || m.id,[m.id]);
    addField(dl,'实体 ID',m.id);addField(dl,'登记节点',m.registered_component_id || '未关联');addField(dl,'几何状态',m.geometry_status || '未标明');addField(dl,'装配分组',groupNames[m.group]);
    addField(dl,'显示分类',m.display_group==='shell'?'壳体（剖开时隐藏）':m.display_group || '普通实体');addField(dl,'三角形',m.indices.length/3);
    if(!G.visibleMesh(m,view.view))$('selection-detail').append(element('p','selection-note','该实体在当前预设中隐藏。点击下方按钮可显示并聚焦。'));
    sourceBlock('来源引用 · source_refs',m.source_refs);
    sourceBlock('源层/装配状态',m.assembly_layer);
    for (const [key,label] of [['geometry_assumptions','几何假设'],['assumptions','参数假设'],['source_metadata','源元数据'],['notes','备注'],['status','来源/候选限制'],['placement_assumptions','安装位置假设']]) if(m[key]!==undefined) sourceBlock(label,m[key]);
    addField(dl,'当前物理状态','静态源几何；此页面未绑定求解器');
    if(m.semantic_part_id)sourceBlock('部件身份候选 · semantic_part_id',m.semantic_part_id);
    if(m.geometry_assumptions)sourceBlock('本实体的几何假设',m.geometry_assumptions);
    if(m.registry_match_status)addField(dl,'登记匹配状态',m.registry_match_status);
    if(m.dimension_status)addField(dl,'尺寸证据状态',m.dimension_status);
  }
  function selectNode(id) {
    const n=resource.registry.find(x=>x.id===id);if(!n)return;selected={type:'node',id};
    const ids=G.componentMeshIds(resource,id),direct=resource.meshes.filter(m=>m.registered_component_id===id),dl=startDetail(n.name || id,ids);
    addField(dl,'登记 ID',n.id);addField(dl,'层级',n.level);addField(dl,'父节点',n.parent || n.parent_id);addField(dl,'编码',n.code);addField(dl,'原表行号',n.source_row);
    addField(dl,'直接关联',direct.length+' 个网格实例');addField(dl,'含登记子级',ids.length+' 个网格实例');
    const candidates=G.candidateMeshIds(resource,id);
    if(!ids.length)$('selection-detail').append(element('p','selection-note',candidates.length?'没有已确认的几何归属；存在身份候选实体，尚未强行对应到此登记节点。':'仅登记，未建模。此节点保留源表信息，没有用占位实体补齐。'));
    else {
      const meshes=resource.meshes.filter(m=>ids.includes(m.id)),states=[...new Set(meshes.map(m=>m.geometry_status || '未标明'))];addField(dl,'几何状态',states.join('\n'));
      sourceBlock('关联实体来源 · 去重汇总',[...new Set(meshes.map(m=>G.textOf(m.source_refs)).filter(Boolean))].join('\n\n'));
    }
    if(n.source_refs)sourceBlock('登记来源',n.source_refs);
    if(candidates.length) {
      addField(dl,'身份候选',candidates.length+' 个网格实例（不是已确认设备数量）');
      const button=element('button','candidate-link','查看候选实体');button.type='button';button.addEventListener('click',()=>{$('component-search').value=id;selectTab('mesh');});$('selection-detail').append(button);
    }
  }
  function clearSelection() {selected=null;view.select([]);updateButtons();$('selection-detail').replaceChildren();setText('selection-title','检查来源');$('selection-hint').hidden=false;$('clear-selection').hidden=true;$('focus-selection').hidden=true;}
  function selectTab(next) {tab=next;$('mesh-list').hidden=tab!=='mesh';$('registry-list').hidden=tab!=='registry';for(const k of ['mesh','registry']){$(k+'-tab').classList.toggle('active',tab===k);$(k+'-tab').setAttribute('aria-selected',String(tab===k));}if(resource)buildLists();}
  function acceptResource(data) {
    G.validateResource(data);resource=data;view.setResource(data);showResourceInfo();clearSelection();buildLists();$('load-overlay').hidden=true;status(view.snapshot());
  }
  $('mesh-tab').addEventListener('click',()=>selectTab('mesh'));$('registry-tab').addEventListener('click',()=>selectTab('registry'));
  $('component-search').addEventListener('input',()=>{if(resource)buildLists();});
  document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{view.setView(b.dataset.view);if(selected){if(selected.type==='mesh')selectMesh(selected.id);else selectNode(selected.id);}}));
  $('zoom-in').addEventListener('click',()=>view.zoomBy(.8));$('zoom-out').addEventListener('click',()=>view.zoomBy(1.25));$('reset').addEventListener('click',()=>view.resetCamera());
  $('clear-selection').addEventListener('click',clearSelection);$('focus-selection').addEventListener('click',()=>{view.focusSelection();if(selected&&selected.type==='mesh')selectMesh(selected.id);});
  $('software-mode').addEventListener('click',()=>{view.fallback('已手动选择兼容模式');view.requestDraw();});
  $('geometry-file').addEventListener('change',async e=>{
    const file=e.target.files && e.target.files[0];if(!file||busy)return;busy=true;showOverlay('正在读取本地文件',file.name,false);
    try{acceptResource(JSON.parse(await file.text()));}catch(error){showOverlay('文件无法加载',error.message+'。请选择新重建导出的 geometry.json。',true);}finally{busy=false;e.target.value='';}
  });
  document.addEventListener('keydown',e=>{if(e.key==='Escape'&&selected)clearSelection();});
  async function start() {
    try {
      const response=await fetch('../inputs/reference_assembly.json.gz',{cache:'no-cache'});
      if(!response.ok)throw new Error('源几何返回 HTTP '+response.status);
      if(typeof DecompressionStream==='undefined')throw new Error('浏览器缺少gzip解压支持，请使用当前Chrome、Edge或Firefox');
      const source=await new Response(response.body.pipeThrough(new DecompressionStream('gzip'))).json();
      source.meshes=source.meshes.map(m=>({...m,id:'cad:'+m.name,display_group:m.name.startsWith('CAD_body_skin_')?'shell':m.display_group,color:m.color||[.5,.55,.6,1]}));
      acceptResource(source);
    }catch(e){showOverlay('没有加载到实际重建网格',e.message+'。请在包根启动HTTP服务并保留inputs目录。',true);setText('render-status','加载失败 · 不使用占位模型');}
  }
  start();
})();
