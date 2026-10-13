(function(){'use strict';
const UI=window.GTSDTruthEvidence,box=document.getElementById('truth-evidence-panel'),aux=document.getElementById('phase6-observations-panel');
if(!UI||!box||!aux)return;
let stopped=false,revision=0,query='',widget=null;
const status=document.createElement('p');status.id='truth-refresh-status';status.setAttribute('role','status');box.before(status);
async function read(path){const response=await fetch(path,{cache:'no-store'});const data=await response.json();if(!response.ok)throw new Error(data.error||'HTTP '+response.status);return data;}
async function refresh(nextQuery){query=nextQuery===undefined?(widget?.queryInput.value||query):nextQuery;const request=++revision;status.textContent='正在独立读取证据；不改变求解器。';try{const data=await read('/api/truth-evidence');if(stopped||request!==revision)return;widget=UI.mount(box,data,{document,query,onRefresh:refresh});status.textContent='证据已读取；几何/照片/型号的候选状态仍独立保留。';}catch(error){if(stopped||request!==revision)return;status.textContent='证据刷新失败，之前显示不代表当前资料：'+error.message;}}
async function pollAuxiliary(){if(stopped)return;try{const data=await read('/api/phase6-observations');if(!stopped)UI.mountAuxiliary(aux,data,{document});}catch(error){if(!stopped&&!aux.hidden){aux.replaceChildren();const p=document.createElement('p');p.textContent='辅机观察读取失败；不外推、不把旧值当当前值：'+error.message;aux.append(p);}}finally{if(!stopped)setTimeout(pollAuxiliary,1000);}}
window.addEventListener('pagehide',()=>{stopped=true;revision++;});refresh();pollAuxiliary();
})();
