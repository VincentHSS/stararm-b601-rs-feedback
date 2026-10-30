import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {STLLoader} from './vendor/STLLoader.js';
import {finite,torqueVisual} from './torque.js';

const $=id=>document.getElementById(id);
const names=['底座回转','肩部俯仰','肘部俯仰','腕部俯仰','腕部偏转','腕部旋转','夹爪'];
const colors=['#36886b','#458eb1','#cc9652','#9381bb','#c67b8b','#82999a','#6baa94'];
let token='',settings=null,boot=null,dirty=false,selected=0,latest=null,latestStatus=null;
let replay=null,replayIndex=0,replayTimer=null,history=[],lastHistory=null,busy=false,logOpen=false,networkFailed=false;
const fmt=(v,n=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(n):'—';
let forceMetric='torque_nm',forceFresh=false,forceSource='live',freshDeadline=0;
const forceElements=[],forceMarkers=[],forceAnchors=[];
function selectJoint(index){
  selected=index;
  for(const row of $('jointRows').rows){const active=Number(row.dataset.joint)===index;row.classList.toggle('selected',active);row.setAttribute('aria-selected',String(active));}
  forceElements.forEach((item,i)=>{item.label.classList.toggle('selected',i===index);item.label.setAttribute('aria-pressed',String(i===index));});
  updateDetail();highlightModel();drawTrend();
}
function notice(text,type=''){ $('notice').textContent=text;$('notice').className='notice '+type; }
async function api(path,body){
  const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json','X-RS-Token':token},body:body===undefined?undefined:JSON.stringify(body),signal:AbortSignal.timeout(4000)});
  const data=await response.json();if(!response.ok)throw new Error(data.error||`HTTP ${response.status}`);return data;
}
function markDirty(){dirty=true;$('dirty').textContent='未保存';$('dirty').className='tag replay';}
function buildTables(){
  $('jointRows').innerHTML=names.map((n,i)=>`<tr data-joint="${i}" class="${i===selected?'selected':''}"><td><div class="joint-name"><span style="background:${colors[i]}"></span><div>J${i+1}<small>${n}</small></div></div></td><td id="pos${i}">—</td><td id="cur${i}" title="RS iqf 滤波电流，约 2.86 Hz">—</td><td><div class="power"><span id="mode${i}" class="mode UNKNOWN">—</span><span id="power${i}" class="pval">—</span></div><div class="powerbar"><i id="bar${i}" style="width:0%"></i></div></td></tr>`).join('');
  $('legend').innerHTML=colors.map((c,i)=>`<span><i style="background:${c}"></i>J${i+1}</span>`).join('');
  for(const tr of $('jointRows').rows){tr.tabIndex=0;tr.setAttribute('aria-selected',String(Number(tr.dataset.joint)===selected));tr.addEventListener('click',()=>selectJoint(Number(tr.dataset.joint)));tr.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();selectJoint(Number(tr.dataset.joint));}});}
  names.forEach((name,i)=>{
    const label=document.createElement('button');label.className='force-label unavailable';label.setAttribute('aria-pressed',String(i===selected));
    label.innerHTML=`<span><span class="force-axis">J${i+1}</span><span class="force-level">等待</span></span><span><b>—</b><em>N·m</em></span>`;
    label.onclick=()=>selectJoint(i);label.title=`J${i+1} · ${name}`;$('forceLabels').appendChild(label);
    const line=document.createElementNS('http://www.w3.org/2000/svg','path');$('forceLines').appendChild(line);
    forceElements.push({label,line,value:label.querySelector('b'),level:label.querySelector('.force-level')});
  });
  document.querySelectorAll('[data-metric]').forEach(button=>button.onclick=()=>{
    forceMetric=button.dataset.metric;document.querySelectorAll('[data-metric]').forEach(item=>item.setAttribute('aria-pressed',String(item===button)));
    $('forceNote').textContent=forceMetric==='torque_nm'?'电机反馈扭矩含重力与运动影响，非末端接触力。夹爪开度按电机行程示意。':'模型剩余力矩为扣除重力与动力学估计后的残差，含摩擦及模型误差；模型无效时显示 —。';updateForces();
  });
  $('heatScale').onchange=()=>{$('scaleMax').textContent=`≥ ${$('heatScale').value} N·m`;updateForces();};
  $('showForces').onchange=()=>{const hidden=!$('showForces').checked;$('forceLabels').hidden=hidden;$('forceLines').hidden=hidden;forceMarkers.forEach(marker=>marker.visible=!hidden);};
  updateForces();
}
function buildParams(){
  $('maxPower').value=settings.max_power_mw;
  $('paramRows').innerHTML=names.map((n,i)=>`<tr><td><div class="joint-name"><span style="background:${colors[i]}"></span>J${i+1}<small>${n}${i===5?' · 普通阻尼关闭':''}</small></div></td>${['joint_power_mw',...Object.keys(boot.arrays)].map(key=>{const bounds=key==='joint_power_mw'?[0,1500]:boot.arrays[key];return `<td><input aria-label="J${i+1} ${key==='joint_power_mw'?'功率上限':boot.arrays[key][2]}" type="number" data-key="${key}" data-index="${i}" min="${bounds[0]}" max="${bounds[1]}" step="${key==='joint_power_mw'?10:.01}" value="${settings[key][i]}" ${i===5?'disabled':''}></td>`;}).join('')}</tr>`).join('');
  $('advancedFields').innerHTML=Object.entries(boot.scalars).map(([key,[lo,hi,label,unit]])=>`<label>${label}<span>${unit}</span><input type="number" data-key="${key}" min="${lo}" max="${hi}" step="${hi>100?100:.01}" value="${settings[key]}"></label>`).join('');
  for(const input of document.querySelectorAll('.params-panel input'))input.addEventListener('input',markDirty);
}
function collectSettings(){
  const next=structuredClone(settings);next.max_power_mw=Number($('maxPower').value);
  for(const input of document.querySelectorAll('[data-key]')){
    if(!input.checkValidity())throw new Error(`请检查 ${input.getAttribute('aria-label')||input.dataset.key} 的范围`);
    if(input.dataset.index!==undefined)next[input.dataset.key][Number(input.dataset.index)]=Number(input.value);
    else next[input.dataset.key]=Number(input.value);
  }
  if(!$('maxPower').checkValidity())throw new Error('总功率上限须为 400–1500 mW');return next;
}
$('save').onclick=async()=>{try{const next=collectSettings();$('save').disabled=true;settings=await api('/api/settings',next);dirty=false;$('dirty').textContent='已保存';$('dirty').className='tag';notice(latestStatus?.running?'参数已保存，等待控制循环确认生效。':'参数已保存，下次启动时生效。','success');}catch(e){notice(e.message,'error');}finally{$('save').disabled=false;}};
$('defaults').onclick=()=>{const revision=settings.revision;settings=structuredClone(boot.defaults);settings.revision=revision;buildParams();markDirty();notice('已填入默认值，点击“保存并应用”后生效。');};
$('advancedToggle').onclick=()=>{const el=$('advancedFields');el.hidden=!el.hidden;$('advancedToggle').textContent=(el.hidden?'＋':'−')+' 滤波与响应参数';};
async function start(mode){
  if(busy)return;
  if(dirty){notice('有未保存参数，请先保存或恢复已保存值。','error');return;}
  busy=true;setButtons();
  try{history=[];lastHistory=null;latest=null;notice('正在连接机械臂，请查看运行日志。');renderStatus(await api('/api/start',{mode}));}
  catch(e){notice(e.message,'error');}finally{busy=false;setButtons();}
}
$('start').onclick=()=>start('feedback');$('monitor').onclick=()=>start('monitor');
$('stop').onclick=async()=>{if(busy)return;busy=true;setButtons();try{renderStatus(await api('/api/stop',{}));notice('已发送正常停止请求，等待回零/卸力清理。');}catch(e){notice(e.message,'error');}finally{busy=false;setButtons();}};
function setButtons(){const running=latestStatus?.running;$('start').disabled=busy||running||!!replay||networkFailed;$('monitor').disabled=$('start').disabled;$('stop').disabled=busy||!running||latestStatus?.stop_requested||networkFailed;$('replay').disabled=busy||running||networkFailed;}
function explain(reason){
  if(!reason)return '等待实时数据';
  return reason.replace('RS DYNAMIC: ','').replace('RS GRIPPER: ','').replace('current moving residual (experimental)','按运动中剩余力矩更新').replace('current residual (experimental)','按当前剩余力矩更新').replace('current motor torque (experimental)','按夹爪当前电机力矩更新').replace('home-pose gravity model invalid','近零姿态模型无效').replace('this joint damping released','本轴普通阻尼已释放').replace('near joint limit','接近本轴软件限位').replace('Leader feedback gap','主臂缺帧').replace('ordinary damping released','普通阻尼已释放').replace('gripper damping released','夹爪普通阻尼已释放').replace('warming up','初始化').replace('no stationary pose required','无需停稳');
}
function updateDetail(){
  const j=latest?.joints?.[selected];$('selectedTitle').textContent=`J${selected+1} · ${names[selected]}`;
  $('selectedTemp').textContent=fmt(j?.temperature_c,1)+' °C';$('selectedReason').textContent=explain(j?.reason);
  $('selectedResidual').textContent=fmt(j?.residual_nm)+' Nm';$('selectedDeadzone').textContent=fmt(j?.deadzone_nm)+' Nm';$('selectedLeader').textContent=fmt(j?.leader_pos_deg,1)+'°';
}
function renderTelemetry(frame,source,fresh=true){
  latest=frame;forceFresh=fresh;forceSource=source;$('source').textContent=source==='replay'?'日志回放 · 非实时':fresh?'实时 CAN':'数据已过期';$('source').className='tag '+(source==='replay'?'replay':fresh?'live':'replay');
  const joints=frame.joints||[];let max=-1,axis=0,active=0;
  Array.from({length:7},(_,i)=>joints[i]||{}).forEach((j,i)=>{
    $('pos'+i).textContent=fmt(j.motor_pos_deg,1);$('cur'+i).textContent=fresh?fmt(j.current_a):'—';
    const mode=fresh?j.leader_mode:'UNKNOWN';$('mode'+i).textContent=mode==='UNKNOWN'?'过期':mode;$('mode'+i).className='mode '+mode;
    $('power'+i).textContent=fresh?fmt(j.leader_power_mw,0):'—';$('bar'+i).style.width=fresh?`${Math.min(100,(j.leader_power_mw||0)/1500*100)}%`:'0%';
    if(finite(j.torque_nm)&&Math.abs(j.torque_nm)>max){max=Math.abs(j.torque_nm);axis=i;}if(j.leader_mode==='DAMPING'&&j.leader_power_mw>0)active++;
  });
  $('activeCount').textContent=fresh?active:'—';$('outputDetail').textContent=source==='replay'?'历史请求值':frame.holding?'HOLD · 独立堵转保持':frame.output_enabled?'反馈输出已启用':'仅监测 · 未发送 Leader 阻尼';
  $('maxTorque').textContent=fmt(max<0?null:max);$('maxTorqueJoint').textContent=`J${axis+1} · ${names[axis]}`;
  updateDetail();setPose(joints);updateForces();
  if(fresh&&frame.monotonic_s!==lastHistory){
    lastHistory=frame.monotonic_s;history.push({time:frame.monotonic_s,torque:joints.map(j=>j.torque_nm)});
    history=history.filter(p=>frame.monotonic_s-p.time<=30);drawTrend();
  }
}
function clearTelemetry(){
  latest=null;forceFresh=false;updateForces();updateDetail();$('source').textContent='等待数据';$('source').className='tag';$('activeCount').textContent='—';$('outputDetail').textContent='等待反馈数据';$('maxTorque').textContent='—';$('maxTorqueJoint').textContent='RS 电机反馈值';
  for(let i=0;i<7;i++){for(const key of ['pos','cur','power'])$(key+i).textContent='—';$('mode'+i).textContent='—';$('mode'+i).className='mode UNKNOWN';$('bar'+i).style.width='0%';}
}
function renderStatus(data){
  latestStatus=data;networkFailed=false;setButtons();if(replay)return;
  freshDeadline=data.fresh?performance.now()+Math.max(0,.8-(data.age_s??0))*1000:0;
  document.body.classList.toggle('stale',!data.fresh);
  const phase={idle:'未启动',connecting:'连接中',running:'运行中',stopping:'正在停止',exited:'已结束'};
  $('session').textContent=phase[data.phase]||'未知';$('sessionDetail').textContent=data.phase==='exited'?`退出代码 ${data.exit_code}`:data.fresh&&data.telemetry?.leader_targets_paused?'主臂短暂缺帧 · 跟随目标暂停，阻尼按 RS 更新':'can0 / /dev/ttyUSB0';
  $('led').className='led '+(data.fresh?'green':data.running?'orange':'');$('connection').textContent=data.fresh?'实时数据已连接':data.running?'等待设备数据':'本地网页已连接';
  $('freshness').textContent=data.age_s===null?'—':data.fresh?`${Math.round(data.age_s*1000)} ms`:'已过期';
  const applied=data.telemetry?.applied_revision;
  $('revision').textContent=data.running&&applied!==undefined?`已应用 v${applied} / 已保存 v${data.saved_revision}${applied!==data.saved_revision?' · 等待应用':''}`:`已保存 v${data.saved_revision} · 启动后应用`;
  if(data.telemetry)renderTelemetry(data.telemetry,'live',data.fresh);else if(latest)clearTelemetry();
  if(data.telemetry?.settings_error)notice('参数未应用：'+data.telemetry.settings_error,'error');
  if(data.phase==='exited'&&data.exit_code!==0&&data.exit_code!==-2)notice(`会话已结束（退出代码 ${data.exit_code}），展开日志查看原因。`,'error');
  if(data.log_file)$('logPath').textContent=data.log_file;
}
async function poll(){try{renderStatus(await api('/api/state'));}catch(e){networkFailed=true;setButtons();document.body.classList.add('stale');$('led').className='led orange';$('connection').textContent='网页服务连接中断';$('freshness').textContent='已过期';if(latest&&!replay)renderTelemetry(latest,'live',false);notice('网页服务暂不可达；此状态不能确认机械臂已停止。','error');}setTimeout(poll,250);}
// Expire on the client even while a state request is still waiting for its timeout.
setInterval(()=>{if(!replay&&forceFresh&&performance.now()>freshDeadline){document.body.classList.add('stale');$('freshness').textContent='已过期';$('connection').textContent='等待新鲜设备数据';$('led').className='led orange';renderTelemetry(latest,'live',false);}},100);
$('replay').onclick=async()=>{
  if(replay){clearInterval(replayTimer);replay=null;history=[];lastHistory=null;drawTrend();$('replay').textContent='↻ 日志回放';notice('已退出回放，尚未连接机械臂。');renderStatus(await api('/api/state'));return;}
  try{replay=await api('/api/replay');replayIndex=0;history=[];lastHistory=null;$('replay').textContent='■ 退出回放';setButtons();notice(`正在回放 ${replay.file}，所有数值为历史数据；旧日志缺少电流时显示“—”。`);document.body.classList.remove('stale');
    $('session').textContent='日志回放';$('sessionDetail').textContent=replay.file;$('connection').textContent='历史数据 · 非实时';$('led').className='led orange';$('freshness').textContent='回放中';$('revision').textContent='历史日志参数';
    replayTimer=setInterval(()=>{if(replayIndex>=replay.frames.length){replayIndex=0;history=[];lastHistory=null;}renderTelemetry(replay.frames[replayIndex++],'replay',true);},100);
  }catch(e){replay=null;notice(e.message,'error');}
};
$('toggleLog').onclick=()=>{logOpen=!logOpen;$('log').hidden=!logOpen;$('toggleLog').textContent=logOpen?'收起':'展开';loadLog();};
async function loadLog(){if(logOpen){try{$('log').textContent=(await api('/api/log')).text||'尚无运行日志';$('log').scrollTop=$('log').scrollHeight;}catch{}}}
setInterval(loadLog,1500);
function drawTrend(){
  const canvas=$('trend'),width=canvas.clientWidth,height=canvas.clientHeight,dpr=devicePixelRatio||1;
  canvas.width=width*dpr;canvas.height=height*dpr;const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,width,height);
  let range=Math.max(1,...history.flatMap(h=>h.torque.filter(Number.isFinite).map(Math.abs)));range=Math.ceil(range*1.15*2)/2;
  const left=42,right=width-20,top=16,bottom=height-22,mid=(top+bottom)/2;
  ctx.font='9px monospace';ctx.fillStyle='#95a690';ctx.strokeStyle='#e9eee5';ctx.lineWidth=1;
  for(const value of [-range,0,range]){const y=mid-value/range*(bottom-top)/2;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillText(value.toFixed(1),7,y+3);}
  if(history.length<2){ctx.fillText('等待关节数据 · 启动监测或回放日志后显示',left+10,mid-9);return;}
  const end=history.at(-1).time,start=end-30;
  colors.forEach((color,i)=>{ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=i===selected?2:1;ctx.globalAlpha=i===selected?1:.55;let connected=false;
    for(const point of history){const value=point.torque[i];if(!Number.isFinite(value)){connected=false;continue;}const x=left+(point.time-start)/30*(right-left),y=mid-value/range*(bottom-top)/2;if(connected)ctx.lineTo(x,y);else ctx.moveTo(x,y);connected=true;}ctx.stroke();});ctx.globalAlpha=1;
}
let scene,camera,renderer,controls,robotGroup,modelJoints=[],linkGroups={},axes=[],modelReady=false;
function poseObject(obj,pose){obj.position.fromArray(pose.xyz);obj.quaternion.setFromEuler(new THREE.Euler(...pose.rpy,'ZYX'));}
function resetView(){camera.position.set(.92,-1.1,.74);controls.target.set(0,0,.27);controls.update();}
async function initModel(){
  try{
    const view=$('viewport');scene=new THREE.Scene();camera=new THREE.PerspectiveCamera(38,1,.005,20);camera.up.set(0,0,1);
    renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.2;
    view.appendChild(renderer.domElement);controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.3;controls.maxDistance=3.;controls.maxPolarAngle=Math.PI*.49;resetView();
    scene.add(new THREE.HemisphereLight(0xffffff,0xa7b49d,2.1));const sun=new THREE.DirectionalLight(0xfff9e9,3);sun.position.set(1,-1,2);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);sun.shadow.camera.left=-1;sun.shadow.camera.right=1;sun.shadow.camera.top=1;sun.shadow.camera.bottom=-1;sun.shadow.bias=-.0005;scene.add(sun);
    const rim=new THREE.DirectionalLight(0xd7eee5,2);rim.position.set(-1,.5,1);scene.add(rim);
    const grid=new THREE.GridHelper(2,40,0xc2cebb,0xdce4d5);grid.rotation.x=Math.PI/2;grid.position.z=-.001;scene.add(grid);
    const floor=new THREE.Mesh(new THREE.PlaneGeometry(5,5),new THREE.ShadowMaterial({opacity:.12}));floor.position.z=-.002;floor.receiveShadow=true;scene.add(floor);
    const model=await(await fetch('model/robot.json')).json();robotGroup=new THREE.Group();scene.add(robotGroup);
    for(const link of model.links){const group=new THREE.Group();group.name=link.name;linkGroups[link.name]=group;}
    robotGroup.add(linkGroups[model.root]);
    for(const joint of model.joints){const base=new THREE.Group(),moving=new THREE.Group();poseObject(base,joint);base.add(moving);moving.add(linkGroups[joint.child]);linkGroups[joint.parent].add(base);modelJoints.push({...joint,base,moving});
      if(joint.type==='revolute'){const helper=new THREE.AxesHelper(.055);helper.visible=false;base.add(helper);axes.push(helper);}}
    for(let i=0;i<7;i++){
      const anchor=i<6?modelJoints.find(j=>j.name===`joint${i+1}`).base:linkGroups.gripper_end;
      const marker=new THREE.Mesh(new THREE.SphereGeometry(.009,16,12),new THREE.MeshBasicMaterial({color:0x92a29e,depthTest:false,transparent:true,opacity:.85}));marker.renderOrder=5;anchor.add(marker);forceAnchors.push(anchor);forceMarkers.push(marker);
    }
    const loader=new STLLoader(),cache=new Map();let completed=0,total=model.links.reduce((n,l)=>n+l.visuals.length,0);
    const tasks=model.links.flatMap(link=>link.visuals.map(async v=>{
      if(!cache.has(v.mesh))cache.set(v.mesh,loader.loadAsync(v.mesh));const geometry=await cache.get(v.mesh);geometry.computeVertexNormals();
      const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...v.color.slice(0,3)),metalness:.35,roughness:.58});const mesh=new THREE.Mesh(geometry,material);mesh.userData.baseColor=material.color.clone();mesh.castShadow=true;mesh.receiveShadow=true;mesh.scale.fromArray(v.scale);poseObject(mesh,v);linkGroups[link.name].add(mesh);$('modelLoading').textContent=`正在加载本地模型 ${++completed}/${total}`;
    }));await Promise.all(tasks);modelReady=true;$('modelLoading').style.display='none';if(latest)setPose(latest.joints);updateForces();
    const resize=()=>{const w=view.clientWidth,h=view.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();drawTrend();};new ResizeObserver(resize).observe(view);resize();
    $('resetView').onclick=resetView;$('showAxes').onchange=()=>axes.forEach(a=>a.visible=$('showAxes').checked);
    const animate=()=>{requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);positionForceLabels();};animate();view.classList.add('model-ready');
  }catch(e){$('modelLoading').textContent='模型加载失败：'+e.message;console.error(e);}
}
function setPose(joints){if(!modelReady)return;for(const j of modelJoints){if(j.type==='revolute'){const index=Number(j.name.replace('joint',''))-1;const angle=(joints[index]?.motor_pos_deg??0)*Math.PI/180;j.moving.quaternion.setFromAxisAngle(new THREE.Vector3(...j.axis),angle);}else if(j.type==='prismatic'){const opening=THREE.MathUtils.clamp((joints[6]?.motor_pos_deg??0)/270,0,1)*.05;j.moving.position.copy(new THREE.Vector3(...j.axis).multiplyScalar(opening));}}}
function highlightModel(){updateForces();}
function updateForces(){
  const scale=Number($('heatScale').value),options={fresh:forceFresh,source:forceSource,metric:forceMetric,scale};
  $('forceStatus').textContent=!latest?'等待关节数据':forceSource==='replay'?'历史回放 · 非实时受力':forceFresh?'实时受力 · N·m':'数据过期 · 保留最后姿态';
  forceElements.forEach((item,i)=>{
    const visual=torqueVisual(latest?.joints?.[i],options),active=selected===i,available=visual.value!==null;
    item.label.style.setProperty('--heat',visual.color);item.line.style.setProperty('--heat',visual.color);item.value.textContent=fmt(visual.value);
    item.level.textContent=!latest?'等待':!forceFresh?'已过期':available?`${visual.level} · ${Math.round(visual.ratio*100)}%`:'无效 / 缺失';
    item.label.classList.toggle('unavailable',!available);item.label.classList.toggle('selected',active);
    item.label.setAttribute('aria-label',`J${i+1} ${names[i]}，${forceSource==='replay'?'历史回放，':''}${forceMetric==='torque_nm'?'电机扭矩':'模型剩余力矩'} ${available?fmt(visual.value)+' N·m':!forceFresh&&latest?'数据过期':'暂无有效数据'}`);
    if(!modelReady)return;
    forceMarkers[i].material.color.set(visual.color);forceMarkers[i].scale.setScalar(active?1.35:1);forceMarkers[i].visible=$('showForces').checked;
    const links=i<6?[`link${i+1}`]:['gripper_end','gripper_left','gripper_right'];
    for(const name of links)for(const mesh of linkGroups[name].children){
      if(!mesh.userData.baseColor)continue;
      mesh.material.color.copy(mesh.userData.baseColor).lerp(new THREE.Color(visual.color),available ? .15+visual.ratio*.35 : .08);
      mesh.material.emissive.set(available?visual.color:0x000000);mesh.material.emissiveIntensity=available ? (active ? .22 : .1) : 0;
    }
  });
}
const projectedPoint=new THREE.Vector3();
function positionForceLabels(){
  if(!modelReady||!$('showForces').checked)return;
  const width=$('viewport').clientWidth,height=$('viewport').clientHeight,labelWidth=forceElements[0].label.offsetWidth,labelHeight=forceElements[0].label.offsetHeight;
  const projected=forceAnchors.map((anchor,i)=>{anchor.getWorldPosition(projectedPoint);projectedPoint.project(camera);return {i,x:(projectedPoint.x+1)*width/2,y:(1-projectedPoint.y)*height/2,visible:Math.abs(projectedPoint.z)<=1&&Math.abs(projectedPoint.x)<=1&&Math.abs(projectedPoint.y)<=1};});
  for(const side of [[0,1,2],[3,4,5,6]]){
    const items=side.map(i=>projected[i]).sort((a,b)=>a.y-b.y),gap=labelHeight+9,minY=94,maxY=height-43-labelHeight;
    const positions=items.map((point,index)=>Math.max(minY+index*gap,Math.min(maxY,point.y-labelHeight/2)));
    for(let k=1;k<positions.length;k++)positions[k]=Math.max(positions[k],positions[k-1]+gap);
    if(positions.at(-1)>maxY){positions[positions.length-1]=maxY;for(let k=positions.length-2;k>=0;k--)positions[k]=Math.min(positions[k],positions[k+1]-gap);}
    items.forEach((point,index)=>{
      const left=side[0]===0,x=left?14:width-labelWidth-14,y=positions[index],item=forceElements[point.i];
      item.label.style.transform=`translate(${x}px,${y}px)`;item.line.style.display=point.visible?'':'none';
      const fromX=left?x+labelWidth:x,fromY=y+labelHeight/2,elbow=left?fromX+12:fromX-12;
      item.line.setAttribute('d',`M ${fromX} ${fromY} L ${elbow} ${fromY} L ${point.x} ${point.y}`);
    });
  }
}
async function init(){
  buildTables();drawTrend();new ResizeObserver(drawTrend).observe($('trend'));initModel();
  try{boot=await api('/api/bootstrap');token=boot.token;settings=boot.settings;$('version').textContent='v'+boot.version;buildParams();poll();}
  catch(e){networkFailed=true;setButtons();notice('网页初始化失败：'+e.message,'error');}
}
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});init();
