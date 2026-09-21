import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import './style.css';

type J = Record<string, any>;
type Selection = {kind:'vertex'|'edge', id:string};
const $ = <T extends HTMLElement>(id:string) => document.getElementById(id) as T;
const logBox=$<HTMLDivElement>('logs'), toastBox=$<HTMLDivElement>('toast');
let requestSequence=0;
const requestId=()=>globalThis.crypto?.randomUUID?.() ??
  `${Date.now().toString(36)}-${(++requestSequence).toString(36)}-${Math.random().toString(36).slice(2,10)}`;
const send=(type:string,data:J={})=>{
  if(!ws||ws.readyState!==WebSocket.OPEN){toast('WebSocket 未连接',true);return}
  ws.send(JSON.stringify({type,request_id:requestId(),...data}));
};
const toast=(s:string,error=false)=>{toastBox.textContent=s;toastBox.className=`show ${error?'error':''}`;setTimeout(()=>toastBox.className='',2600)};
const log=(s:string)=>{const t=new Date().toLocaleTimeString();logBox.textContent+=`[${t}] ${s}\n`;logBox.scrollTop=logBox.scrollHeight};

let ws:WebSocket, clientId='';
let topology:any=null, revision='', dirty=false;
let pcdFiles:any[]=[], topologyFiles:any[]=[], poseSources:any[]=[];
let selectedPcd='', selectedTopology='';
// The loaded resource and the user's pending dropdown choice are intentionally separate.
// Periodic backend snapshots update selectedPcd/selectedTopology, but must not overwrite
// a new choice the user has made before pressing the corresponding load button.
let pcdChoiceDirty=false, topologyChoiceDirty=false;
let mapVoxelMin=0.01,mapVoxelMax=2.0;
let selected:Selection|null=null, tool='select', edgeStart:string|null=null;
let currentPose:any=null; const undoStack:string[]=[], redoStack:string[]=[];
// WebSocket 回调只保存最新遥测值；Three.js 和 DOM 统一在动画帧里更新，
// 避免 200 Hz 里程计把浏览器主线程事件队列堵住后显示旧位置。
let pendingPoseRender=false,pendingVelocity:any=null;
let navigationTarget:any=null,lastTrajectorySignature='',lastStatusRender=0;
let lastSelectedPcd='',lastSelectedTopology='';
let lastRecordingState='idle';
const mapChunks=new Map<number,Float32Array[]>();

function connect(){
  ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);ws.binaryType='arraybuffer';
  ws.onopen=()=>{$('wsLamp').classList.add('ok');$('wsText').textContent='已连接';send('resource.list');send('cloud.live.set',{enabled:$<HTMLInputElement>('showLive').checked})};
  ws.onclose=()=>{$('wsLamp').classList.remove('ok');$('wsText').textContent='连接断开，正在重连';setTimeout(connect,1500)};
  ws.onerror=()=>{};
  ws.onmessage=e=>typeof e.data==='string'?handle(JSON.parse(e.data)):handleCloud(e.data);
}

function handle(m:J){
  if(m.type==='hello'){clientId=m.client_id;return}
  if(m.type==='ack'){if(m.message!=='heartbeat'){toast(m.message,!m.success);log(`${m.success?'✓':'✗'} ${m.message}`)}return}
  if(m.type==='error'){toast(m.message,true);log(`错误: ${m.message}`);return}
  if(m.type==='process.log'){log(`[${m.process}] ${m.line}`);if(m.process==='map_save'&&String(m.line).includes('[process exited with code 0]'))setTimeout(()=>send('resource.list'),350);return}
  if(m.type==='resource.catalog'){
    pcdFiles=m.pcds||[];topologyFiles=m.topologies||[];poseSources=m.pose_sources||[];
    if('selected_pcd'in m)selectedPcd=m.selected_pcd||'';
    if('selected_topology'in m)selectedTopology=m.selected_topology||'';
    if(Number.isFinite(Number(m.map_voxel_min_m)))mapVoxelMin=Number(m.map_voxel_min_m);
    if(Number.isFinite(Number(m.map_voxel_max_m)))mapVoxelMax=Number(m.map_voxel_max_m);
    const voxelInput=$<HTMLInputElement>('mapVoxelSize');voxelInput.min=String(mapVoxelMin);voxelInput.max=String(mapVoxelMax);
    if(Number.isFinite(Number(m.map_voxel_m)))voxelInput.value=Number(m.map_voxel_m).toFixed(2);
    renderResourceCatalog();return;
  }
  if(m.type==='resource.selection'){
    if('selected_pcd'in m)selectedPcd=m.selected_pcd||'';
    if('selected_topology'in m)selectedTopology=m.selected_topology||'';
    renderResourceCatalog();return;
  }
  if(m.type==='map.cloud.clear'){clearGroup(mapGroup);mapChunks.clear();return}
  if(m.type==='topology.snapshot'||m.type==='topology.saved'){
    const keepSelection=m.type==='topology.saved'?selected:null;
    topology=m.data;revision=m.revision;selectedTopology=m.path||m.id||selectedTopology;dirty=false;
    undoStack.length=0;redoStack.length=0;selected=keepSelection;edgeStart=null;
    renderTopology();renderResourceCatalog();setDirty();
    if(m.type==='topology.saved')toast('拓扑属性已写入 JSON');
    return;
  }
  if(m.type==='topology.preview'){renderTopologyPreview(m.data);return}
  if(m.type==='topology.preview.clear'){clearGroup(previewGroup);return}
  if(m.type==='topology.record.saved'){toast(`拓扑 ${m.name} 已保存并加载`);log(`新拓扑文件：${m.path}`);clearGroup(previewGroup);send('resource.list');return}
  if(m.type==='topology.generated'){toast(`拓扑 ${m.name} 已生成并加载`);log(`轨迹生成拓扑：${m.vertices} 点 / ${m.edges} 边，${m.path}`);return}
  if(m.type==='pose'){currentPose=m;pendingPoseRender=true;return}
  if(m.type==='navigation.target'){navigationTarget=m;renderGoalAccuracy();return}
  if(m.type==='velocity'){pendingVelocity=m;return}
  if(m.type==='ros.status'){
    if(m.source==='active_controller')$('controller').textContent=typeof m.data==='string'?m.data:(m.data?.data||'none');
    return;
  }
  if(m.type==='snapshot'){
    const f=m.features||{};
    updateFeature('radar',f.radar,m.processes?.radar);
    updateFeature('mapping',f.mapping,m.processes?.mapping);
    updateFeature('localization',f.localization,m.processes?.localization);
    updateFeature('planner',f.planner,m.processes?.planner);
    updateFeature('initializer',f.vertex_initializer,null);
    const loopFeature=f.loop_patrol||{},loopProcess=m.processes?.loop_patrol||{};
    const loopOnline=!!loopFeature.online||!!loopProcess.running,loopManaged=!!loopFeature.managed||!!loopProcess.running;
    const loopState=$('loopPatrolState');loopState.textContent=loopOnline?(loopManaged?'运行中':'ROS在线'):'未运行';loopState.classList.toggle('running',loopOnline&&loopManaged);loopState.classList.toggle('external',loopOnline&&!loopManaged);
    const saveService=$('mapSaveServiceState');const serviceOnline=!!f.mapping?.save_service;saveService.textContent=serviceOnline?'在线':'离线';saveService.classList.toggle('running',serviceOnline);saveService.classList.toggle('external',false);
    const saveState=$('mapSaveState');const saving=!!m.processes?.map_save?.running;saveState.textContent=saving?'保存中':'空闲';saveState.classList.toggle('running',saving);
    const generationNames:J={idle:'空闲',running:'生成中',saved:'已生成',error:'失败'};const generationState=String(m.topology_generation_state||'idle');const generation=$('topologyGenerationState');generation.textContent=generationNames[generationState]||generationState;generation.classList.toggle('running',generationState==='running'||generationState==='saved');
    updateRecording(m.topology_recording_state,!!f.topology_recording?.online);
    if(m.selected_pcd)selectedPcd=m.selected_pcd;if(m.selected_topology)selectedTopology=m.selected_topology;
    if(m.navigation_target){navigationTarget=m.navigation_target;renderGoalAccuracy()}
    if(m.pose){currentPose=m.pose;pendingPoseRender=true}if(m.trajectory)renderTrajectory(m.trajectory);
    const voxelInput=$<HTMLInputElement>('mapVoxelSize');
    if(document.activeElement!==voxelInput&&Number.isFinite(Number(m.map_voxel_m)))voxelInput.value=Number(m.map_voxel_m).toFixed(2);
    if(selectedPcd!==lastSelectedPcd||selectedTopology!==lastSelectedTopology){lastSelectedPcd=selectedPcd;lastSelectedTopology=selectedTopology;renderResourceCatalog()}
    const now=performance.now();if(now-lastStatusRender>=1000){lastStatusRender=now;$('statusJson').textContent=JSON.stringify({features:m.features,processes:m.processes,topics:m.topics,ros:m.ros,selectedPcd,selectedTopology,mapVoxelM:Number(voxelInput.value)},null,2)}return;
  }
  if(m.type==='map.cloud_ready'){log(`地图点云已加载：${m.points} 点，体素 ${m.voxel_m}m`)}
}

function setModuleStatus(module:string,text:string,stateClass=''){const el=document.getElementById(`${module}ModuleState`);if(!el)return;el.textContent=text;el.className=stateClass}
function updateFeature(name:string,feature:any,process:any){const el=$(`${name}State`);if(!el)return;const online=!!feature?.online||!!process?.running;const managed=!!feature?.managed||!!process?.running;const text=online?(managed?'运行中':'ROS在线'):'未运行';el.textContent=text;el.classList.toggle('running',online&&managed);el.classList.toggle('external',online&&!managed);const module=name==='planner'?'navigation':name;if(['radar','mapping','localization','navigation'].includes(module))setModuleStatus(module,text,online?(managed?'running':'external'):'')}
function activateModule(module:string){if(module==='navigation'&&tool==='select'){tool='navigate';edgeStart=null;setToolButton()}else if(module!=='navigation'&&tool==='navigate'){tool='select';edgeStart=null;setToolButton()}document.querySelectorAll<HTMLElement>('[data-module]').forEach(x=>x.classList.toggle('active-module',x.dataset.module===module));document.querySelectorAll<HTMLElement>('[data-module-tab]').forEach(x=>x.classList.toggle('active',x.dataset.moduleTab===module));document.body.classList.toggle('module-topology',module==='topology')}

function optionLabel(path:string){const parts=path.split('/').filter(Boolean);return parts.slice(-3).join('/')||path}
function fillPathSelect(id:string,items:any[],loadedPath:string,placeholder:string,preservePending=false){
  const select=$<HTMLSelectElement>(id),current=select.value;
  select.innerHTML=`<option value="">${placeholder}</option>`;
  for(const item of items){const o=document.createElement('option');o.value=item.path;o.textContent=item.label||optionLabel(item.path);o.title=item.path;select.append(o)}
  // While the user is choosing another resource, preserve that pending choice across
  // resource.catalog/snapshot refreshes. Only fall back to the loaded path when there
  // is no pending user choice (initial page load, successful switch, or invalid option).
  const pendingValid=preservePending&&current&&[...select.options].some(o=>o.value===current);
  const wanted=pendingValid?current:(loadedPath||current);
  if([...select.options].some(o=>o.value===wanted))select.value=wanted;
}
function reconcilePendingChoices(){
  const pcd=$<HTMLSelectElement>('pcdSelect'),topo=$<HTMLSelectElement>('topologySelect');
  if(pcdChoiceDirty&&pcd.value===selectedPcd)pcdChoiceDirty=false;
  if(topologyChoiceDirty&&topo.value===selectedTopology)topologyChoiceDirty=false;
}
function renderResourceCatalog(){
  reconcilePendingChoices();
  fillPathSelect('pcdSelect',pcdFiles,selectedPcd,'请选择本地 PCD',pcdChoiceDirty);
  fillPathSelect('topologySelect',topologyFiles,selectedTopology,'请选择本地拓扑 JSON',topologyChoiceDirty);
  const poseSelect=$<HTMLSelectElement>('poseSourceSelect'),poseCurrent=poseSelect.value;
  poseSelect.innerHTML='<option value="">请选择含 pose.json 的地图</option>';
  for(const item of poseSources){const option=document.createElement('option');option.value=item.id;option.textContent=`${item.label}${item.generated?'（已有拓扑）':''}`;option.title=item.path;poseSelect.append(option)}
  if([...poseSelect.options].some(option=>option.value===poseCurrent))poseSelect.value=poseCurrent;
  renderPoseSourcePath();
  $('selectedPcdPath').textContent=selectedPcd?`已加载：${selectedPcd}`:'未加载 PCD';
  $('selectedTopologyPath').textContent=selectedTopology?`已加载：${selectedTopology}`:'未加载拓扑';
  setModuleStatus('topology',selectedTopology?'已加载':'未加载',selectedTopology?'running':'');
}
function renderPoseSourcePath(){
  const id=$<HTMLSelectElement>('poseSourceSelect').value,item=poseSources.find(source=>source.id===id);
  $('poseSourcePath').textContent=item?`输入：${item.path}\n输出：${item.output_path}`:'未选择轨迹';
}
function updateRecording(state='idle',running=false){
  const names:J={idle:'未运行',starting:'启动中',recording:'记录中',processing:'处理中',saved:'已保存',aborted:'已放弃',error:'失败'};
  const el=$('recordingState');el.textContent=names[state]||state;el.classList.toggle('running',running||state==='processing');setModuleStatus('recording',names[state]||state,(running||state==='processing')?'running':'');
  const active=state==='starting'||state==='recording'||state==='processing';const wasActive=lastRecordingState==='starting'||lastRecordingState==='recording'||lastRecordingState==='processing';
  topoGroup.visible=!active&&$<HTMLInputElement>('showTopo').checked;if(active&&!wasActive){selected=null;renderInspector();refreshTopologySelectionStyles()}if(!active&&wasActive)renderTopology();lastRecordingState=state;
}

const scene=new THREE.Scene();scene.background=new THREE.Color(0x050b0f);scene.fog=new THREE.FogExp2(0x050b0f,0.012);
const camera=new THREE.PerspectiveCamera(55,1,.001,100000);camera.position.set(8,-10,9);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));$('viewport').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,0);controls.enableDamping=true;
// 大地图允许以鼠标指针所在位置持续放大，避免只能围绕地图中心缩放而产生“到达上限”的感觉。
controls.zoomToCursor=true;controls.zoomSpeed=1.35;controls.minDistance=.005;controls.maxDistance=Infinity;
scene.add(new THREE.GridHelper(100,100,0x23515d,0x122b34).rotateX(Math.PI/2));scene.add(new THREE.AxesHelper(1.5));
const mapGroup=new THREE.Group(),liveGroup=new THREE.Group(),topoGroup=new THREE.Group(),previewGroup=new THREE.Group(),trackGroup=new THREE.Group();scene.add(mapGroup,liveGroup,topoGroup,previewGroup,trackGroup);
const robot=new THREE.Group();const body=new THREE.Mesh(new THREE.BoxGeometry(.55,.32,.16),new THREE.MeshBasicMaterial({color:0x43f1c6}));body.position.z=.18;robot.add(body);const nose=new THREE.Mesh(new THREE.ConeGeometry(.12,.35,8),new THREE.MeshBasicMaterial({color:0xffffff}));nose.rotation.z=-Math.PI/2;nose.position.x=.42;nose.position.z=.18;robot.add(nose);scene.add(robot);
const raycaster=new THREE.Raycaster();raycaster.params.Points!.threshold=.16;const pointer=new THREE.Vector2();
const interactionPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);let dragStart:THREE.Vector3|null=null,yawArrow:THREE.ArrowHelper|null=null;let draggingVertex:{id:string,object:THREE.Object3D}|null=null,dragMoved=false;
let followRobot=false,followPosition:THREE.Vector3|null=null;

function setRobotFollow(enabled:boolean){if(enabled&&!currentPose){toast('尚无机器人位置，无法开启跟随',true);return}followRobot=enabled;controls.enablePan=!enabled;followPosition=null;const button=$<HTMLButtonElement>('followRobot');button.textContent=enabled?'停止跟随':'跟随机器人';button.classList.toggle('active',enabled);button.setAttribute('aria-pressed',String(enabled));if(enabled){updateFollowCamera(currentPose.position);toast('已跟随机器人中心，可旋转和缩放视角')}}
function updateFollowCamera(position:number[]){if(!followRobot)return;const next=new THREE.Vector3(position[0],position[1],position[2]);if(followPosition){const delta=next.clone().sub(followPosition);camera.position.add(delta);controls.target.add(delta)}else{const offset=camera.position.clone().sub(controls.target);controls.target.copy(next);camera.position.copy(next).add(offset)}followPosition=next;controls.update()}
function resize(){const box=$('viewport').getBoundingClientRect();camera.aspect=box.width/box.height;camera.updateProjectionMatrix();renderer.setSize(box.width,box.height,false)}new ResizeObserver(resize).observe($('viewport'));
let lastRenderAt=0;
function animate(now=0){
  requestAnimationFrame(animate);if(now-lastRenderAt<33)return;lastRenderAt=now;
  if(pendingPoseRender){pendingPoseRender=false;renderPose()}
  if(pendingVelocity){const value=pendingVelocity;pendingVelocity=null;$('velocity').textContent=`${Number(value.vx).toFixed(2)} / ${Number(value.wz).toFixed(2)}`}
  controls.update();renderer.render(scene,camera)
}animate();
function clearGroup(g:THREE.Group){for(const o of [...g.children]){g.remove(o);const x=o as any;x.geometry?.dispose();const disposeMaterial=(m:any)=>{m?.map?.dispose?.();m?.dispose?.()};if(Array.isArray(x.material))x.material.forEach(disposeMaterial);else disposeMaterial(x.material)}}

function handleCloud(buffer:ArrayBuffer){
  if(buffer.byteLength<60)return;const d=new DataView(buffer);if(String.fromCharCode(...new Uint8Array(buffer,0,4))!=='R3PC')return;
  const stream=d.getUint8(5),flags=d.getUint16(6,true),seq=d.getUint32(8,true),count=d.getUint32(20,true),stride=d.getUint16(24,true),frameLen=d.getUint16(26,true),chunk=d.getUint32(28,true),chunks=d.getUint32(32,true);let off=60+frameLen;
  const xyz=new Float32Array(count*3);for(let i=0;i<count;i++){xyz[i*3]=d.getFloat32(off,true);xyz[i*3+1]=d.getFloat32(off+4,true);xyz[i*3+2]=d.getFloat32(off+8,true);off+=stride}
  if(stream===1){if(!mapChunks.has(seq))mapChunks.set(seq,[]);mapChunks.get(seq)![chunk]=xyz;if(mapChunks.get(seq)!.filter(Boolean).length===chunks){const arrays=mapChunks.get(seq)!;const total=arrays.reduce((n,a)=>n+a.length,0);const all=new Float32Array(total);let p=0;arrays.forEach(a=>{all.set(a,p);p+=a.length});mapChunks.clear();setPoints(mapGroup,all,0xd6f3ff,.025)}}
  else setPoints(liveGroup,xyz,stream===2?0x43f1c6:0xf2b84b,.045);
}
function setPoints(group:THREE.Group,positions:Float32Array,color:number,size:number){
  let pts=group.children[0] as THREE.Points|undefined;
  if(!pts?.isPoints){clearGroup(group);const geo=new THREE.BufferGeometry();pts=new THREE.Points(geo,new THREE.PointsMaterial({color,size,sizeAttenuation:true,transparent:true,opacity:.9}));pts.frustumCulled=group===mapGroup;group.add(pts)}
  const geo=pts.geometry,old=geo.getAttribute('position') as THREE.BufferAttribute|undefined;
  if(old&&old.array.length===positions.length){(old.array as Float32Array).set(positions);old.needsUpdate=true}else geo.setAttribute('position',new THREE.BufferAttribute(positions,3));
  if(group===mapGroup)geo.computeBoundingSphere();
}
function yawFromQuaternion(q:number[]){return Math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))}
function wrappedAngle(value:number){return Math.atan2(Math.sin(value),Math.cos(value))}
function chooseNavigationTarget(vertexId:number){
  const vertex=topology?.vertices?.[String(vertexId)];if(!vertex)return false;
  navigationTarget={vertex_id:vertexId,position:[...vertex.pos],yaw:Number(vertex.rpy?.[2]||0),frame_id:topology.frame_id||'map'};renderGoalAccuracy();return true;
}
function sendGoal(vertexId:number){chooseNavigationTarget(vertexId);send('navigation.goal',{goal_id:vertexId})}
function renderGoalAccuracy(){
  const position=navigationTarget?.position,yaw=Number(navigationTarget?.yaw);
  if(!Array.isArray(position)||position.length<3||!Number.isFinite(yaw)){$('accuracyTargetId').textContent='未选择';$('accuracyTargetXYZ').textContent='--';$('accuracyTargetYaw').textContent='--';$('accuracyCurrentXYZ').textContent='--';$('accuracyCurrentYaw').textContent='--';$('accuracyPositionError').textContent='--';$('accuracyYawError').textContent='--';$('accuracyDetail').textContent='发送目标后实时计算，位置误差为目标减当前。';return}
  $('accuracyTargetId').textContent=`点 ${navigationTarget.vertex_id}`;$('accuracyTargetXYZ').textContent=position.map((v:number)=>Number(v).toFixed(3)).join(' / ');$('accuracyTargetYaw').textContent=`${(yaw*180/Math.PI).toFixed(2)}°`;
  if(!currentPose?.position||!currentPose?.orientation){$('accuracyCurrentXYZ').textContent='等待定位';$('accuracyCurrentYaw').textContent='等待定位';$('accuracyPositionError').textContent='--';$('accuracyYawError').textContent='--';return}
  const current=currentPose.position,currentYaw=yawFromQuaternion(currentPose.orientation),dx=position[0]-current[0],dy=position[1]-current[1],dz=position[2]-current[2],xy=Math.hypot(dx,dy),xyz=Math.hypot(dx,dy,dz),yawError=wrappedAngle(yaw-currentYaw);
  $('accuracyCurrentXYZ').textContent=current.map((v:number)=>Number(v).toFixed(3)).join(' / ');$('accuracyCurrentYaw').textContent=`${(currentYaw*180/Math.PI).toFixed(2)}°`;$('accuracyPositionError').textContent=`3D ${xyz.toFixed(3)} m / XY ${xy.toFixed(3)} m`;$('accuracyYawError').textContent=`${(yawError*180/Math.PI).toFixed(2)}°`;$('accuracyDetail').textContent=`目标−当前：dx ${dx.toFixed(3)} m，dy ${dy.toFixed(3)} m，dz ${dz.toFixed(3)} m；Yaw 误差已归一化到 ±180°。`;
}
function renderPose(){if(!currentPose)return;const p=currentPose.position,q=currentPose.orientation;robot.position.set(p[0],p[1],p[2]);robot.quaternion.set(q[0],q[1],q[2],q[3]);updateFollowCamera(p);$('poseX').textContent=p[0].toFixed(2);$('poseY').textContent=p[1].toFixed(2);const yaw=yawFromQuaternion(q);$('poseYaw').textContent=`${(yaw*180/Math.PI).toFixed(1)}°`;renderGoalAccuracy()}
function renderTrajectory(points:number[][]){const last=points.at(-1),first=points[0],signature=points.length<2?'empty':`${points.length}:${first?.join(',')}:${last?.join(',')}`;if(signature===lastTrajectorySignature)return;lastTrajectorySignature=signature;clearGroup(trackGroup);if(points.length<2)return;const geo=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p)));trackGroup.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x4ad7ff})))}

const obstacleModeStyle=[
  {color:0xff5b57,label:'0 停障 / PID'},
  {color:0x25d796,label:'1 绕障 / Efficient 3D'},
  {color:0x4da3ff,label:'2 PID / 关闭普通停障'},
  {color:0x8a9aa5,label:'3 兼容模式'},
  {color:0xbc73ff,label:'4 外部栅格交接'},
];
function edgeModeStyle(edge:any){const mode=Number(edge?.meta?.obstacleMode);return obstacleModeStyle[mode]||{color:0x3ab4bb,label:`${Number.isFinite(mode)?mode:'?'} 未知模式`}}
function edgeObject(a:number[],b:number[],id:string,edge:any){
  const va=new THREE.Vector3(...a),vb=new THREE.Vector3(...b),delta=vb.clone().sub(va),len=delta.length();
  const style=edgeModeStyle(edge),mesh=new THREE.Mesh(new THREE.CylinderGeometry(.04,.04,Math.max(len,.001),8),new THREE.MeshBasicMaterial({color:style.color}));
  mesh.position.copy(va.clone().add(vb).multiplyScalar(.5));mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),delta.normalize());
  mesh.userData={kind:'edge',id,baseColor:style.color};return mesh;
}
function vertexLabel(id:string,pos:number[]){
  const canvas=document.createElement('canvas');canvas.width=128;canvas.height=64;const ctx=canvas.getContext('2d')!;ctx.clearRect(0,0,128,64);ctx.fillStyle='rgba(4,14,19,.82)';ctx.beginPath();ctx.roundRect(8,8,112,48,10);ctx.fill();ctx.strokeStyle='rgba(160,224,240,.75)';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#eaf8ff';ctx.font='bold 30px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(id,64,33);const texture=new THREE.CanvasTexture(canvas);texture.minFilter=THREE.LinearFilter;const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false}));sprite.position.set(pos[0],pos[1],pos[2]+.28);sprite.scale.set(.55,.275,1);sprite.renderOrder=10;sprite.userData={label:true};return sprite;
}
function vertexYawArrow(vertex:any){
  const yaw=Number(vertex?.rpy?.[2]);if(!Number.isFinite(yaw))return null;
  const origin=new THREE.Vector3(Number(vertex.pos[0]),Number(vertex.pos[1]),Number(vertex.pos[2])+.08);
  const direction=new THREE.Vector3(Math.cos(yaw),Math.sin(yaw),0);
  // 使用不在路径色板中的高亮荧光黄，并加长箭身/箭头，远距离也能看清朝向。
  const arrow=new THREE.ArrowHelper(direction,origin,.85,0xffff00,.25,.16);
  arrow.userData={yawIndicator:true};arrow.visible=$<HTMLInputElement>('showVertexYaw').checked;return arrow;
}
function renderTopology(){
  clearGroup(topoGroup);if(!topology){selected=null;renderInspector();return}
  const vertices=topology.vertices||{};
  for(const [id,e] of Object.entries<any>(topology.edges||{})){const a=vertices[String(e.v[0])],b=vertices[String(e.v[1])];if(a&&b)topoGroup.add(edgeObject(a.pos,b.pos,id,e))}
  for(const [id,v] of Object.entries<any>(vertices)){const base=v.meta?.isSlope?0xff8a4c:v.meta?.isCorner?0xffd84d:v.meta?.isJunction?0xc47cff:0x55aaff;const mesh=new THREE.Mesh(new THREE.SphereGeometry(.13,14,10),new THREE.MeshBasicMaterial({color:base}));mesh.position.set(...v.pos);mesh.userData={kind:'vertex',id,baseColor:base};topoGroup.add(mesh);topoGroup.add(vertexLabel(id,v.pos));const yawArrow=vertexYawArrow(v);if(yawArrow)topoGroup.add(yawArrow)}
  if(selected&&(selected.kind==='vertex'&&!vertices[selected.id]||selected.kind==='edge'&&!topology.edges?.[selected.id]))selected=null;
  refreshTopologySelectionStyles();renderInspector();
}
function refreshTopologySelectionStyles(){
  for(const obj of topoGroup.children){const u=obj.userData as any;if(!u.kind)continue;const material=(obj as THREE.Mesh).material as THREE.MeshBasicMaterial;const isSelected=!!selected&&selected.kind===u.kind&&selected.id===u.id;const isEdgeStart=u.kind==='vertex'&&edgeStart===u.id;
    material.color.setHex(isSelected?0xffffff:isEdgeStart?0x48ff88:u.baseColor);if(u.kind==='vertex')obj.scale.setScalar(isSelected||isEdgeStart?1.65:1);else obj.scale.set(isSelected?2.2:1,1,isSelected?2.2:1);
  }
}
function renderTopologyPreview(data:any){clearGroup(previewGroup);const vertices=data?.vertices||{};for(const v of Object.values<any>(vertices)){const color=v.meta?.isSlope?0xffd24a:v.meta?.isCorner?0xff5151:0x43a7ff;const mesh=new THREE.Mesh(new THREE.SphereGeometry(.10,10,8),new THREE.MeshBasicMaterial({color}));mesh.position.set(...v.pos);previewGroup.add(mesh)}for(const e of Object.values<any>(data?.edges||{})){const a=vertices[String(e.v[0])],b=vertices[String(e.v[1])];if(!a||!b)continue;const source=String(e.meta?.source||'');const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a.pos),new THREE.Vector3(...b.pos)]),new THREE.LineBasicMaterial({color:source.includes('loop')?0xff8a32:0x43e58a}));previewGroup.add(line)}}
function renderInspector(){
  const none=$('emptyInspector'),vf=$<HTMLFormElement>('vertexForm'),ef=$<HTMLFormElement>('edgeForm'),summary=$('selectionSummary');none.hidden=!!selected;vf.hidden=selected?.kind!=='vertex';ef.hidden=selected?.kind!=='edge';summary.hidden=!selected;if(!selected||!topology){summary.textContent='';return}
  if(selected.kind==='vertex'){
    const v=topology.vertices[selected.id],m=v.meta||{};$('vertexId').textContent=selected.id;
    setForm(vf,{x:v.pos[0],y:v.pos[1],z:v.pos[2],roll:v.rpy[0],pitch:v.rpy[1],yaw:v.rpy[2],acc:v.acc,passRadiusM:v.passRadiusM,type:m.type??0,typeId:m.typeId??0,chargingMode:m.chargingMode??0,component:m.component??0,turnDeg:m.turnDeg??0,state:m.state??'confirmed',isCorner:m.isCorner,isSlope:m.isSlope,isJunction:m.isJunction,mustPassThrough:v.mustPassThrough,alignFinalYaw:v.alignFinalYaw,turnable:v.turnable});
    summary.textContent=`点 ${selected.id}｜xyz ${v.pos.map((x:number)=>Number(x).toFixed(3)).join(', ')}｜yaw ${(Number(v.rpy?.[2]||0)*180/Math.PI).toFixed(1)}°｜acc ${Number(v.acc).toFixed(2)} m｜pass ${Number(v.passRadiusM).toFixed(2)} m｜拐点 ${!!m.isCorner}｜坡点 ${!!m.isSlope}｜交汇 ${!!m.isJunction}｜终点对齐 ${!!v.alignFinalYaw}`;
  }else{
    const e=topology.edges[selected.id],m=e.meta||{},box=m.obstacleBoxM||[0,0,0,0];$('edgeId').textContent=selected.id;
    setForm(ef,{v0:e.v[0],v1:e.v[1],dir:m.dir??0,controllerMode:m.controllerMode??'auto',linearSpeedMps:m.linearSpeedMps??.8,angularSpeedRadps:m.angularSpeedRadps??0,obstacleMode:m.obstacleMode??0,heightOffsetM:m.heightOffsetM??0,headingAngleRad:m.headingAngleRad??0,gridMapName:m.gridMapName??'',box0:box[0]??0,box1:box[1]??0,box2:box[2]??0,box3:box[3]??0,rotationAllowed:e.rotationAllowed});
    summary.textContent=`边 ${selected.id}｜${e.v[0]} ↔ ${e.v[1]}｜${edgeModeStyle(e).label}｜控制器 ${m.controllerMode??'auto'}｜线速度 ${Number(m.linearSpeedMps??.8).toFixed(2)} m/s｜方向 ${m.travelMode??m.dir??0}｜允许旋转 ${!!e.rotationAllowed}`;
  }
}
function compactNumber(v:any,maxDecimals=6){const n=Number(v);if(!Number.isFinite(n))return String(v??'');if(Number.isInteger(n))return String(n);return Number(n.toFixed(maxDecimals)).toString()}
function setForm(form:HTMLFormElement,values:J){for(const [k,v] of Object.entries(values)){const el=form.elements.namedItem(k) as HTMLInputElement;if(!el)continue;if(el.type==='checkbox')el.checked=!!v;else if(el.type==='number')el.value=compactNumber(v);else el.value=String(v??'')}}
function formValues(form:HTMLFormElement){const o:J={};new FormData(form).forEach((v,k)=>o[k]=v);for(const el of [...form.elements] as HTMLInputElement[]){if(el.name&&el.type==='checkbox')o[el.name]=el.checked}return o}
function checkpoint(){if(topology)undoStack.push(JSON.stringify(topology));if(undoStack.length>50)undoStack.shift();redoStack.length=0;dirty=true;setDirty()}
function setDirty(){$('dirtyState').textContent=dirty?'有未保存修改（仅浏览器内存）':'已保存到 JSON';$('dirtyState').style.color=dirty?'#f2b84b':'#6f8b97'}
function nextId(collection:J){return String(Math.max(0,...Object.keys(collection).map(Number).filter(Number.isFinite))+1)}
function defaultVertex(pos:number[]){return{pos,rpy:[0,0,0],meta:{type:0,typeId:0,isCorner:false,isSlope:false,isJunction:false,chargingMode:0,source:'web_console',sourceStamp:Date.now()/1000,component:0,turnDeg:0,state:'confirmed'},pcd:'',acc:.5,turnable:true,alignFinalYaw:false,mustPassThrough:false,passRadiusM:.45}}
function recomputeIncidentEdgeWeights(vertexId:string){if(!topology)return;for(const e of Object.values<any>(topology.edges||{})){if(!e.v.map(String).includes(vertexId))continue;const a=topology.vertices[String(e.v[0])]?.pos,b=topology.vertices[String(e.v[1])]?.pos;if(a&&b)e.weight=Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2])}}
function addVertex(pos:number[]){if(!topology)return;checkpoint();const id=nextId(topology.vertices);topology.vertices[id]=defaultVertex(pos);selected={kind:'vertex',id};renderTopology()}
function addEdge(a:string,b:string){
  if(!topology||a===b)return;if(Object.values<any>(topology.edges).some(e=>{const v=e.v.map(String);return v.includes(a)&&v.includes(b)})){toast('两点之间已经存在边',true);return}
  checkpoint();const id=nextId(topology.edges),pa=topology.vertices[a].pos,pb=topology.vertices[b].pos;topology.edges[id]={v:[Number(a),Number(b)],weight:Math.hypot(pa[0]-pb[0],pa[1]-pb[1],pa[2]-pb[2]),rotationAllowed:true,meta:{dir:0,source:'web_console',linearSpeedMps:.8,angularSpeedRadps:0,heightOffsetM:0,obstacleMode:0,travelMode:'bidirectional',headingAngleRad:0,obstacleBoxM:[0,0,0,0],gridMapName:'',controllerMode:'auto'}};
  edgeStart=null;selected={kind:'edge',id};renderTopology();toast(`已创建边 ${id}: ${a} ↔ ${b}`);
}

function pointerNdc(e:PointerEvent){const r=renderer.domElement.getBoundingClientRect();pointer.x=(e.clientX-r.left)/r.width*2-1;pointer.y=-(e.clientY-r.top)/r.height*2+1;raycaster.setFromCamera(pointer,camera)}
function groundPoint(e:PointerEvent,pickCloud=false){pointerNdc(e);if(pickCloud&&mapGroup.visible){const hit=raycaster.intersectObjects(mapGroup.children,false)[0];if(hit)return hit.point}const p=new THREE.Vector3();return raycaster.ray.intersectPlane(interactionPlane,p)?p:null}
renderer.domElement.addEventListener('pointerdown',e=>{
  if(e.button!==0)return;pointerNdc(e);
  if(tool==='initial'){interactionPlane.constant=0;dragStart=groundPoint(e,false);if(dragStart)dragStart.z=0;controls.enabled=false;return}
  const hits=raycaster.intersectObjects(topoGroup.children,false).filter(h=>['vertex','edge'].includes(String((h.object.userData as any).kind||'')));
  if(hits.length){const u=hits[0].object.userData as any;
    if(tool==='navigate'&&u.kind==='vertex'){
      selected={kind:'vertex',id:u.id};refreshTopologySelectionStyles();renderInspector();
      sendGoal(+u.id);
      $('quickActionStatus').textContent=`已点选导航到顶点 ${u.id}`;
      return;
    }
    if(tool==='edge'&&u.kind==='vertex'){
      if(!edgeStart){edgeStart=u.id;selected={kind:'vertex',id:u.id};refreshTopologySelectionStyles();renderInspector();toast(`已选择连接起点 ${u.id}，请再点击第二个点`)}
      else if(edgeStart===u.id){edgeStart=null;selected={kind:'vertex',id:u.id};refreshTopologySelectionStyles();toast('已取消连接起点')}
      else addEdge(edgeStart,u.id);
    }else{
      selected={kind:u.kind,id:u.id};refreshTopologySelectionStyles();renderInspector();
      if(u.kind==='vertex'){
        if(!$<HTMLInputElement>('startId').value)$<HTMLInputElement>('startId').value=u.id;else $<HTMLInputElement>('goalId').value=u.id;
        if(tool==='select'&&$<HTMLInputElement>('enableVertexDrag').checked){draggingVertex={id:u.id,object:hits[0].object};dragMoved=false;interactionPlane.constant=-topology.vertices[u.id].pos[2];controls.enabled=false}
      }
    }return;
  }
  if(tool==='select'){selected=null;refreshTopologySelectionStyles();renderInspector()}
  const p=groundPoint(e,true);if(tool==='vertex'&&p)addVertex([p.x,p.y,p.z]);
});
renderer.domElement.addEventListener('pointermove',e=>{
  if(draggingVertex){const p=groundPoint(e);if(!p)return;if(!dragMoved){checkpoint();dragMoved=true}draggingVertex.object.position.copy(p);topology.vertices[draggingVertex.id].pos=[p.x,p.y,p.z];renderInspector();return}
  if(!dragStart)return;const p=groundPoint(e);if(!p)return;if(yawArrow)scene.remove(yawArrow);const dir=p.clone().sub(dragStart);dir.z=0;if(dir.length()<.05)return;yawArrow=new THREE.ArrowHelper(dir.normalize(),dragStart,Math.min(2,dir.length()),0xffd84d);scene.add(yawArrow);
});
renderer.domElement.addEventListener('pointerup',e=>{
  if(draggingVertex){const id=draggingVertex.id;draggingVertex=null;controls.enabled=true;if(dragMoved){recomputeIncidentEdgeWeights(id);renderTopology()}return}
  if(!dragStart)return;const p=groundPoint(e);controls.enabled=true;if(p){const yaw=Math.atan2(p.y-dragStart.y,p.x-dragStart.x);send('localization.set_initial_pose',{position:[dragStart.x,dragStart.y,0],yaw})}if(yawArrow)scene.remove(yawArrow);yawArrow=null;dragStart=null;interactionPlane.constant=0;tool='select';setToolButton();
});

type BatchField={key:string,label:string,type:'bool'|'int'|'number'|'text'|'enum'|'number4',values?:string[]};
const vertexBatchFields:BatchField[]=[
  {key:'isCorner',label:'拐点 isCorner',type:'bool'},{key:'isSlope',label:'坡点 isSlope',type:'bool'},
  {key:'isJunction',label:'交汇点 isJunction',type:'bool'},{key:'type',label:'type',type:'int'},
  {key:'typeId',label:'typeId',type:'int'},{key:'chargingMode',label:'chargingMode',type:'int'},
  {key:'acc',label:'到点精度 acc',type:'number'},{key:'turnable',label:'允许旋转 turnable',type:'bool'},
  {key:'alignFinalYaw',label:'终点对齐 alignFinalYaw',type:'bool'},
  {key:'mustPassThrough',label:'必须经过 mustPassThrough',type:'bool'},
  {key:'passRadiusM',label:'通过半径 passRadiusM',type:'number'},
];
const edgeBatchFields:BatchField[]=[
  {key:'obstacleMode',label:'obstacleMode',type:'int'},
  {key:'controllerMode',label:'controllerMode',type:'enum',values:['auto','pid','local_planner','efficient_3d_local_planner']},
  {key:'travelMode',label:'travelMode / dir',type:'enum',values:['bidirectional','first_to_second','second_to_first']},
  {key:'rotationAllowed',label:'rotationAllowed',type:'bool'},
  {key:'linearSpeedMps',label:'linearSpeedMps',type:'number'},
  {key:'angularSpeedRadps',label:'angularSpeedRadps',type:'number'},
  {key:'heightOffsetM',label:'heightOffsetM',type:'number'},
  {key:'headingAngleRad',label:'headingAngleRad',type:'number'},
  {key:'obstacleBoxM',label:'obstacleBoxM（前,后,左,右）',type:'number4'},
  {key:'gridMapName',label:'gridMapName',type:'text'},
];
function activeBatchFields(){return $<HTMLSelectElement>('batchKind').value==='vertex'?vertexBatchFields:edgeBatchFields}
function updateBatchFieldUi(){
  const fieldSelect=$<HTMLSelectElement>('batchField'),old=fieldSelect.value;fieldSelect.innerHTML='';
  for(const field of activeBatchFields()){const option=document.createElement('option');option.value=field.key;option.textContent=field.label;fieldSelect.append(option)}
  if([...fieldSelect.options].some(option=>option.value===old))fieldSelect.value=old;
  const field=activeBatchFields().find(item=>item.key===fieldSelect.value),input=$<HTMLInputElement>('batchValue');
  input.placeholder=field?.type==='bool'?'true 或 false':field?.type==='number4'?'例如 1,0.2,0.3,0.3':field?.values?.join(' / ')||'输入新值';
}
function parseBatchValue(field:BatchField,text:string){
  const value=text.trim();
  if(field.type==='bool'){if(/^(true|1|yes|on)$/i.test(value))return true;if(/^(false|0|no|off)$/i.test(value))return false;throw new Error('布尔值必须为 true 或 false')}
  if(field.type==='int'){const number=Number(value);if(!Number.isInteger(number))throw new Error('请输入整数');if(field.key==='obstacleMode'&&(number<0||number>4))throw new Error('obstacleMode 必须为 0～4');return number}
  if(field.type==='number'){const number=Number(value);if(!Number.isFinite(number))throw new Error('请输入有限数字');if(['acc','passRadiusM'].includes(field.key)&&number<=0)throw new Error(`${field.key} 必须大于 0`);if(['linearSpeedMps','angularSpeedRadps'].includes(field.key)&&number<0)throw new Error(`${field.key} 不能小于 0`);return number}
  if(field.type==='number4'){const values=value.split(/[，,\s]+/).filter(Boolean).map(Number);if(values.length!==4||values.some(number=>!Number.isFinite(number)))throw new Error('obstacleBoxM 需要四个数字：前,后,左,右');return values}
  if(field.type==='enum'){if(!field.values?.includes(value))throw new Error(`可选值：${field.values?.join(' / ')}`);return value}
  return value;
}
function shortestTopologyPath(start:string,goal:string){
  const vertices=topology?.vertices||{},edges=topology?.edges||{};if(!vertices[start]||!vertices[goal])throw new Error('批量起点或终点不存在');
  const distance=new Map<string,number>(Object.keys(vertices).map(id=>[id,Infinity])),previous=new Map<string,{vertex:string,edge:string}>(),remaining=new Set(Object.keys(vertices));distance.set(start,0);
  while(remaining.size){let current:string|null=null,best=Infinity;for(const id of remaining){const value=distance.get(id)!;if(value<best){best=value;current=id}}if(current===null||!Number.isFinite(best))break;remaining.delete(current);if(current===goal)break;
    for(const [edgeId,edge] of Object.entries<any>(edges)){const a=String(edge.v?.[0]),b=String(edge.v?.[1]);const next=a===current?b:b===current?a:null;if(!next||!remaining.has(next))continue;let weight=Number(edge.weight);if(!Number.isFinite(weight)||weight<0){const pa=vertices[a].pos,pb=vertices[b].pos;weight=Math.hypot(pa[0]-pb[0],pa[1]-pb[1],pa[2]-pb[2])}const candidate=best+weight;if(candidate<distance.get(next)!){distance.set(next,candidate);previous.set(next,{vertex:current,edge:edgeId})}}
  }
  if(start!==goal&&!previous.has(goal))throw new Error(`点 ${start} 与点 ${goal} 之间没有连通路径`);const vertexIds=[goal],edgeIds:string[]=[];let cursor=goal;while(cursor!==start){const step=previous.get(cursor);if(!step)throw new Error('无法还原最短路径');edgeIds.push(step.edge);cursor=step.vertex;vertexIds.push(cursor)}return {vertexIds:vertexIds.reverse(),edgeIds:edgeIds.reverse()};
}
function applyBatchField(kind:'vertex'|'edge',ids:string[],field:BatchField,value:any){
  for(const id of ids){const item=kind==='vertex'?topology.vertices[id]:topology.edges[id];if(!item)continue;item.meta=item.meta||{};
    if(kind==='vertex'){
      if(['acc','turnable','alignFinalYaw','mustPassThrough','passRadiusM'].includes(field.key))item[field.key]=value;else item.meta[field.key]=value;
    }else if(field.key==='rotationAllowed')item.rotationAllowed=value;
    else if(field.key==='travelMode'){item.meta.travelMode=value;item.meta.dir={bidirectional:0,first_to_second:1,second_to_first:2}[value as string]}
    else item.meta[field.key]=value;
  }
}
function applyAllObstacleMode(mode:0|1|2){
  if(!topology){toast('请先加载拓扑',true);return}const ids=Object.keys(topology.edges||{});if(!ids.length){toast('当前拓扑没有边',true);return}const labels={0:'停障（obstacleMode=0，PID 三维扫掠）',1:'绕障（obstacleMode=1，Efficient 3D）',2:'不停障（obstacleMode=2，PID，关闭普通点云停障）'} as const,label=labels[mode],safetyNote=mode===2?'\n模式 2 仍保留外部急停、碰撞等级和超时保护。':'';if(!confirm(`确认把全图 ${ids.length} 条边设为${label}？${safetyNote}\n修改可撤销，尚不会写入文件。`))return;checkpoint();for(const id of ids){const edge=topology.edges[id];edge.meta=edge.meta||{};edge.meta.obstacleMode=mode;edge.meta.controllerMode=mode===1?'efficient_3d_local_planner':'pid'}renderTopology();$('batchStatus').textContent=`已修改全图 ${ids.length} 条边为${label}；点击“保存拓扑”写入 JSON。`;toast(`已应用全图${mode===0?'停障':mode===1?'绕障':'不停障'}`);
}
function applyBatch(){
  if(!topology){toast('请先加载拓扑',true);return}const kind=$<HTMLSelectElement>('batchKind').value as 'vertex'|'edge',scope=$<HTMLSelectElement>('batchScope').value,field=activeBatchFields().find(item=>item.key===$<HTMLSelectElement>('batchField').value);if(!field)return;
  try{const value=parseBatchValue(field,$<HTMLInputElement>('batchValue').value);let ids:string[],scopeLabel:string;if(scope==='all'){ids=Object.keys(kind==='vertex'?topology.vertices||{}:topology.edges||{});scopeLabel='全图'}else{const start=$<HTMLInputElement>('batchStartId').value.trim(),goal=$<HTMLInputElement>('batchEndId').value.trim();if(!start||!goal)throw new Error('请输入批量起点和终点');const path=shortestTopologyPath(start,goal);ids=kind==='vertex'?path.vertexIds:path.edgeIds;scopeLabel=`最短路径 ${start} → ${goal}`}if(!ids.length)throw new Error('选择范围内没有可修改对象');if(!confirm(`确认把“${field.label}=${JSON.stringify(value)}”应用到${scopeLabel}的 ${ids.length} 个${kind==='vertex'?'点':'边'}？`))return;checkpoint();applyBatchField(kind,ids,field,value);renderTopology();$('batchStatus').textContent=`已修改${scopeLabel}的 ${ids.length} 个${kind==='vertex'?'点':'边'}；点击“保存拓扑”写入 JSON。`;toast('批量属性已应用');}catch(error){toast(error instanceof Error?error.message:String(error),true)}
}

$<HTMLFormElement>('vertexForm').onsubmit=e=>{
  e.preventDefault();if(!selected||selected.kind!=='vertex')return;checkpoint();const f=formValues(e.currentTarget as HTMLFormElement),v=topology.vertices[selected.id];v.meta=v.meta||{};
  v.pos=[+f.x,+f.y,+f.z];v.rpy=[+f.roll,+f.pitch,+f.yaw];v.acc=+f.acc;v.passRadiusM=+f.passRadiusM;
  v.meta.type=+f.type;v.meta.typeId=+f.typeId;v.meta.chargingMode=+f.chargingMode;v.meta.component=+f.component;v.meta.turnDeg=+f.turnDeg;v.meta.state=String(f.state||'confirmed');
  v.meta.isCorner=f.isCorner;v.meta.isSlope=f.isSlope;v.meta.isJunction=f.isJunction;v.mustPassThrough=f.mustPassThrough;v.alignFinalYaw=f.alignFinalYaw;v.turnable=f.turnable;
  recomputeIncidentEdgeWeights(selected.id);renderTopology();toast('顶点属性已应用；点击“保存拓扑”写回本地 JSON');
};
$<HTMLFormElement>('edgeForm').onsubmit=e=>{
  e.preventDefault();if(!selected||selected.kind!=='edge')return;checkpoint();const f=formValues(e.currentTarget as HTMLFormElement),x=topology.edges[selected.id];x.meta=x.meta||{};
  x.meta.dir=+f.dir;x.meta.travelMode=['bidirectional','first_to_second','second_to_first'][+f.dir];x.meta.controllerMode=f.controllerMode;x.meta.linearSpeedMps=+f.linearSpeedMps;x.meta.angularSpeedRadps=+f.angularSpeedRadps;delete x.meta.locomotionMode;x.meta.obstacleMode=+f.obstacleMode;x.meta.heightOffsetM=+f.heightOffsetM;x.meta.headingAngleRad=+f.headingAngleRad;x.meta.gridMapName=String(f.gridMapName||'');x.meta.obstacleBoxM=[+f.box0,+f.box1,+f.box2,+f.box3];x.rotationAllowed=f.rotationAllowed;
  renderTopology();toast('边属性已应用；点击“保存拓扑”写回本地 JSON');
};
$('vertexInitialPose').onclick=()=>{if(!selected||selected.kind!=='vertex')return;if(dirty&&!confirm('当前顶点有未保存修改。定位初值将按网页当前选中拓扑文件中已保存的数据发送。是否继续？'))return;send('localization.set_initial_pose_vertex',{vertex_id:+selected.id})};
$('vertexNavigate').onclick=()=>{if(!selected||selected.kind!=='vertex')return;sendGoal(+selected.id)};
$('deleteVertex').onclick=()=>{if(!selected||selected.kind!=='vertex'||!confirm(`删除顶点 ${selected.id} 及其所有关联边？`))return;checkpoint();const id=selected.id;delete topology.vertices[id];for(const [eid,e] of Object.entries<any>(topology.edges))if(e.v.map(String).includes(id))delete topology.edges[eid];selected=null;renderTopology()};
$('deleteEdge').onclick=()=>{if(!selected||selected.kind!=='edge'||!confirm(`删除边 ${selected.id}？`))return;checkpoint();delete topology.edges[selected.id];selected=null;renderTopology()};

document.querySelectorAll<HTMLButtonElement>('[data-start]').forEach(b=>b.onclick=()=>{
  const target=b.dataset.start;
  if(target==='planner'&&$<HTMLInputElement>('enableMotion').checked&&!confirm('即将启用实机运动。确认遥控急停已就绪、机器人周围安全？'))return;
  const request:any={target,enable_motion:$<HTMLInputElement>('enableMotion').checked};
  if(target==='mapping')request.mapping_config=$<HTMLSelectElement>('mappingConfig').value;
  send('process.start',request);
});
document.querySelectorAll<HTMLButtonElement>('[data-stop]').forEach(b=>b.onclick=()=>send('process.stop',{target:b.dataset.stop}));
document.querySelectorAll<HTMLButtonElement>('[data-tool]').forEach(b=>b.onclick=()=>{tool=b.dataset.tool!;edgeStart=null;if(tool==='robotVertex'){if(currentPose)addVertex([...currentPose.position]);else toast('尚无机器人位置',true);tool='select'}refreshTopologySelectionStyles();setToolButton()});
$('enableVertexDrag').onchange=()=>{if(!$<HTMLInputElement>('enableVertexDrag').checked&&draggingVertex){draggingVertex=null;dragMoved=false;controls.enabled=true}setToolButton();toast($<HTMLInputElement>('enableVertexDrag').checked?'拖动编辑已开启：拖动顶点会修改坐标':'拖动编辑已关闭：点击顶点只会选择/高亮')};
function setToolButton(){document.querySelectorAll('[data-tool]').forEach(x=>x.classList.toggle('active',(x as HTMLElement).dataset.tool===tool));$('hint').textContent=tool==='initial'?'在地图上按住左键拖出朝向':tool==='navigate'?'点选导航已开启：直接点击任意拓扑顶点即可导航过去':tool==='edge'?(edgeStart?`已选点 ${edgeStart}，请点击第二个点完成连接`:'依次点击两个拓扑点创建边；第一个点会绿色高亮'):tool==='vertex'?'点击 PCD/XY 平面添加拓扑点':($<HTMLInputElement>('enableVertexDrag').checked?'左键点击点/边选择并高亮；已开启拖动编辑，拖动顶点会修改坐标':'左键点击点/边只选择和高亮；拖动编辑默认关闭，右键旋转，滚轮缩放');const nav=$('pointNavigateMode');if(nav){nav.textContent=tool==='navigate'?'点选导航：开启':'点选导航：关闭';nav.classList.toggle('active',tool==='navigate')}}
$('initialPoseMode').onclick=()=>{tool='initial';edgeStart=null;refreshTopologySelectionStyles();setToolButton()};
$('pointNavigateMode').onclick=()=>{tool=tool==='navigate'?'select':'navigate';edgeStart=null;refreshTopologySelectionStyles();setToolButton()};
$('refreshResources').onclick=()=>send('resource.list');
$<HTMLSelectElement>('pcdSelect').onchange=()=>{pcdChoiceDirty=true};
$<HTMLSelectElement>('topologySelect').onchange=()=>{topologyChoiceDirty=true};
function requestedMapVoxel(){const input=$<HTMLInputElement>('mapVoxelSize');let value=Number(input.value);if(!Number.isFinite(value))value=0.20;value=Math.max(mapVoxelMin,Math.min(mapVoxelMax,value));input.value=value.toFixed(2);return value}
$('selectPcd').onclick=()=>{const path=$<HTMLSelectElement>('pcdSelect').value;if(path)send('map.file.select',{path,voxel_m:requestedMapVoxel()});else toast('请选择地图文件夹',true)};
$('applyMapVoxel').onclick=()=>{if(!selectedPcd){toast('请先加载 PCD 地图',true);return}send('map.cloud_lod',{voxel_m:requestedMapVoxel()})};
$('selectTopology').onclick=()=>{const path=$<HTMLSelectElement>('topologySelect').value;if(path)send('topology.file.select',{path});else toast('请选择本地拓扑 JSON',true)};
$<HTMLSelectElement>('poseSourceSelect').onchange=renderPoseSourcePath;
$('generateTopology').onclick=()=>{const name=$<HTMLSelectElement>('poseSourceSelect').value;if(!name){toast('请选择含 pose.json 的地图文件夹',true);return}if(dirty){toast('当前拓扑有未保存修改，请先保存或撤销后再生成',true);return}const overwrite=$<HTMLInputElement>('topologyGenerateOverwrite').checked;if(overwrite&&!confirm(`确认覆盖 ${name} 已有的拓扑 JSON？后台不会改动原 pose.json，并会备份旧拓扑。`))return;send('topology.generate',{name,overwrite})};
$<HTMLSelectElement>('batchKind').onchange=updateBatchFieldUi;
$<HTMLSelectElement>('batchField').onchange=updateBatchFieldUi;
$<HTMLSelectElement>('batchScope').onchange=()=>{$('batchPathInputs').hidden=$<HTMLSelectElement>('batchScope').value==='all'};
$('applyBatch').onclick=applyBatch;
$('allEdgesStop').onclick=()=>applyAllObstacleMode(0);
$('allEdgesAvoid').onclick=()=>applyAllObstacleMode(1);
$('allEdgesNoStop').onclick=()=>applyAllObstacleMode(2);
$('saveMappingMap').onclick=()=>{const name=$<HTMLInputElement>('mappingMapName').value.trim();if(!name){toast('请输入地图名称',true);return}if(!/^[A-Za-z0-9._-]+$/.test(name)){toast('地图名称只能包含字母、数字、点、下划线和横线',true);return}const overwrite=$<HTMLInputElement>('mappingOverwrite').checked;if(overwrite&&!confirm(`确认允许覆盖地图 ${name}？`))return;send('mapping.save_map',{name,overwrite})};
$('startRecording').onclick=()=>send('topology.record.start');
$('stopRecording').onclick=()=>{const name=$<HTMLInputElement>('recordingName').value.trim();if(!name){toast('请输入保存名称',true);return}send('topology.record.stop',{name})};
$('abortRecording').onclick=()=>{if(confirm('放弃本次自动打点？临时数据不会保存为正式拓扑。'))send('topology.record.abort')};
$('saveTopology').onclick=()=>{if(!topology)return;if(!selectedTopology){toast('请先加载本地拓扑 JSON',true);return}send('topology.save',{data:topology,revision})};
$('sendGoalOnly').onclick=()=>{const goal=+$<HTMLInputElement>('goalOnlyId').value;if(!Number.isInteger(goal)||goal<0){toast('请输入有效目标点编号',true);return}sendGoal(goal)};
$('planRoute').onclick=()=>{const start=+$<HTMLInputElement>('startId').value,goal=+$<HTMLInputElement>('goalId').value;chooseNavigationTarget(goal);send('navigation.plan',{start_id:start,goal_id:goal})};
function startLoop(mode:'fixed'|'goal_only'){const start=+$<HTMLInputElement>('loopStartId').value,goal=+$<HTMLInputElement>('loopGoalId').value,dwell=+$<HTMLInputElement>('loopDwell').value,rounds=+$<HTMLInputElement>('loopRounds').value;if(!Number.isInteger(start)||start<0||!Number.isInteger(goal)||goal<0||start===goal){toast('循环需要两个不同的有效点位',true);return}if(!Number.isFinite(dwell)||dwell<0){toast('端点停留时间无效',true);return}if(!Number.isInteger(rounds)||rounds<0){toast('往返次数必须是非负整数',true);return}chooseNavigationTarget(goal);send('navigation.loop.start',{mode,start_id:start,goal_id:goal,dwell_time_s:dwell,max_round_trips:rounds})}
$('startFixedLoop').onclick=()=>startLoop('fixed');
$('startGoalOnlyLoop').onclick=()=>startLoop('goal_only');
$('stopLoop').onclick=()=>send('navigation.loop.stop');
$('pause').onclick=()=>send('navigation.pause');$('resume').onclick=()=>send('navigation.resume');$('cancel').onclick=()=>send('navigation.cancel');
$('undo').onclick=()=>{if(!undoStack.length)return;redoStack.push(JSON.stringify(topology));topology=JSON.parse(undoStack.pop()!);dirty=true;renderTopology();setDirty()};
$('redo').onclick=()=>{if(!redoStack.length)return;undoStack.push(JSON.stringify(topology));topology=JSON.parse(redoStack.pop()!);dirty=true;renderTopology();setDirty()};
$('clearLog').onclick=()=>logBox.textContent='';
$<HTMLInputElement>('showMap').onchange=e=>mapGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showLive').onchange=e=>{const enabled=(e.target as HTMLInputElement).checked;liveGroup.visible=enabled;if(!enabled)clearGroup(liveGroup);send('cloud.live.set',{enabled})};$<HTMLInputElement>('showTopo').onchange=e=>topoGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showPreview').onchange=e=>previewGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showTrack').onchange=e=>trackGroup.visible=(e.target as HTMLInputElement).checked;
$<HTMLInputElement>('showVertexYaw').onchange=e=>{const visible=(e.target as HTMLInputElement).checked;for(const object of topoGroup.children)if((object.userData as any).yawIndicator)object.visible=visible};
$('followRobot').onclick=()=>setRobotFollow(!followRobot);
$('fitView').onclick=()=>{const target=(topoGroup.visible&&topoGroup.children.length)?topoGroup:mapGroup;const box=new THREE.Box3().setFromObject(target);if(box.isEmpty())return;setRobotFollow(false);const size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(size.length()*.55,-size.length()*.55,size.length()*.45));camera.near=Math.max(.001,size.length()/100000);camera.far=Math.max(100,size.length()*100);camera.updateProjectionMatrix()};
window.addEventListener('keydown',e=>{const target=e.target as HTMLElement;if(e.key.toLowerCase()==='f'&&!target.closest('input, select, textarea'))setRobotFollow(!followRobot)});
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue=''}});
document.querySelectorAll<HTMLElement>('[data-module-tab]').forEach(x=>x.onclick=()=>activateModule(x.dataset.moduleTab||'radar'));
activateModule('radar');
liveGroup.visible=$<HTMLInputElement>('showLive').checked;
trackGroup.visible=$<HTMLInputElement>('showTrack').checked;
updateBatchFieldUi();connect();resize();setToolButton();
