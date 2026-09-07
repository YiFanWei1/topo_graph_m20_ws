import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import './style.css';

type J = Record<string, any>;
const $ = <T extends HTMLElement>(id:string) => document.getElementById(id) as T;
const logBox=$<HTMLDivElement>('logs'), toastBox=$<HTMLDivElement>('toast');
let requestSequence=0;
const requestId=()=>globalThis.crypto?.randomUUID?.() ??
  `${Date.now().toString(36)}-${(++requestSequence).toString(36)}-${Math.random().toString(36).slice(2,10)}`;
const send=(type:string,data:J={})=>{if(ws.readyState!==WebSocket.OPEN){toast('WebSocket 未连接',true);return}ws.send(JSON.stringify({type,request_id:requestId(),...data}))};
const toast=(s:string,error=false)=>{toastBox.textContent=s;toastBox.className=`show ${error?'error':''}`;setTimeout(()=>toastBox.className='',2600)};
const log=(s:string)=>{const t=new Date().toLocaleTimeString();logBox.textContent+=`[${t}] ${s}\n`;logBox.scrollTop=logBox.scrollHeight};

let ws:WebSocket, clientId='', hasControl=false, heartbeat:number|undefined;
let catalog:any[]=[]; let topology:any=null, revision='', dirty=false, selectedMap='', selectedTopology='';
let selected:{kind:'vertex'|'edge',id:string}|null=null, tool='select', edgeStart:string|null=null;
let currentPose:any=null; const undoStack:string[]=[], redoStack:string[]=[];
let lastRecordingState='idle';
const mapChunks=new Map<number,Float32Array[]>();

function connect(){
  ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);ws.binaryType='arraybuffer';
  ws.onopen=()=>{ $('wsLamp').classList.add('ok');$('wsText').textContent='已连接';send('map.list') };
  ws.onclose=()=>{hasControl=false;$('wsLamp').classList.remove('ok');$('wsText').textContent='连接断开，正在重连';clearInterval(heartbeat);setTimeout(connect,1500)};
  ws.onerror=()=>{};
  ws.onmessage=e=>typeof e.data==='string'?handle(JSON.parse(e.data)):handleCloud(e.data);
}

function handle(m:J){
  if(m.type==='hello'){clientId=m.client_id;return}
  if(m.type==='ack'){if(m.message!=='heartbeat'){toast(m.message,!m.success);log(`${m.success?'✓':'✗'} ${m.message}`)}return}
  if(m.type==='error'){toast(m.message,true);log(`错误: ${m.message}`);return}
  if(m.type==='process.log'){log(`[${m.process}] ${m.line}`);return}
  if(m.type==='map.catalog'){catalog=m.maps||[];if('selected' in m)selectedMap=m.selected;if('selected_topology' in m)selectedTopology=m.selected_topology;renderCatalog();return}
  if(m.type==='topology.catalog'){if('selected' in m)selectedTopology=m.selected;renderTopologyCatalog(m.topologies||[]);return}
  if(m.type==='topology.snapshot'||m.type==='topology.saved'){
    topology=m.data;revision=m.revision;selectedTopology=m.id||selectedTopology;dirty=false;undoStack.length=0;redoStack.length=0;renderTopology();renderCatalog();setDirty();return
  }
  if(m.type==='topology.preview'){renderTopologyPreview(m.data);return}
  if(m.type==='topology.preview.clear'){clearGroup(previewGroup);return}
  if(m.type==='topology.record.saved'){toast(`拓扑 ${m.name} 已保存并选中`);log(`新控制链文件：${m.path}`);clearGroup(previewGroup);return}
  if(m.type==='pose'){currentPose=m;renderPose();return}
  if(m.type==='velocity'){$('velocity').textContent=`${Number(m.vx).toFixed(2)} / ${Number(m.wz).toFixed(2)}`;return}
  if(m.type==='ros.status'){
    if(m.source==='active_controller')$('controller').textContent=typeof m.data==='string'?m.data:(m.data?.data||'none');
    return
  }
  if(m.type==='control.expired'){hasControl=false;updateControlButton();toast(m.message,true);return}
  if(m.type==='snapshot'){
    updateProcess('radar',m.processes?.radar?.running);updateProcess('localization',m.processes?.localization?.running);updateProcess('planner',m.processes?.planner?.running);updateRecording(m.topology_recording_state,m.processes?.topology_recording?.running);
    if(m.pose){currentPose=m.pose;renderPose()} if(m.trajectory)renderTrajectory(m.trajectory);
    if(m.control_owner===clientId&&!hasControl){hasControl=true;startHeartbeat()} else if(m.control_owner!==clientId){hasControl=false;clearInterval(heartbeat)}
    updateControlButton();$('statusJson').textContent=JSON.stringify({processes:m.processes,topics:m.topics,ros:m.ros},null,2);return
  }
  if(m.type==='map.cloud_ready'){log(`地图点云已加载：${m.points} 点，体素 ${m.voxel_m}m`)}
}

function updateProcess(name:string,running:boolean){const el=$(`${name}State`);el.textContent=running?'运行中':'未运行';el.classList.toggle('running',!!running)}
function renderCatalog(){const select=$<HTMLSelectElement>('mapSelect');const current=select.value;select.innerHTML='<option value="">请选择定位地图</option>';for(const m of catalog){const o=document.createElement('option');o.value=m.id;o.textContent=`${m.label} (${m.id})`;select.append(o)}select.value=selectedMap||current;const map=catalog.find(m=>m.id===(select.value||selectedMap));renderTopologyCatalog(map?.topologies||[])}
function renderTopologyCatalog(items:any[]){const select=$<HTMLSelectElement>('topologySelect'),current=select.value;select.innerHTML='<option value="">请选择拓扑图</option>';for(const t of items){if(t.enabled===false)continue;const o=document.createElement('option');o.value=t.id;o.textContent=t.label||t.id;select.append(o)}select.value=selectedTopology||current}
function updateRecording(state='idle',running=false){const names:J={idle:'未运行',starting:'启动中',recording:'记录中',processing:'处理中',saved:'已保存',aborted:'已放弃',error:'失败'};const el=$('recordingState');el.textContent=names[state]||state;el.classList.toggle('running',running||state==='processing');const active=state==='starting'||state==='recording'||state==='processing';const wasActive=lastRecordingState==='starting'||lastRecordingState==='recording'||lastRecordingState==='processing';topoGroup.visible=!active&&$<HTMLInputElement>('showTopo').checked;if(active&&!wasActive){selected=null;renderInspector()}if(!active&&wasActive)renderTopology();lastRecordingState=state}
function updateControlButton(){$('claimControl').textContent=hasControl?'释放控制权':'取得控制权';$('claimControl').classList.toggle('danger',hasControl)}
function startHeartbeat(){clearInterval(heartbeat);heartbeat=window.setInterval(()=>send('control.heartbeat'),1000)}

const scene=new THREE.Scene();scene.background=new THREE.Color(0x050b0f);scene.fog=new THREE.FogExp2(0x050b0f,0.012);
const camera=new THREE.PerspectiveCamera(55,1,.05,1000);camera.position.set(8,-10,9);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('viewport').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,0);controls.enableDamping=true;
scene.add(new THREE.GridHelper(100,100,0x23515d,0x122b34).rotateX(Math.PI/2));
scene.add(new THREE.AxesHelper(1.5));
const mapGroup=new THREE.Group(),liveGroup=new THREE.Group(),topoGroup=new THREE.Group(),previewGroup=new THREE.Group(),trackGroup=new THREE.Group();scene.add(mapGroup,liveGroup,topoGroup,previewGroup,trackGroup);
const robot=new THREE.Group();const body=new THREE.Mesh(new THREE.BoxGeometry(.55,.32,.16),new THREE.MeshBasicMaterial({color:0x43f1c6}));body.position.z=.18;robot.add(body);const nose=new THREE.Mesh(new THREE.ConeGeometry(.12,.35,8),new THREE.MeshBasicMaterial({color:0xffffff}));nose.rotation.z=-Math.PI/2;nose.position.x=.42;nose.position.z=.18;robot.add(nose);scene.add(robot);
const raycaster=new THREE.Raycaster();raycaster.params.Points!.threshold=.16;const pointer=new THREE.Vector2();
const interactionPlane=new THREE.Plane(new THREE.Vector3(0,0,1),0);let dragStart:THREE.Vector3|null=null, yawArrow:THREE.ArrowHelper|null=null;let draggingVertex:{id:string,object:THREE.Object3D}|null=null,dragMoved=false;
let followRobot=false,followPosition:THREE.Vector3|null=null;

function setRobotFollow(enabled:boolean){
  if(enabled&&!currentPose){toast('尚无机器人位置，无法开启跟随',true);return}
  followRobot=enabled;controls.enablePan=!enabled;followPosition=null;
  const button=$<HTMLButtonElement>('followRobot');
  button.textContent=enabled?'停止跟随':'跟随机器人';
  button.classList.toggle('active',enabled);button.setAttribute('aria-pressed',String(enabled));
  if(enabled){updateFollowCamera(currentPose.position);toast('已跟随机器人中心，可旋转和缩放视角')}
}

function updateFollowCamera(position:number[]){
  if(!followRobot)return;
  const next=new THREE.Vector3(position[0],position[1],position[2]);
  if(followPosition){
    const delta=next.clone().sub(followPosition);camera.position.add(delta);controls.target.add(delta);
  }else{
    const offset=camera.position.clone().sub(controls.target);controls.target.copy(next);camera.position.copy(next).add(offset);
  }
  followPosition=next;controls.update();
}

function resize(){const box=$('viewport').getBoundingClientRect();camera.aspect=box.width/box.height;camera.updateProjectionMatrix();renderer.setSize(box.width,box.height,false)}new ResizeObserver(resize).observe($('viewport'));
function animate(){requestAnimationFrame(animate);controls.update();renderer.render(scene,camera)}animate();
function clearGroup(g:THREE.Group){for(const o of [...g.children]){g.remove(o);const x=o as any;x.geometry?.dispose();if(Array.isArray(x.material))x.material.forEach((m:any)=>m.dispose());else x.material?.dispose()}}

function handleCloud(buffer:ArrayBuffer){
  if(buffer.byteLength<60)return;const d=new DataView(buffer);if(String.fromCharCode(...new Uint8Array(buffer,0,4))!=='R3PC')return;
  const stream=d.getUint8(5),flags=d.getUint16(6,true),seq=d.getUint32(8,true),count=d.getUint32(20,true),stride=d.getUint16(24,true),frameLen=d.getUint16(26,true),chunk=d.getUint32(28,true),chunks=d.getUint32(32,true);let off=60+frameLen;
  const xyz=new Float32Array(count*3);const intensity=new Float32Array(count);for(let i=0;i<count;i++){xyz[i*3]=d.getFloat32(off,true);xyz[i*3+1]=d.getFloat32(off+4,true);xyz[i*3+2]=d.getFloat32(off+8,true);if(flags&1)intensity[i]=d.getFloat32(off+12,true);off+=stride}
  if(stream===1){if(!mapChunks.has(seq))mapChunks.set(seq,[]);mapChunks.get(seq)![chunk]=xyz;if(mapChunks.get(seq)!.filter(Boolean).length===chunks){const arrays=mapChunks.get(seq)!;const total=arrays.reduce((n,a)=>n+a.length,0);const all=new Float32Array(total);let p=0;arrays.forEach(a=>{all.set(a,p);p+=a.length});mapChunks.clear();setPoints(mapGroup,all,0xd6f3ff,.025)}}
  else setPoints(liveGroup,xyz,stream===2?0x43f1c6:0xf2b84b,.045);
}
function setPoints(group:THREE.Group,positions:Float32Array,color:number,size:number){clearGroup(group);const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(positions,3));geo.computeBoundingSphere();const pts=new THREE.Points(geo,new THREE.PointsMaterial({color,size,sizeAttenuation:true,transparent:true,opacity:.9}));group.add(pts)}

function renderPose(){if(!currentPose)return;const p=currentPose.position,q=currentPose.orientation;robot.position.set(p[0],p[1],p[2]);robot.quaternion.set(q[0],q[1],q[2],q[3]);updateFollowCamera(p);$('poseX').textContent=p[0].toFixed(2);$('poseY').textContent=p[1].toFixed(2);const yaw=Math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));$('poseYaw').textContent=`${(yaw*180/Math.PI).toFixed(1)}°`}
function renderTrajectory(points:number[][]){clearGroup(trackGroup);if(points.length<2)return;const geo=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p)));trackGroup.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x4ad7ff})))}

function renderTopology(){clearGroup(topoGroup);selected=null;renderInspector();if(!topology)return;const vertices=topology.vertices||{};for(const [id,v] of Object.entries<any>(vertices)){const color=v.meta?.isSlope?0xff8a4c:v.meta?.isCorner?0xffd84d:0x55aaff;const mesh=new THREE.Mesh(new THREE.SphereGeometry(.13,14,10),new THREE.MeshBasicMaterial({color}));mesh.position.set(...v.pos);mesh.userData={kind:'vertex',id};topoGroup.add(mesh)}for(const [id,e] of Object.entries<any>(topology.edges||{})){const a=vertices[String(e.v[0])],b=vertices[String(e.v[1])];if(!a||!b)continue;const geo=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a.pos),new THREE.Vector3(...b.pos)]);const line=new THREE.Line(geo,new THREE.LineBasicMaterial({color:0x3ab4bb}));line.userData={kind:'edge',id};topoGroup.add(line)}}
function renderTopologyPreview(data:any){clearGroup(previewGroup);const vertices=data?.vertices||{};for(const v of Object.values<any>(vertices)){const color=v.meta?.isSlope?0xffd24a:v.meta?.isCorner?0xff5151:0x43a7ff;const mesh=new THREE.Mesh(new THREE.SphereGeometry(.10,10,8),new THREE.MeshBasicMaterial({color}));mesh.position.set(...v.pos);previewGroup.add(mesh)}for(const e of Object.values<any>(data?.edges||{})){const a=vertices[String(e.v[0])],b=vertices[String(e.v[1])];if(!a||!b)continue;const source=String(e.meta?.source||'');const color=source.includes('loop')?0xff8a32:0x43e58a;const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a.pos),new THREE.Vector3(...b.pos)]),new THREE.LineBasicMaterial({color,linewidth:source.includes('loop')?3:1}));previewGroup.add(line)}}
function renderInspector(){const none=$('emptyInspector'),vf=$<HTMLFormElement>('vertexForm'),ef=$<HTMLFormElement>('edgeForm');none.hidden=!!selected;vf.hidden=selected?.kind!=='vertex';ef.hidden=selected?.kind!=='edge';if(!selected||!topology)return;if(selected.kind==='vertex'){const v=topology.vertices[selected.id];$('vertexId').textContent=selected.id;setForm(vf,{x:v.pos[0],y:v.pos[1],z:v.pos[2],roll:v.rpy[0],pitch:v.rpy[1],yaw:v.rpy[2],acc:v.acc,passRadiusM:v.passRadiusM,isCorner:v.meta?.isCorner,isSlope:v.meta?.isSlope,isJunction:v.meta?.isJunction,mustPassThrough:v.mustPassThrough,alignFinalYaw:v.alignFinalYaw,turnable:v.turnable})}else{const e=topology.edges[selected.id];$('edgeId').textContent=selected.id;setForm(ef,{v0:e.v[0],v1:e.v[1],dir:e.meta.dir,controllerMode:e.meta.controllerMode,linearSpeedMps:e.meta.linearSpeedMps,angularSpeedRadps:e.meta.angularSpeedRadps,locomotionMode:e.meta.locomotionMode,obstacleMode:e.meta.obstacleMode,heightOffsetM:e.meta.heightOffsetM,rotationAllowed:e.rotationAllowed})}}
function setForm(form:HTMLFormElement,values:J){for(const [k,v] of Object.entries(values)){const el=form.elements.namedItem(k) as HTMLInputElement;if(!el)continue;if(el.type==='checkbox')el.checked=!!v;else el.value=String(v??'')}}
function formValues(form:HTMLFormElement){const o:J={};new FormData(form).forEach((v,k)=>o[k]=v);for(const el of [...form.elements] as HTMLInputElement[]){if(el.name&&el.type==='checkbox')o[el.name]=el.checked}return o}
function checkpoint(){if(topology)undoStack.push(JSON.stringify(topology));if(undoStack.length>50)undoStack.shift();redoStack.length=0;dirty=true;setDirty()}
function setDirty(){$('dirtyState').textContent=dirty?'有未保存修改':'已保存';$('dirtyState').style.color=dirty?'#f2b84b':'#6f8b97'}
function nextId(collection:J){return String(Math.max(0,...Object.keys(collection).map(Number))+1)}
function defaultVertex(pos:number[]){return{pos,rpy:[0,0,0],meta:{type:0,typeId:0,isCorner:false,isSlope:false,isJunction:false,chargingMode:0,source:'web_console',sourceStamp:Date.now()/1000,component:0,turnDeg:0,state:'confirmed'},pcd:'',acc:.5,turnable:true,alignFinalYaw:true,mustPassThrough:false,passRadiusM:.45}}
function addVertex(pos:number[]){if(!topology)return;checkpoint();const id=nextId(topology.vertices);topology.vertices[id]=defaultVertex(pos);selected={kind:'vertex',id};renderTopology();selected={kind:'vertex',id};renderInspector()}
function addEdge(a:string,b:string){if(!topology||a===b)return;if(Object.values<any>(topology.edges).some(e=>new Set(e.v.map(String)).has(a)&&new Set(e.v.map(String)).has(b))){toast('两点之间已经存在边',true);return}checkpoint();const id=nextId(topology.edges),pa=topology.vertices[a].pos,pb=topology.vertices[b].pos;topology.edges[id]={v:[Number(a),Number(b)],weight:Math.hypot(pa[0]-pb[0],pa[1]-pb[1],pa[2]-pb[2]),rotationAllowed:true,meta:{dir:0,source:'web_console',locomotionMode:0,linearSpeedMps:.8,angularSpeedRadps:0,heightOffsetM:0,obstacleMode:0,travelMode:'bidirectional',headingAngleRad:0,obstacleBoxM:[0,0,0,0],gridMapName:'',controllerMode:'auto'}};renderTopology()}

function pointerNdc(e:PointerEvent){const r=renderer.domElement.getBoundingClientRect();pointer.x=(e.clientX-r.left)/r.width*2-1;pointer.y=-(e.clientY-r.top)/r.height*2+1;raycaster.setFromCamera(pointer,camera)}
function groundPoint(e:PointerEvent,pickCloud=false){pointerNdc(e);if(pickCloud&&mapGroup.visible){const hit=raycaster.intersectObjects(mapGroup.children,false)[0];if(hit)return hit.point}const p=new THREE.Vector3();return raycaster.ray.intersectPlane(interactionPlane,p)?p:null}
renderer.domElement.addEventListener('pointerdown',e=>{if(e.button!==0)return;pointerNdc(e);if(tool==='initial'){
  // Match RViz's "2D Pose Estimate": project onto the map XY plane. Point-cloud
  // picking can accidentally select a wall or ceiling even when the cursor looks right.
  interactionPlane.constant=0;dragStart=groundPoint(e,false);if(dragStart)dragStart.z=0;controls.enabled=false;return
}const hits=raycaster.intersectObjects(topoGroup.children,false);if(hits.length){const u=hits[0].object.userData;if(tool==='edge'&&u.kind==='vertex'){if(!edgeStart){edgeStart=u.id;toast(`已选择起点 ${u.id}，再选择终点`)}else{addEdge(edgeStart,u.id);edgeStart=null}}else{selected={kind:u.kind,id:u.id};renderInspector();if(u.kind==='vertex'){if(!$<HTMLInputElement>('startId').value)$<HTMLInputElement>('startId').value=u.id;else $<HTMLInputElement>('goalId').value=u.id;if(tool==='select'){draggingVertex={id:u.id,object:hits[0].object};dragMoved=false;interactionPlane.constant=-topology.vertices[u.id].pos[2];controls.enabled=false}}}return}const p=groundPoint(e,true);if(tool==='vertex'&&p)addVertex([p.x,p.y,p.z])});
renderer.domElement.addEventListener('pointermove',e=>{if(draggingVertex){const p=groundPoint(e);if(!p)return;if(!dragMoved){checkpoint();dragMoved=true}draggingVertex.object.position.copy(p);topology.vertices[draggingVertex.id].pos=[p.x,p.y,p.z];renderInspector();return}if(!dragStart)return;const p=groundPoint(e);if(!p)return;if(yawArrow)scene.remove(yawArrow);const dir=p.clone().sub(dragStart);dir.z=0;if(dir.length()<.05)return;yawArrow=new THREE.ArrowHelper(dir.normalize(),dragStart,Math.min(2,dir.length()),0xffd84d);scene.add(yawArrow)});
renderer.domElement.addEventListener('pointerup',e=>{if(draggingVertex){draggingVertex=null;controls.enabled=true;if(dragMoved)renderTopology();return}if(!dragStart)return;const p=groundPoint(e);controls.enabled=true;if(p){const yaw=Math.atan2(p.y-dragStart.y,p.x-dragStart.x);send('localization.set_initial_pose',{position:[dragStart.x,dragStart.y,0],yaw})}if(yawArrow)scene.remove(yawArrow);yawArrow=null;dragStart=null;interactionPlane.constant=0;tool='select';setToolButton()});

$<HTMLFormElement>('vertexForm').onsubmit=e=>{e.preventDefault();if(!selected||selected.kind!=='vertex')return;checkpoint();const f=formValues(e.currentTarget as HTMLFormElement),v=topology.vertices[selected.id];v.pos=[+f.x,+f.y,+f.z];v.rpy=[+f.roll,+f.pitch,+f.yaw];v.acc=+f.acc;v.passRadiusM=+f.passRadiusM;v.meta.isCorner=f.isCorner;v.meta.isSlope=f.isSlope;v.meta.isJunction=f.isJunction;v.mustPassThrough=f.mustPassThrough;v.alignFinalYaw=f.alignFinalYaw;v.turnable=f.turnable;renderTopology();toast('顶点属性已应用')};
$<HTMLFormElement>('edgeForm').onsubmit=e=>{e.preventDefault();if(!selected||selected.kind!=='edge')return;checkpoint();const f=formValues(e.currentTarget as HTMLFormElement),x=topology.edges[selected.id];x.meta.dir=+f.dir;x.meta.travelMode=['bidirectional','first_to_second','second_to_first'][+f.dir];x.meta.controllerMode=f.controllerMode;x.meta.linearSpeedMps=+f.linearSpeedMps;x.meta.angularSpeedRadps=+f.angularSpeedRadps;x.meta.locomotionMode=+f.locomotionMode;x.meta.obstacleMode=+f.obstacleMode;x.meta.heightOffsetM=+f.heightOffsetM;x.rotationAllowed=f.rotationAllowed;renderTopology();toast('边属性已应用')};
$('deleteVertex').onclick=()=>{if(!selected||selected.kind!=='vertex'||!confirm(`删除顶点 ${selected.id} 及其所有关联边？`))return;checkpoint();const id=selected.id;delete topology.vertices[id];for(const [eid,e] of Object.entries<any>(topology.edges))if(e.v.map(String).includes(id))delete topology.edges[eid];renderTopology()};
$('deleteEdge').onclick=()=>{if(!selected||selected.kind!=='edge'||!confirm(`删除边 ${selected.id}？`))return;checkpoint();delete topology.edges[selected.id];renderTopology()};

document.querySelectorAll<HTMLButtonElement>('[data-start]').forEach(b=>b.onclick=()=>{if(b.dataset.start==='planner'&&$<HTMLInputElement>('enableMotion').checked&&!confirm('即将启用实机运动。确认遥控急停已就绪、机器人周围安全？'))return;send('process.start',{target:b.dataset.start,enable_motion:$<HTMLInputElement>('enableMotion').checked})});
document.querySelectorAll<HTMLButtonElement>('[data-stop]').forEach(b=>b.onclick=()=>send('process.stop',{target:b.dataset.stop}));
document.querySelectorAll<HTMLButtonElement>('[data-tool]').forEach(b=>b.onclick=()=>{tool=b.dataset.tool!;edgeStart=null;if(tool==='robotVertex'){if(currentPose)addVertex([...currentPose.position]);else toast('尚无机器人位置',true);tool='select'}setToolButton()});
function setToolButton(){document.querySelectorAll('[data-tool]').forEach(x=>x.classList.toggle('active',(x as HTMLElement).dataset.tool===tool));$('hint').textContent=tool==='initial'?'在地图上按住左键拖出朝向':tool==='edge'?'依次点击两个拓扑点创建边':tool==='vertex'?'点击地图平面添加拓扑点':'左键选择，右键旋转，滚轮缩放'}
$('initialPoseMode').onclick=()=>{tool='initial';setToolButton()};
$('refreshMaps').onclick=()=>send('map.list');$('selectMap').onclick=()=>{const id=$<HTMLSelectElement>('mapSelect').value;if(id)send('map.select',{id});else toast('请选择地图',true)};
$('selectTopology').onclick=()=>{const id=$<HTMLSelectElement>('topologySelect').value;if(id)send('topology.select',{id});else toast('请选择拓扑图',true)};
$('startRecording').onclick=()=>send('topology.record.start');
$('stopRecording').onclick=()=>{const name=$<HTMLInputElement>('recordingName').value.trim();if(!name){toast('请输入保存名称',true);return}send('topology.record.stop',{name})};
$('abortRecording').onclick=()=>{if(confirm('放弃本次自动打点？临时数据不会保存为正式拓扑。'))send('topology.record.abort')};
$('saveTopology').onclick=()=>topology&&send('topology.save',{data:topology,revision});
$('planRoute').onclick=()=>send('navigation.plan',{start_id:+$<HTMLInputElement>('startId').value,goal_id:+$<HTMLInputElement>('goalId').value});
$('pause').onclick=()=>send('navigation.pause');$('resume').onclick=()=>send('navigation.resume');$('cancel').onclick=()=>send('navigation.cancel');
$('claimControl').onclick=()=>hasControl?send('control.release'):send('control.claim');
$('undo').onclick=()=>{if(!undoStack.length)return;redoStack.push(JSON.stringify(topology));topology=JSON.parse(undoStack.pop()!);dirty=true;renderTopology();setDirty()};
$('redo').onclick=()=>{if(!redoStack.length)return;undoStack.push(JSON.stringify(topology));topology=JSON.parse(redoStack.pop()!);dirty=true;renderTopology();setDirty()};
$('clearLog').onclick=()=>logBox.textContent='';
$<HTMLInputElement>('showMap').onchange=e=>mapGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showLive').onchange=e=>liveGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showTopo').onchange=e=>topoGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showPreview').onchange=e=>previewGroup.visible=(e.target as HTMLInputElement).checked;$<HTMLInputElement>('showTrack').onchange=e=>trackGroup.visible=(e.target as HTMLInputElement).checked;
$('followRobot').onclick=()=>setRobotFollow(!followRobot);
$('fitView').onclick=()=>{const box=new THREE.Box3().setFromObject(mapGroup.children.length?mapGroup:topoGroup);if(box.isEmpty())return;setRobotFollow(false);const size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(size.length()*.55,-size.length()*.55,size.length()*.45));camera.near=Math.max(.01,size.length()/10000);camera.far=Math.max(100,size.length()*10);camera.updateProjectionMatrix()};
window.addEventListener('keydown',e=>{const target=e.target as HTMLElement;if(e.key.toLowerCase()==='f'&&!target.closest('input, select, textarea'))setRobotFollow(!followRobot)});
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue=''}});connect();resize();
