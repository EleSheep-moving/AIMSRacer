import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';

const $=id=>document.getElementById(id);
const clone=value=>JSON.parse(JSON.stringify(value));
let project={schema_version:1,frame_id:'map',coordinate_reference:'rear_axle',map_sha256:'',waypoints:[],left_width:.5,right_width:.5,calibration_regions:[]};
let history=[],future=[],selected=-1,mode='add',map=null,preview=null,drag=null,threeD=false,markStart=null,revision=0;
const viewport=$('viewport');
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setClearColor(0x0b1420);viewport.appendChild(renderer.domElement);
const scene=new THREE.Scene();let span=12;
const camera=new THREE.OrthographicCamera(-8,8,6,-6,.01,10000);camera.position.set(0,0,100);camera.up.set(0,1,0);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableRotate=false;
controls.mouseButtons={LEFT:null,MIDDLE:THREE.MOUSE.PAN,RIGHT:THREE.MOUSE.PAN};controls.target.set(0,0,0);controls.update();
const plane=new THREE.Plane(new THREE.Vector3(0,0,1),0),ray=new THREE.Raycaster();
const mapGroup=new THREE.Group(),pathGroup=new THREE.Group();scene.add(mapGroup,pathGroup);
const axes=new THREE.AxesHelper(2);axes.material.depthTest=false;axes.renderOrder=2;scene.add(axes);
let mapObject=null;
function disposeGroup(group){while(group.children.length){const item=group.children[0];group.remove(item);item.geometry?.dispose();item.material?.dispose();}}
function status(text,error=false){$('status').textContent=text;$('status').classList.toggle('error',error);}
function resize(){const w=viewport.clientWidth,h=viewport.clientHeight;renderer.setSize(w,h);const aspect=w/h;camera.left=-span*aspect/2;camera.right=span*aspect/2;camera.top=span/2;camera.bottom=-span/2;camera.updateProjectionMatrix();drawPlot();}
new ResizeObserver(resize).observe(viewport);
function scale(){const width=(camera.right-camera.left)/camera.zoom;const metre=viewport.clientWidth/width;const step=10**Math.floor(Math.log10(100/metre));$('scale').style.width=`${step*metre}px`;$('scale').textContent=`${step.toPrecision(2)} m`;}
function animate(){requestAnimationFrame(animate);controls.update();scale();renderer.render(scene,camera);}animate();
function remember(){history.push(clone(project));if(history.length>200)history.shift();future=[];}
function dirty(){revision++;preview=null;markStart=null;$('download').hidden=true;$('metrics').textContent='Preview needs validation';drawPlot();renderPath();update();}
function update(){$('points-info').textContent=`${project.waypoints.length} waypoints · selected ${selected<0?'none':selected+1} · start = point 1`;$('undo').disabled=!history.length;$('redo').disabled=!future.length;$('left-width').value=project.left_width;$('right-width').value=project.right_width;renderRegions();}
function setMode(value){mode=value;markStart=null;for(const id of ['add','move','insert','delete','mark'])$(id).classList.toggle('active',id===mode);}
for(const id of ['add','move','insert','delete','mark'])$(id).onclick=()=>{if(id==='mark'&&!preview){status('Validate the curve before marking progress intervals.',true);return;}setMode(id);};
function line(points,color){const geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(p[0],p[1],.03)));const item=new THREE.Line(geometry,new THREE.LineBasicMaterial({color,depthTest:false}));item.renderOrder=10;pathGroup.add(item);return item;}
function renderPath(){disposeGroup(pathGroup);const points=project.waypoints;if(!points.length)return;
  line([...points,points[0]],0x496f89);
  const geometry=new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(p[0],p[1],.05)));
  const colors=points.flatMap((_,i)=>new THREE.Color(i===selected?0xffc857:i===0?0x85f0aa:0xd5e6ff).toArray());geometry.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));pathGroup.add(new THREE.Points(geometry,new THREE.PointsMaterial({size:10,sizeAttenuation:false,vertexColors:true,depthTest:false})));
  if(preview){line([...preview.samples.map(p=>[p.x,p.y]),[preview.samples[0].x,preview.samples[0].y]],0x43c6ee);
    for(const region of project.calibration_regions){const inside=preview.samples.filter(p=>region.start_s<region.end_s?p.s>=region.start_s&&p.s<=region.end_s:p.s>=region.start_s||p.s<=region.end_s);
      // Split wrapping intervals at the seam to avoid drawing a false chord.
      if(region.start_s>region.end_s){line(inside.filter(p=>p.s>=region.start_s).map(p=>[p.x,p.y]),0xffb760);line(inside.filter(p=>p.s<=region.end_s).map(p=>[p.x,p.y]),0xffb760);}else line(inside.map(p=>[p.x,p.y]),0xffb760);
    }
  }
  pathGroup.traverse(item=>{item.renderOrder=10;});
  const first=new THREE.Vector3(points[0][0],points[0][1],.08);let direction;
  if(preview){const p=preview.samples[Math.min(5,preview.samples.length-1)];direction=new THREE.Vector3(p.x-first.x,p.y-first.y,0);}else if(points.length>1){direction=new THREE.Vector3(points[1][0]-first.x,points[1][1]-first.y,0);}
  if(direction&&direction.length()>1e-9){const arrow=new THREE.ArrowHelper(direction.normalize(),first,Math.max(.3,span/camera.zoom*.06),0x85f0aa);arrow.traverse(item=>{item.renderOrder=11;if(item.material)item.material.depthTest=false;});pathGroup.add(arrow);}
}
function filterMap(){disposeGroup(mapGroup);if(!map)return;const lo=Number($('z-min').value),hi=Number($('z-max').value);if(!Number.isFinite(lo)||!Number.isFinite(hi)||lo>hi){status('Z filter needs finite minimum ≤ maximum.',true);return;}
  const points=map.points.filter(p=>p[2]>=lo&&p[2]<=hi);const positions=new Float32Array(points.flat());const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
  const colors=new Float32Array(points.flatMap(p=>{const t=Math.max(0,Math.min(1,(p[2]-map.bounds[0][2])/Math.max(.1,map.bounds[1][2]-map.bounds[0][2])));return new THREE.Color().setHSL(.58-.2*t,.3,.38+.18*t).toArray();}));geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));
  mapObject=new THREE.Points(geometry,new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true}));mapGroup.add(mapObject);
  $('map-info').textContent=`${map.original_count.toLocaleString()} original points · ${points.length.toLocaleString()} visible · voxel 0.05 m · SHA ${map.sha256}`;
}
function fit(){const pts=project.waypoints;let low,high;if(map){[low,high]=map.bounds;}else if(pts.length){low=[Math.min(...pts.map(p=>p[0])),Math.min(...pts.map(p=>p[1]))];high=[Math.max(...pts.map(p=>p[0])),Math.max(...pts.map(p=>p[1]))];}else{low=[-5,-5];high=[5,5];}
  const aspect=viewport.clientWidth/viewport.clientHeight;span=Math.max(3,high[1]-low[1],(high[0]-low[0])/aspect)*1.2;controls.target.set((low[0]+high[0])/2,(low[1]+high[1])/2,0);camera.zoom=1;orient();resize();}
function orient(){const target=controls.target;camera.up.set(0,threeD?0:1,threeD?1:0);camera.position.copy(target).add(threeD?new THREE.Vector3(span,-span,span):new THREE.Vector3(0,0,Math.max(100,span*2)));camera.lookAt(target);controls.enableRotate=threeD;controls.mouseButtons.LEFT=threeD?THREE.MOUSE.ROTATE:null;controls.update();$('view-label').textContent=threeD?'3D inspect · rotate left · pan middle · Z up':'XY top view · +X right · +Y up · metres';$('view-mode').textContent=threeD?'XY draw':'3D inspect';}
$('fit').onclick=fit;$('view-mode').onclick=()=>{threeD=!threeD;orient();};$('z-min').onchange=filterMap;$('z-max').onchange=filterMap;
function world(event){const bounds=renderer.domElement.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((event.clientX-bounds.left)/bounds.width*2-1,-(event.clientY-bounds.top)/bounds.height*2+1),camera);const p=ray.ray.intersectPlane(plane,new THREE.Vector3());return p?[p.x,p.y]:null;}
function screen(point){const bounds=renderer.domElement.getBoundingClientRect();const p=new THREE.Vector3(point[0],point[1],0).project(camera);return [bounds.left+(p.x+1)*bounds.width/2,bounds.top+(1-p.y)*bounds.height/2];}
function pointHit(event){let index=-1,best=15;project.waypoints.forEach((p,i)=>{const q=screen(p),d=Math.hypot(q[0]-event.clientX,q[1]-event.clientY);if(d<best){index=i;best=d;}});return index;}
function segmentHit(event){let best=18,index=-1;project.waypoints.forEach((p,i)=>{const a=screen(p),b=screen(project.waypoints[(i+1)%project.waypoints.length]),dx=b[0]-a[0],dy=b[1]-a[1];const t=Math.max(0,Math.min(1,((event.clientX-a[0])*dx+(event.clientY-a[1])*dy)/(dx*dx+dy*dy)));const d=Math.hypot(event.clientX-a[0]-t*dx,event.clientY-a[1]-t*dy);if(d<best){best=d;index=i;}});return index;}
renderer.domElement.addEventListener('pointerdown',event=>{
  if(threeD||event.button!==0)return;const p=world(event);if(!p)return;const hit=pointHit(event);
  if(mode==='mark'){
    if(!preview)return;let best=Infinity,progress=0;for(const s of preview.samples){const d=Math.hypot(s.x-p[0],s.y-p[1]);if(d<best){best=d;progress=s.s;}}
    if(markStart===null){markStart=progress;$('region-start').value=progress.toFixed(3);status('Interval start selected. Click the end on the validated curve.');}else{$('region-end').value=progress.toFixed(3);markStart=null;status('Interval endpoints selected. Add interval to retain them.');}return;
  }
  if(mode==='add'){remember();project.waypoints.push(p);selected=project.waypoints.length-1;dirty();}
  else if(mode==='move'&&hit>=0){selected=hit;remember();drag=hit;controls.enabled=false;renderer.domElement.setPointerCapture(event.pointerId);renderPath();update();}
  else if(mode==='delete'&&hit>=0){remember();project.waypoints.splice(hit,1);selected=-1;dirty();}
  else if(mode==='insert'){const segment=segmentHit(event);if(segment>=0){remember();project.waypoints.splice(segment+1,0,p);selected=segment+1;dirty();}else status('Click within 18 screen pixels of a control chord to insert.',true);}
});
renderer.domElement.addEventListener('pointermove',event=>{if(drag===null)return;const p=world(event);if(p){project.waypoints[drag]=p;dirty();}});
renderer.domElement.addEventListener('pointerup',()=>{drag=null;controls.enabled=true;});renderer.domElement.addEventListener('pointercancel',()=>{drag=null;controls.enabled=true;});
$('undo').onclick=()=>{if(!history.length)return;future.push(clone(project));project=history.pop();selected=-1;dirty();};$('redo').onclick=()=>{if(!future.length)return;history.push(clone(project));project=future.pop();selected=-1;dirty();};
$('clear').onclick=()=>{remember();project.waypoints=[];project.calibration_regions=[];selected=-1;dirty();};
$('reverse').onclick=()=>{if(project.waypoints.length<2)return;remember();project.waypoints=[project.waypoints[0],...project.waypoints.slice(1).reverse()];project.calibration_regions=[];dirty();status('Direction reversed; progress intervals cleared.');};
$('start').onclick=()=>{if(selected<0){status('Select a waypoint with Move mode first.',true);return;}remember();project.waypoints=[...project.waypoints.slice(selected),...project.waypoints.slice(0,selected)];selected=0;project.calibration_regions=[];dirty();status('Startpoint changed; progress intervals cleared.');};
for(const id of ['left-width','right-width'])$(id).onchange=()=>{remember();project[id==='left-width'?'left_width':'right_width']=Number($(id).value);dirty();};
async function api(path,data){const options=data instanceof FormData?{method:'POST',body:data}:data===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)};const response=await fetch(path,options);const result=await response.json();if(!response.ok)throw new Error(typeof result.detail==='string'?result.detail:JSON.stringify(result.detail));return result;}
async function validate(){const current=revision;status('Validating exact quintic curve…');const result=await api('/api/preview',project);if(revision!==current){status('Project changed during validation. Validate again.');return null;}preview=result;renderPath();drawPlot();$('metrics').textContent=`Length ${result.length.toFixed(2)} m · max |curvature| ${result.max_curvature.toFixed(3)} 1/m · max |steering| ${result.max_steering_rad.toFixed(3)} / ${result.steer_limit_rad.toFixed(3)} rad · ${result.samples.length} checks`;status('Valid reference. '+result.validation);return result;}
$('preview').onclick=()=>validate().catch(error=>status(error.message,true));
function drawPlot(){const canvas=$('plot'),w=canvas.clientWidth,h=150;canvas.width=w*devicePixelRatio;canvas.height=h*devicePixelRatio;const ctx=canvas.getContext('2d');ctx.scale(devicePixelRatio,devicePixelRatio);ctx.fillStyle='#131f2b';ctx.fillRect(0,0,w,h);ctx.font='12px system-ui';if(!preview){ctx.fillStyle='#90a9c0';ctx.fillText('Validate to show |curvature| (1/m), |steering| (rad) and planned speed (m/s)',12,24);return;}
  const keys=[['curvature','|κ| 1/m','#46caef'],['steering_rad','|δ| rad','#ffd078'],['speed','v m/s','#9bedac']];keys.forEach(([key,label,color],index)=>{const maximum=Math.max(.001,...preview.samples.map(p=>Math.abs(p[key])));ctx.fillStyle=color;ctx.fillText(`${label} max ${maximum.toFixed(3)}`,12+index*(w/3),18);ctx.strokeStyle=color;ctx.beginPath();preview.samples.forEach((p,i)=>{const x=12+p.s/preview.length*(w-24),y=h-18-Math.abs(p[key])/maximum*(h-48);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();});ctx.fillStyle='#8da7bf';ctx.fillText(`s = 0 → ${preview.length.toFixed(2)} m · independent vertical scales`,12,h-3);
}
function renderRegions(){$('regions').replaceChildren();project.calibration_regions.forEach((region,index)=>{const li=document.createElement('li');li.textContent=`${region.name}: s ${region.start_s.toFixed(2)} → ${region.end_s.toFixed(2)} m`;const remove=document.createElement('button');remove.textContent='×';remove.title='Delete interval';remove.onclick=()=>{remember();project.calibration_regions.splice(index,1);revision++;renderRegions();renderPath();$('download').hidden=true;};li.append(remove);$('regions').append(li);});}
$('region-add').onclick=()=>{if(!preview){status('Validate before adding an interval.',true);return;}const start=Number($('region-start').value),end=Number($('region-end').value),name=$('region-name').value.trim();if(!name||!Number.isFinite(start)||!Number.isFinite(end)||start<0||end<0||start>=preview.length||end>=preview.length||start===end){status('Interval name and distinct progress endpoints within [0,length) required.',true);return;}const segments=(a,b)=>a<b?[[a,b]]:[[a,preview.length],[0,b]];if(project.calibration_regions.some(r=>segments(start,end).some(a=>segments(r.start_s,r.end_s).some(b=>Math.max(a[0],b[0])<Math.min(a[1],b[1]))))){status('Trial intervals overlap; use distinct progress windows.',true);return;}remember();project.calibration_regions.push({id:crypto.randomUUID(),name,start_s:start,end_s:end,kind:'straight'});revision++;$('download').hidden=true;renderRegions();renderPath();status('Trial interval retained. Curvature and steering preview remain available.');};
function download(value,name){const blob=new Blob([JSON.stringify(value,null,2)+'\n'],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
$('save-project').onclick=()=>download(project,'reference-project.json');
function reopen(data){if(data.schema_version!==1||!Array.isArray(data.waypoints)||!Array.isArray(data.calibration_regions))throw new Error('Invalid editable project schema');remember();project=clone(data);selected=-1;dirty();if(map&&map.sha256!==project.map_sha256)status('Project map differs from displayed PCD. Load its original PCD before export.',true);else status('Project reopened. Validate before export.');}
$('project-file').onchange=async event=>{try{const file=event.target.files[0];if(file)reopen(JSON.parse(await file.text()));}catch(error){status(error.message,true);}finally{event.target.value='';}};
$('reference-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;const form=new FormData();form.append('file',file);form.append('replan_speed',String($('replan-speed').checked));reopen(await api('/api/reference',form));status('Reference loaded. Exact original waypoints retained; validate edits before export.');}catch(error){status(error.message,true);}finally{event.target.value='';}};
$('map-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;status('Loading original PCD and preparing display voxels…');const form=new FormData();form.append('file',file);map=await api('/api/map',form);
    if(project.waypoints.length&&project.map_sha256&&project.map_sha256!==map.sha256){filterMap();fit();status('Displayed map differs from this project. Project identity retained; reopen a matching project or clear points to start on this map.',true);}
    else{remember();project.map_sha256=map.sha256;dirty();filterMap();fit();status('Original map loaded. Draw forward rear-axle waypoints in XY view.');}
  }catch(error){status(error.message,true);}finally{event.target.value='';}};
$('export').onclick=async()=>{let exportRevision=revision;try{if(!map||map.sha256!==project.map_sha256)throw new Error('Load the project’s matching original PCD before export.');if(!await validate())return;exportRevision=revision;const exportProject=clone(project);const data=await api('/api/export',{project:exportProject,name:$('bundle-name').value});if(revision!==exportRevision)return;$('download').href=`/api/bundle/${encodeURIComponent(data.directory)}`;$('download').textContent=`Download ${data.directory}.zip`;$('download').hidden=false;status(`Exported ${data.directory} · reference SHA ${data.reference_sha256}`);}catch(error){if(revision===exportRevision)status(error.message,true);}};
window.addEventListener('keydown',event=>{if(['INPUT','TEXTAREA'].includes(document.activeElement.tagName))return;if((event.ctrlKey||event.metaKey)&&event.key==='z'){event.preventDefault();$(event.shiftKey?'redo':'undo').click();}else if((event.ctrlKey||event.metaKey)&&event.key==='y'){event.preventDefault();$('redo').click();}});
api('/api/session').then(data=>{if(data.map){map=data.map;project.map_sha256=map.sha256;filterMap();}if(data.project)reopen(data.project);fit();update();}).catch(error=>status(error.message,true));resize();update();
