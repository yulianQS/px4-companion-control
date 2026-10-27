// 无浏览器的DOM逻辑检查；不代替实际渲染/移动端验证。
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const root = require('path').join(__dirname, '..') + '/';
const html = fs.readFileSync(root+'web/index.html', 'utf8');
class Element {
  constructor(tag = 'div') {
    this.tagName=tag; this.children=[]; this.dataset={}; this.style={}; this.attributes={};
    this.value=''; this.textContent=''; this.hidden=false; this.disabled=false; this.listeners={};
    this.classList={toggle(){},add(){},remove(){}};
  }
  append(...items){this.children.push(...items); if(this.tagName==='select' && !this.value && items.length) this.value=items[0].value;}
  replaceChildren(...items){this.children=[...items]; if(this.tagName==='select') this.value='';}
  addEventListener(event, fn){this.listeners[event]=fn;}
  setAttribute(key, val){this.attributes[key]=val;}
  reportValidity(){return true;}
  getBoundingClientRect(){return {left:0,top:0,width:this.id==='stick-throttle-control'?88:this.id==='stick-yaw-control'?220:250,height:this.id==='stick-throttle-control'?250:this.id==='stick-yaw-control'?88:250};}
  get offsetWidth(){return 54;}
  get offsetHeight(){return 54;}
  setPointerCapture(id){this.capture=id;}
  hasPointerCapture(id){return this.capture===id;}
  releasePointerCapture(id){if(this.capture===id)this.capture=null;}
  get selectedOptions(){return this.children.filter(c=>c.value===this.value);}
}
const elements={};
for(const match of html.matchAll(/<([\w-]+)\b[^>]*\bid="([^"]+)"[^>]*>/g)) {
  const element=elements[match[2]]=new Element(match[1]);
  element.id=match[2];
  element.value=match[0].match(/\bvalue="([^"]*)"/)?.[1] || '';
}
elements.direction.append(Object.assign(new Element('option'),{value:'forward',textContent:'向前'}));
const nav=['overview','motion','tests','mission','parameters','records'].map(page=>Object.assign(new Element('button'),{dataset:{page}}));
const types=['move','rotate','height'].map(kind=>Object.assign(new Element('button'),{dataset:{kind}}));
let raf=null, now=0;
const docListeners={},windowListeners={},intervals=[];
const document={visibilityState:'visible',addEventListener:(name,fn)=>docListeners[name]=fn,
  getElementById:id=>{assert(elements[id], 'Missing DOM ID '+id);return elements[id];},
  createElement:tag=>new Element(tag), createElementNS:(_,tag)=>new Element(tag), createTextNode:text=>({textContent:text}),
  querySelectorAll:selector=>selector==='nav button'?nav:selector==='#task-types button'?types:selector==='.page'?Object.entries(elements).filter(([id])=>id.startsWith('page-')).map(([,el])=>el):[]
};
const context=vm.createContext({document,location:{hash:'#motion',hostname:'192.168.137.3',protocol:'http:'},
  crypto:require('crypto').webcrypto,AbortController,window:{addEventListener:(name,fn)=>windowListeners[name]=fn},performance:{now:()=>now},fetch:async()=>({ok:true,json:async()=>({source:'unavailable'})}),
  AbortSignal:{timeout(){}},setTimeout:()=>1,clearTimeout(){},setInterval:(fn,ms)=>intervals.push({fn,ms}),
  requestAnimationFrame:fn=>{raf=fn;return 1;},cancelAnimationFrame:()=>{raf=null;},
  Date,Number,Math,JSON,Blob,URL,FormData:class{constructor(){return new Map([['gain','0.3'],['max_speed','0.2'],['deadzone','15'],['lost_timeout','0.5']]);}}});
vm.runInContext(fs.readFileSync(root+'web/app.js','utf8'),context);


function evaluate(code){return vm.runInContext(code,context);}
const requests=[];
context.fetch = async (url, options) => {
  if(!options?.method) return {ok:true,json:async()=>({source:'unavailable'})};
  const body=JSON.parse(options.body); requests.push({url,body});
  if(url.endsWith('/stick')) return {ok:true,json:async()=>({ok:true,sequence:body.sequence,joystick:{active:true,session:body.session,message:'双摇杆控制',input_age_s:0,command_ned:[0,0,0,0]}})};
  return {ok:true,json:async()=>({ok:true,confirmed:true,session:'test-stick-session',message:'已处理'})};
};
const waitMicrotasks=()=>new Promise(setImmediate);
(async()=>{
  await waitMicrotasks();
  vm.runInContext(fs.readFileSync(root+'web/map3d.js','utf8'),context);
  evaluate('renderSnapshot({px4:{connected:true,position_age_s:0.1,local_position:{x:1,y:2,z:-3},position_sample_ms:1,attitude_deg:[0,0,0],attitude_age_s:0.1,estimator_flags:46,estimator_age_s:0.1},motion:{state:"idle"}})');
  assert(!elements['trajectory-frame'].src || elements['trajectory-frame'].src === 'about:blank'); // No automatic CSV load.
  vm.runInContext(fs.readFileSync(root+'web/joystick.js','utf8'),context);
  const setup='controlOwned=true; controlUntil=Date.now()+20000; lastSnapshot={px4:{connected:true,bidirectional:true,armed:true,mode:"OFFBOARD",landed_state:2},control:{source:"zero"},motion:{state:"idle"},joystick:{active:false}}; updateControlButtons();';
  evaluate(setup);
  assert.equal(elements['toggle-joystick'].disabled,false);
  await elements['toggle-joystick'].onclick(); await waitMicrotasks();
  assert.equal(requests[0].body.action,'stick_start');
  assert.equal(requests[1].body.sequence,0);
  assert.deepEqual(requests[1].body.axes,{throttle:0,yaw:0,pitch:0,roll:0});
  assert.equal(elements['stick-throttle-control'].attributes['aria-disabled'],'false');
  const pointer=(id,x,y)=>({pointerId:id,pointerType:'touch',button:0,clientX:x,clientY:y,preventDefault(){}});
  const left=elements['stick-throttle-control'],yaw=elements['stick-yaw-control'],right=elements['stick-right'];
  left.listeners.pointerdown(pointer(1,44,34));
  yaw.listeners.pointerdown(pointer(7,186,44));
  right.listeners.pointerdown(pointer(2,216,125));
  assert.equal(elements['stick-throttle'].textContent,'100%');
  assert.equal(elements['stick-yaw'].textContent,'100%');
  assert.equal(elements['stick-roll'].textContent,'100%');
  assert.equal(elements['knob-throttle-control'].style.transform,'translate(0px, -91px)');
  assert.equal(elements['knob-yaw-control'].style.transform,'translate(76px, 0px)');
  assert(left.hasPointerCapture(1));assert(yaw.hasPointerCapture(7));assert(right.hasPointerCapture(2));
  const pulse=intervals.find(x=>x.ms===100).fn;
  await pulse();
  assert.deepEqual(requests.at(-1).body.axes,{throttle:1,yaw:1,pitch:0,roll:1});
  left.listeners.pointerup({...pointer(1,44,34),type:'pointerup'});await waitMicrotasks();
  assert.deepEqual(requests.at(-1).body.axes,{throttle:1,yaw:1,pitch:0,roll:1});
  yaw.listeners.pointerup({...pointer(7,186,44),type:'pointerup'});await waitMicrotasks();
  assert.deepEqual(requests.at(-1).body.axes,{throttle:1,yaw:0,pitch:0,roll:1});
  right.listeners.pointercancel(pointer(2,216,125));await waitMicrotasks();
  assert.deepEqual(requests.at(-1).body.axes,{throttle:1,yaw:0,pitch:0,roll:0});
  // 横向偏离竖杆不会改变偏航；竖向偏离偏航杆不会改变已保持的升降。
  left.listeners.pointerdown(pointer(3,999,80));await pulse();
  const heldThrottle=requests.at(-1).body.axes.throttle;
  assert(heldThrottle>0 && heldThrottle<1);assert.equal(requests.at(-1).body.axes.yaw,0);
  left.listeners.pointerup({...pointer(3,999,80),type:'pointerup'});await waitMicrotasks();
  yaw.listeners.pointerdown(pointer(8,34,-999));await pulse();
  assert.equal(requests.at(-1).body.axes.throttle,heldThrottle);assert.equal(requests.at(-1).body.axes.yaw,-1);
  yaw.listeners.pointerup({...pointer(8,34,-999),type:'pointerup'});await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.throttle,heldThrottle);assert.equal(requests.at(-1).body.axes.yaw,0);
  left.listeners.pointerdown(pointer(4,44,125));
  left.listeners.pointerup({...pointer(4,44,125),type:'pointerup'});await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.throttle,0);
  left.listeners.keydown({key:'ArrowUp',preventDefault(){}});await pulse();
  assert.equal(requests.at(-1).body.axes.throttle,.05);
  left.listeners.keyup({key:'ArrowUp',preventDefault(){}});await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.throttle,.05);
  yaw.listeners.keydown({key:'ArrowRight',preventDefault(){}});await pulse();
  assert.equal(requests.at(-1).body.axes.yaw,1);assert.equal(requests.at(-1).body.axes.throttle,.05);
  yaw.listeners.keyup({key:'ArrowRight',preventDefault(){}});await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.throttle,.05);assert.equal(requests.at(-1).body.axes.yaw,0);
  left.listeners.pointerdown(pointer(5,44,34));
  left.listeners.pointercancel({...pointer(5,44,34),type:'pointercancel'});await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.throttle,0);
  left.listeners.pointerdown(pointer(6,44,80));
  left.listeners.pointerup({...pointer(6,44,80),type:'pointerup'});await waitMicrotasks();
  assert(requests.at(-1).body.axes.throttle>0);
  const seqs=requests.filter(x=>x.url.endsWith('/stick')).map(x=>x.body.sequence);
  assert(seqs.every((seq,i)=>!i||seq>seqs[i-1]));
  assert(elements['start-task'].disabled);
  assert(!elements['land-flight'].disabled);
  right.listeners.keydown({key:'ArrowUp',preventDefault(){}}); await pulse();
  assert.equal(requests.at(-1).body.axes.pitch,1);
  right.listeners.keyup({key:'ArrowUp',preventDefault(){}}); await waitMicrotasks();
  assert.equal(requests.at(-1).body.axes.pitch,0);
  windowListeners.blur(); await waitMicrotasks();
  assert.equal(requests.at(-1).body.action,'stick_stop');
  assert.equal(elements['stick-throttle'].textContent,'0%');
  const before=requests.length; await pulse(); assert.equal(requests.length,before);
  evaluate(setup);
  elements['takeoff-height'].value='0.5';
  evaluate('lastSnapshot.px4.landed_state=1;lastSnapshot.px4.takeoff_alt_m=.5;lastSnapshot.px4.takeoff_speed_m_s=.2;lastSnapshot.px4.takeoff_alt_age_s=0;lastSnapshot.px4.takeoff_speed_age_s=0;updateControlButtons();');
  assert(!elements['takeoff-flight'].disabled);
  assert.equal(elements['takeoff-height'].value,'0.5');
  elements['takeoff-height'].value='0.65';
  evaluate('updateControlButtons();');
  assert.equal(elements['takeoff-height'].value,'0.65');
  elements['takeoff-height'].value='6';
  evaluate('updateControlButtons();');
  assert(elements['takeoff-flight'].disabled);
  elements['takeoff-height'].value='0.5';
  assert.equal(elements['takeoff-speed'].value,'0.20');
  evaluate('lastSnapshot.px4.takeoff_alt_age_s=31;updateControlButtons();');
  assert(elements['takeoff-flight'].disabled);
  evaluate('lastSnapshot.motion={state:"landing"};updateControlButtons();');
  assert(elements['toggle-joystick'].disabled);
  evaluate('lastSnapshot.motion={state:"running",params:{kind:"auto_takeoff"}};updateControlButtons();');
  assert(elements['toggle-joystick'].disabled);
  assert(elements['cancel-task'].disabled);
  assert(!elements['land-flight'].disabled);
  evaluate('lastSnapshot.motion={state:"running",params:{kind:"move"}};updateControlButtons();');
  assert(elements['toggle-joystick'].disabled);
  assert(elements['joystick-note'].textContent.includes('先停止'));
  evaluate('lastSnapshot.px4.armed=false;lastSnapshot.px4.landed_state=1;lastSnapshot.motion={state:"idle"};updateControlButtons();');
  assert(!elements['toggle-joystick'].disabled);
  assert(elements['joystick-note'].textContent.includes('未解锁地面调试'));
  await elements['toggle-joystick'].onclick(); await waitMicrotasks();
  assert.equal(elements['stick-throttle-control'].attributes['aria-disabled'],'false');
  assert(!requests.some(request=>request.body.action==='arm'));
  assert.deepEqual(requests.at(-1).body.axes,{throttle:0,yaw:0,pitch:0,roll:0});
  console.log('PASS: 独立升降/偏航单轴、非方形控件坐标/旋钮、三指同时控制、升降保持/手动回中/步进、取消/失焦、新会话中立及任务协调');
})().catch(error=>{console.error(error);process.exitCode=1;});
