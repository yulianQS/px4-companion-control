// UI arbitration only: no network requests or physical flight commands.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {value:'',checked:false,disabled:false,hidden:false,
    listeners:{},classList:{toggle(){}},setAttribute(){},reportValidity(){return true;},
    addEventListener(name,fn){const previous=this.listeners[name];this.listeners[name]=previous ? event=>{previous(event);return fn(event);} : fn;}});
  return elements.get(id);
}
const tabs = ['vision','joystick','task'].map(name => Object.assign(element('tab-'+name),{dataset:{motionPanel:name}}));
let leaves=0; const actions=[],configs=[];
const context = {document:{getElementById:element,querySelectorAll:()=>tabs},
  window:{addEventListener(){},dispatchEvent(){},joystickPanel:{leave(){leaves++;}}}, CustomEvent:class {}, controlOwned:true,actionBusy:false,clientId:'test',
  actualAction:async action=>actions.push(action),AbortSignal,
  fetch:async(url,options)=>{if(options.method==='POST')configs.push(JSON.parse(options.body));return {ok:true,json:async()=>({ok:true,stored:true,parameters:configs.at(-1)||{yaw_gain:.3,tag_size_m:.12,target_distance_m:1}})};}};
vm.createContext(context);
vm.runInContext(fs.readFileSync('web/motion_workspace.js','utf8'),context);
const ui=context.window.motionWorkspace;
const flight={service:{connected:true},px4:{connected:true,bidirectional:true,armed:true,mode:'OFFBOARD',landed_state:2},motion:{state:'idle'},joystick:{active:false}};
const vision={target_found:true,camera_ready:true,preview_parameters:{yaw_gain:.3,max_yaw_rate:.15,
  deadzone_normalized:.05,max_target_age_seconds:.3,camera_upright_confirmed:true,preview_enabled:true}};
(async()=>{
await new Promise(resolve=>setImmediate(resolve));
ui.renderFlight(flight); ui.renderVision(vision);
context.controlOwned=false;ui.renderFlight(flight);
assert(!element('vision-flight-start').disabled,'No webpage lease gate');
assert(!element('vision-flight-start').disabled);
tabs[2].listeners.click(); assert.equal(leaves,1); assert.equal(actions.length,0);
assert(element('motion-panel-vision').hidden); assert(!element('motion-panel-task').hidden);
for (const state of [{state:'landing'},{state:'running',params:{kind:'move'}}]) {
  ui.renderFlight({...flight,motion:state}); assert(element('vision-flight-start').disabled);
}
ui.renderFlight({...flight,joystick:{active:true}}); assert(element('vision-flight-start').disabled);
ui.renderFlight({...flight,motion:{state:'running',params:{kind:'auto_takeoff'}}});
assert(!element('vision-flight-start').disabled,'Explicit vision click can request native takeoff handover');
ui.renderFlight({...flight,motion:{state:'running',params:{kind:'vision_follow'}}});
assert(element('vision-flight-start').disabled); assert(!element('vision-flight-stop').disabled);
assert(!element('vision-apply').disabled,'Running vision permits applying and saving PID'); assert(element('vision-toggle-preview').disabled);
ui.renderFlight(flight); ui.renderVision({...vision,target_found:false});
assert(!element('vision-flight-start').disabled,'Backend validates target instead of silently disabling button');
ui.renderVision(vision);
ui.renderVision({...vision,preview_image_ready:false,preview_parameters:{...vision.preview_parameters,preview_enabled:false}});
assert(!element('vision-flight-start').disabled,'Preview disabled/missing JPEG must not block valid backend control');
ui.renderFlight({...flight,px4:{...flight.px4,armed:false,landed_state:1,mode:'LAND'}});
assert(!element('vision-flight-start').disabled,'Backend must explain invalid flight state on request');
ui.renderFlight(flight);
for(const [id,value] of Object.entries({'vision-vertical-gain':'.15','vision-vertical-limit':'.1'})) element(id).value=value;
element('vision-vertical-enabled').checked=true;
  element('vision-yaw-gain').value='.4';
  element('vision-control-form').listeners.input();
  await element('vision-flight-form').onsubmit({preventDefault(){},target:element('vision-flight-form')});
  assert.equal(configs.length,1,'Start persists shared defaults once');
  assert.equal(actions.at(-1).command,'VISION_START');
  assert.equal(actions.at(-1).params.yaw_gain,.4);
  assert(!('camera_mount_confirmed' in actions.at(-1).params));
  assert.equal(actions.at(-1).params.vertical_enabled,true);
  for (const key of ['max_vertical_speed','max_yaw_rate','deadzone','lost_timeout']) assert(!(key in actions.at(-1).params));
  assert(!('timeout' in actions.at(-1).params));
  assert(!('max_vertical_excursion' in actions.at(-1).params));
  assert(!('distance' in actions.at(-1).params));
  console.log('PASS: one-click parameter apply/start; no duplicate mounting/preview gates; backend validates flight/target; task isolation; yaw/vertical payload');
})().catch(e=>{console.error(e);process.exitCode=1;});
