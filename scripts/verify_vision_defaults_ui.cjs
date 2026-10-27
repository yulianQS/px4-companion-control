'use strict';
// Run the actual UI component with a fake DOM/HTTP store. Never sends flight commands.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../web/motion_workspace.js'),'utf8');
const factory={yaw_gain:.3,yaw_ki:0,yaw_kd:0,yaw_speed_rad_s:0,
  vertical_gain:.15,vertical_ki:0,vertical_kd:0,vertical_speed_m_s:0,
  distance_gain:.3,distance_ki:0,distance_kd:0,forward_speed_m_s:0,
  lateral_gain:.15,lateral_ki:0,lateral_kd:0,lateral_speed_m_s:0,lateral_enabled:false,
  target_distance_m:1,tag_size_m:.12,vertical_enabled:true,distance_enabled:true};
let stored={...factory,yaw_gain:.8}, confirmed=true, flightCalls=0, failSave=false;
function openPage(){
  const ids=new Map();
  function element(id){
    if(!ids.has(id))ids.set(id,{value:'0',checked:true,textContent:'',disabled:false,
      handlers:{},addEventListener(name,handler){this.handlers[name]=handler;},setAttribute(){},reportValidity(){return true;}});
    return ids.get(id);
  }
  element('vision-tag-size').value='.12';
  const context={document:{getElementById:element,querySelectorAll(){return [];}},window:{addEventListener(){},dispatchEvent(){}},CustomEvent:class{constructor(type){this.type=type;}},
    localStorage:{getItem(){return JSON.stringify({...factory,yaw_gain:.3});},setItem(){}},
    actionBusy:false,clientId:'test-client',AbortSignal,setTimeout,
    async fetch(url,options){
      assert.equal(url,'/api/vision/control-config');
      if(options.method==='POST'){
        if(failSave)return {ok:false,json:async()=>({ok:false,message:'test disk error'})};
        stored=JSON.parse(options.body);
      }
      return {ok:true,json:async()=>({ok:true,stored:true,parameters:structuredClone(stored),message:'已保存为默认值'})};
    },
    async actualAction(body){flightCalls++;return {confirmed,params:{...body.params,kind:'vision_follow'}};}};
  vm.createContext(context);vm.runInContext(source,context);
  return {context,element,async ready(){await new Promise(resolve=>setImmediate(resolve));}};
}
async function run(){
  let page=openPage();await page.ready();
  assert.equal(page.element('vision-yaw-gain').value,.8,'Server defaults override old browser cache');
  page.element('vision-yaw-gain').value='0.9';page.element('vision-yaw-ki').value='0.02';
  page.element('vision-control-form').handlers.input();
  await page.element('vision-control-form').handlers.submit({preventDefault(){}});
  assert.equal(stored.yaw_gain,.9);assert.equal(stored.yaw_ki,.02);assert.equal(flightCalls,0);
  page=openPage();await page.ready();assert.equal(page.element('vision-yaw-gain').value,.9,'Reopened page retains confirmed values');
  page.context.window.motionWorkspace.renderFlight({motion:{state:'running',params:{...stored,kind:'vision_follow'}},service:{connected:true}});
  page.element('vision-yaw-gain').value='1.1';page.element('vision-control-form').handlers.input();
  confirmed=false;
  await page.element('vision-control-form').handlers.submit({preventDefault(){}});
  assert.equal(stored.yaw_gain,.9,'Unconfirmed active update must not overwrite defaults');
  confirmed=true;failSave=true;
  await page.element('vision-control-form').handlers.submit({preventDefault(){}});
  assert.match(page.element('vision-config-result').textContent,/当前控制已应用，但默认值保存失败/);
  assert.equal(stored.yaw_gain,.9);
  failSave=false;
  await page.element('vision-control-form').handlers.submit({preventDefault(){}});
  assert.equal(stored.yaw_gain,1.1);
  page=openPage();await page.ready();assert.equal(page.element('vision-yaw-gain').value,1.1);
  page.element('vision-yaw-gain').value='1.2';page.element('vision-flight-start').disabled=false;
  await page.element('vision-flight-form').onsubmit({preventDefault(){}});
  assert.equal(stored.yaw_gain,1.2,'Starting with edited values also persists defaults');
  page=openPage();await page.ready();assert.equal(page.element('vision-yaw-gain').value,1.2);
  page.context.window.missionPanel={active:()=>true};
  page.context.window.motionWorkspace.renderFlight({motion:{state:'running',params:{...stored,kind:'vision_follow'}},service:{connected:true}});
  const before=flightCalls;
  page.element('vision-yaw-gain').value='1.3';page.element('vision-control-form').handlers.input();
  await page.element('vision-control-form').handlers.submit({preventDefault(){}});
  assert.equal(stored.yaw_gain,1.3);assert.equal(flightCalls,before,'During missions, save common defaults without bypassing C++ command arbitration');
  console.log('PASS: server defaults, inactive save, reopen, confirmed live update, failed apply/save reporting');
}
run().catch(error=>{console.error(error);process.exitCode=1;});
