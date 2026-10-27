(() => {
  'use strict';
  const $=id=>document.getElementById(id);
  const form=$('vision-control-form'), result=$('vision-config-result'), badge=$('vision-control-badge');
  const storageKey='drone-vision-pid-v1';
  const fields={yaw_gain:'yaw-gain',yaw_ki:'yaw-ki',yaw_kd:'yaw-kd',yaw_speed_rad_s:'yaw-speed',
    vertical_gain:'vertical-gain',vertical_ki:'vertical-ki',vertical_kd:'vertical-kd',vertical_speed_m_s:'vertical-speed',
    distance_gain:'distance-gain',distance_ki:'distance-ki',distance_kd:'distance-kd',forward_speed_m_s:'forward-speed',
    lateral_gain:'lateral-gain',lateral_ki:'lateral-ki',lateral_kd:'lateral-kd',lateral_speed_m_s:'lateral-speed',
    tag_size_m:'tag-size',target_distance_m:'target-distance'};
  const flags={vertical_enabled:'vertical-enabled',distance_enabled:'distance-enabled',lateral_enabled:'lateral-enabled'};
  let flight=null, visionState=null, current=null, dirty=false, busy=false, defaultsReady=false;
  function active(){return flight?.motion?.state==='running'&&flight?.motion?.params?.kind==='vision_follow';}
  function parameters(){
    const value={};
    Object.entries(fields).forEach(([key,id])=>value[key]=Number($('vision-'+id).value));
    Object.entries(flags).forEach(([key,id])=>value[key]=$('vision-'+id).checked);
    return value;
  }
  function fill(value){
    Object.entries(fields).forEach(([key,id])=>{if(Number.isFinite(value[key]))$('vision-'+id).value=value[key];});
    Object.entries(flags).forEach(([key,id])=>{if(typeof value[key]==='boolean')$('vision-'+id).checked=value[key];});
    dirty=false;
  }
  function remember(){try{localStorage.setItem(storageKey,JSON.stringify(parameters()));}catch(_){}}
  try{const saved=JSON.parse(localStorage.getItem(storageKey));if(saved)fill(saved);}catch(_){}
  async function configRequest(method, value){
    const response=await fetch('/api/vision/control-config',{method,cache:'no-store',
      ...(method==='POST'?{headers:{'Content-Type':'application/json','X-Control-Request':'1','X-Client-ID':clientId},body:JSON.stringify(value)}:{}),
      signal:AbortSignal.timeout(5000)});
    const data=await response.json();if(!response.ok||!data.ok)throw new Error(data.message||'默认参数保存/读取失败');
    return data;
  }
  async function loadDefaults(){
    result.textContent='正在读取开发板默认参数…';
    try{
      const saved=await configRequest('GET');
      if(saved.stored&&!dirty&&!active())fill(saved.parameters);
      defaultsReady=true;
      result.textContent=saved.stored?'已读取开发板保存的默认参数。':'尚未保存开发板默认参数；修改后点击保存。';
    }catch(error){result.textContent=error.message+'；稍后重试读取。';setTimeout(loadDefaults,3000);}
    buttons();
  }
  document.querySelectorAll('[data-motion-panel]').forEach(button=>button.addEventListener('click',()=>{
    if(button.dataset.motionPanel!=='joystick')window.joystickPanel?.leave();
    document.querySelectorAll('[data-motion-panel]').forEach(item=>{
      const selected=item===button;item.classList.toggle('selected',selected);item.setAttribute('aria-pressed',String(selected));
      $('motion-panel-'+item.dataset.motionPanel).hidden=!selected;
    });
  }));
  function display(){
    const pose=visionState?.pose_valid&&visionState?.calibrated;
    const side=Number(visionState?.tag_size_m);
    const distance=pose&&side>0?visionState.distance_m*Number($('vision-tag-size').value)/side:null;
    const requested=Number($('vision-target-distance').value);
    const applied=active()?Number(flight.motion.params.target_distance_m):null;
    $('vision-distance-live').textContent='当前距离：'+(Number.isFinite(distance)?distance.toFixed(3)+' m':'等待目标')+
      (Number.isFinite(applied)?' · 当前执行目标：'+applied.toFixed(3)+' m':'');
    const error=Number.isFinite(distance)&&Number.isFinite(requested)?distance-requested:null;
    $('vision-distance-error').textContent=Number.isFinite(error)?
      '相对填写距离的偏差：'+(error>0?'+':'')+error.toFixed(3)+' m（'+(error>0?'需前进':error<0?'需后退':'已到目标距离')+'）':'距离偏差：等待目标';
    $('vision-calibration-status').textContent=visionState?.calibrated?
      '内参已加载 · '+visionState.width+'×'+visionState.height+' · 当前前向距离 '+(Number.isFinite(distance)?distance.toFixed(3)+' m':'等待目标位姿'):'等待已标定的视觉节点';
    const output=flight?.motion?.vision_output;
    $('vision-control-live').textContent=active()&&output?
      `控制输出：前后 ${Number(output.forward_m_s).toFixed(3)} m/s · 升降 ${Number(output.vertical_m_s).toFixed(3)} m/s · 偏航 ${Number(output.yaw_rad_s).toFixed(3)} rad/s`:'视觉控制未运行';
  }
  function buttons(){
    const running=active(), takingOff=flight?.motion?.params?.kind==='auto_takeoff';
    const other=flight?.motion?.state==='landing'||(flight?.motion?.state==='running'&&!running&&!takingOff)||flight?.joystick?.active;
    const blocked=[];
    const missionActive=!!window.missionPanel?.active();
    if(missionActive)blocked.push('比赛任务调度中，请在比赛编排中停止');
    if(!flight?.service?.connected)blocked.push('C++服务未连接');
    if(actionBusy)blocked.push('正在等待其他操作结果');
    if(other)blocked.push('需先结束其他任务或退出摇杆');
    $('vision-flight-start').disabled=busy||!defaultsReady||running||blocked.length>0;
    $('vision-flight-stop').disabled=busy||actionBusy||!running||missionActive;
    $('vision-apply').disabled=busy||actionBusy;
    $('vision-distance-apply').disabled=busy||actionBusy;
    $('vision-apply').textContent=running?'应用并保存为默认值':'保存为默认值';
    const toggle=$('vision-toggle-preview');toggle.disabled=busy||!current||running||missionActive;
    toggle.textContent=!current?'等待节点':current.preview_enabled?'停用偏航建议':'启用偏航建议';
    toggle.setAttribute('aria-pressed',String(!!current?.preview_enabled));
    $('vision-flight-note').textContent=running?flight.motion.phase==='handover'?
      '正在预发送位置保持，等待OFFBOARD确认。':'控制在开发板本地执行，关闭网页后继续；可修改PID、目标距离与速度并点击应用。':blocked.length?'暂不能启动：'+blocked.join('；'):'起飞后点击启动，将当前参数提交机载C++；没有目标时保持等待。';
    display();
  }
  form.addEventListener('input',()=>{dirty=true;display();});
  form.addEventListener('submit',async event=>{
    event.preventDefault();if(busy||!form.reportValidity())return;
    busy=true;buttons();
    try{
      let confirmed=parameters(), applied=false;
      if(active()&&!window.missionPanel?.active()){
        const response=await actualAction({action:'command',command:'VISION_UPDATE',params:confirmed});
        if(!response?.confirmed){result.textContent='未确认参数更新，未保存为默认值。';return;}
        confirmed=Object.fromEntries([...Object.keys(fields),...Object.keys(flags)].map(key=>[key,response.params[key]]));
        applied=true;
      }
      try{
        const saved=await configRequest('POST',confirmed);
        fill(saved.parameters);remember();defaultsReady=true;
        window.dispatchEvent(new CustomEvent('vision-defaults-changed'));
        result.textContent=(applied?'当前控制已应用；':'')+saved.message;
      }catch(error){result.textContent=(applied?'当前控制已应用，但默认值保存失败：':'默认值保存失败：')+error.message;}
    }catch(error){result.textContent=error.message;
    }finally{busy=false;buttons();}
  });
  $('vision-toggle-preview').onclick=async()=>{
    if(busy||!current)return;busy=true;buttons();
    try{
      const response=await fetch('/api/vision/preview',{method:'POST',cache:'no-store',
        headers:{'Content-Type':'application/json','X-Control-Request':'1','X-Client-ID':clientId},
        body:JSON.stringify({...current,preview_enabled:!current.preview_enabled}),signal:AbortSignal.timeout(3500)});
      const value=await response.json();if(!response.ok)throw new Error(value.message);
      result.textContent=value.message;
      if(value.applied)current=value.vision.preview_parameters;
    }catch(error){result.textContent=error.message;}finally{busy=false;buttons();}
  };
  $('vision-flight-form').onsubmit=async event=>{
    event.preventDefault();if($('vision-flight-start').disabled||!form.reportValidity())return;
    remember();busy=true;buttons();
    try{
      const saved=await configRequest('POST',parameters());fill(saved.parameters);remember();
      window.dispatchEvent(new CustomEvent('vision-defaults-changed'));
      await actualAction({action:'command',command:'VISION_START',params:saved.parameters});dirty=false;
    }
    catch(error){result.textContent=error.message;}
    finally{busy=false;buttons();}
  };
  $('vision-flight-stop').onclick=()=>actualAction({action:'command',command:'VISION_STOP',params:{}});
  window.motionWorkspace={renderFlight(snapshot){
    flight=snapshot;if(active()&&!dirty&&!busy)fill(flight.motion.params);buttons();
  },renderVision(vision){
    visionState=vision;current=vision.preview_parameters||null;
    badge.textContent=active()?'视觉控制运行中':vision.calibrated?'内参已接入':'等待视觉节点';buttons();
  }};
  buttons();
  window.addEventListener('vision-defaults-changed',()=>{if(!dirty&&!busy&&!active())loadDefaults();});
  loadDefaults();
})();
