'use strict';
// Existing browser GUI: collect parameters only. The C++ service owns execution.
(() => {
  const numeric = id => Number($(id).value);
  const command = (name, params = {}) => actualAction({action:'command', command:name, params});
  async function connection(path, body) {
    try {
      const response = await fetch('/api/service/' + path, {method:'POST',
        headers:{'Content-Type':'application/json','X-Control-Request':'1','X-Client-ID':clientId},
        body:JSON.stringify(body),signal:AbortSignal.timeout(4000)});
      const result = await response.json();
      if (!response.ok) throw new Error(result.message);
      controlOwned = false; controlUntil = 0; window.joystickPanel?.suspend();
      toast(result.message);
    } catch(error) { toast(error.message); }
  }
  $('cpp-connect').onclick = () => connection('connect',{host:$('cpp-host').value.trim(),port:numeric('cpp-port')});
  $('cpp-disconnect').onclick = () => connection('disconnect',{});
  $('cpp-position-form').onsubmit = event => {
    event.preventDefault();
    command('SET_POSITION',{frame:'LOCAL_NED',unit:'cm',x:numeric('cpp-x'),y:numeric('cpp-y'),z:numeric('cpp-z'),speed:.2,tolerance:.05,timeout:60});
  };
  $('cpp-velocity-form').onsubmit = event => {
    event.preventDefault();
    command('SET_VELOCITY',{frame:$('cpp-velocity-frame').value,vx:numeric('cpp-vx'),vy:numeric('cpp-vy'),vz:numeric('cpp-vz'),duration:numeric('cpp-velocity-duration')});
  };
  $('cpp-hold').onclick = $('cpp-velocity-stop').onclick = () => command('HOLD');
  $('cpp-stop-trajectory').onclick = () => command('STOP_TRAJECTORY');
  function fields() {
    const type = $('cpp-trajectory-type').value;
    document.querySelectorAll('[data-for]').forEach(el => {
      el.hidden = !el.dataset.for.split(' ').includes(type);
      el.querySelectorAll('input,textarea').forEach(input => { input.disabled = el.hidden; });
    });
  }
  $('cpp-trajectory-type').onchange = fields; fields();
  $('cpp-trajectory-form').onsubmit = event => {
    event.preventDefault();
    const trajectory = $('cpp-trajectory-type').value;
    const params = {trajectory,height:numeric('cpp-traj-height'),speed:.2,tolerance:.05,timeout:numeric('cpp-traj-timeout')};
    if (trajectory === 'hover') params.duration = numeric('cpp-traj-duration');
    if (['line','rectangle'].includes(trajectory)) params.length = numeric('cpp-traj-length');
    if (trajectory === 'rectangle') params.width = numeric('cpp-traj-width');
    if (trajectory === 'circle') Object.assign(params,{radius:numeric('cpp-traj-radius'),angular_speed_deg:numeric('cpp-traj-angular'),turns:numeric('cpp-traj-turns')});
    if (trajectory === 'waypoints') {
      try { Object.assign(params,{waypoints:JSON.parse($('cpp-waypoints').value),frame:'LOCAL_NED',unit:'cm'}); }
      catch { toast('航点必须是JSON数组，例如[[0,0,-50],[50,0,-50]]'); return; }
    }
    command('EXECUTE_TRAJECTORY',params);
  };
  setInterval(() => {
    const service = lastSnapshot?.service || {}, px4 = lastSnapshot?.px4 || {}, control = lastSnapshot?.control || {};
    $('cpp-connection').textContent = service.connected ? 'C++已连接' : 'C++未连接';
    $('cpp-connection').className = 'tag ' + (service.connected ? 'cyan' : 'amber');
    $('cpp-status').textContent = `${service.host || '127.0.0.1'}:${service.port || 8090} · ` +
      (service.connected ? `飞控 ${px4.connected ? '在线' : '离线'} / ${px4.mode || '未知'} / ${px4.armed === true ? '已解锁' : px4.armed === false ? '未解锁' : '状态未知'} · EKF标志 ${px4.estimator_flags ?? '—'}` : service.error || '等待C++控制服务');
    if (document.activeElement !== $('cpp-host') && service.host) $('cpp-host').value = service.host;
    if (document.activeElement !== $('cpp-port') && service.port) $('cpp-port').value = service.port;
    const missionActive = !!window.missionPanel?.active();
    $('cpp-connect').disabled = $('cpp-disconnect').disabled = missionActive;
    const available = !missionActive && !lastSnapshot?.joystick?.active && controlOwned && service.connected && px4.connected && px4.bidirectional && !actionBusy && px4.mode === 'OFFBOARD' && !(lastSnapshot?.motion?.state === 'running' && lastSnapshot?.motion?.params?.kind === 'vision_follow') && !(lastSnapshot?.motion?.state === 'running' && lastSnapshot?.motion?.params?.kind === 'auto_takeoff');
    document.querySelectorAll('[data-cpp-flight]').forEach(el => {el.disabled = !available || px4.armed !== true || lastSnapshot?.motion?.state === 'landing';});
    document.querySelectorAll('[data-cpp-hold]').forEach(el => {el.disabled = !available || lastSnapshot?.motion?.state === 'landing';});
    $('cpp-target').textContent = 'C++目标 LOCAL_NED / m：' + (control.setpoint?.position || []).map(n => Number.isFinite(n) ? n.toFixed(3) : '速度轴').join(' / ');
    const event = lastSnapshot?.command_events?.at(-1);
    if (event) $('cpp-command-state').textContent = JSON.stringify(event,null,2);
  },300);
})();
