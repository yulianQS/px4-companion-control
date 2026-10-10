'use strict';
// 页面只采集[-1,1]杆量。限速、坐标转换、过期处理与20Hz发送均在开发板完成。
(() => {
  let session = null, sequence = 0, primed = false, inFlight = false;
  let requestController = null, startedAt = 0, sawSession = false;
  const axes = {throttle: 0, yaw: 0, pitch: 0, roll: 0};
  const pads = [
    {side: 'left', x: 'yaw', y: 'throttle', pointer: null, keys: new Set()},
    {side: 'right', x: 'roll', y: 'pitch', pointer: null, keys: new Set()}
  ];
  const pageVisible = () => document.visibilityState === 'visible' && location.hash === '#motion';
  const flightReady = () => {
    const px4 = lastSnapshot?.px4 || {};
    return controlOwned && px4.connected && px4.bidirectional &&
      px4.mode === 'OFFBOARD' && !actionBusy && pageVisible();
  };
  const canDrag = () => !!session && primed && flightReady();

  function paint(pad) {
    const surface = $('stick-' + pad.side), knob = $('knob-' + pad.side);
    const radius = Math.max(1, surface.getBoundingClientRect().width / 2 - knob.offsetWidth / 2 - 7);
    knob.style.transform = `translate(${axes[pad.x] * radius}px, ${-axes[pad.y] * radius}px)`;
    for (const name of [pad.x, pad.y]) $('stick-' + name).textContent = Math.round(axes[name] * 100) + '%';
  }
  function reset(pad, keepThrottle = false) {
    const pointer = pad.pointer;
    pad.pointer = null;
    pad.keys.clear();
    axes[pad.x] = 0;
    if (!keepThrottle || pad.side !== 'left') axes[pad.y] = 0;
    const surface = $('stick-' + pad.side);
    surface.classList.remove('dragging');
    if (pointer !== null && surface.hasPointerCapture(pointer)) surface.releasePointerCapture(pointer);
    paint(pad);
  }
  function suspend() {
    const previous = session;
    session = null; primed = false; sawSession = false;
    if (requestController) requestController.abort();
    pads.forEach(pad => reset(pad));
    return previous;
  }
  function leave() {
    const previous = suspend();
    if (previous) postControl('action', {action: 'stick_stop', session: previous, request_id: uniqueId()}).catch(() => {});
  }
  function setAxes(pad, x, y) {
    const magnitude = Math.max(1, Math.hypot(x, y));
    axes[pad.x] = x / magnitude;
    axes[pad.y] = y / magnitude;
    paint(pad);
  }
  function pointerPosition(pad, event) {
    const rect = $('stick-' + pad.side).getBoundingClientRect();
    const radius = Math.max(1, rect.width / 2 - $('knob-' + pad.side).offsetWidth / 2 - 7);
    setAxes(pad, (event.clientX - rect.left - rect.width / 2) / radius,
      -(event.clientY - rect.top - rect.height / 2) / radius);
  }
  for (const pad of pads) {
    const surface = $('stick-' + pad.side);
    surface.addEventListener('pointerdown', event => {
      if (!canDrag() || pad.pointer !== null || (event.pointerType === 'mouse' && event.button !== 0)) return;
      event.preventDefault();
      pad.pointer = event.pointerId;
      pad.keys.clear();
      surface.setPointerCapture(event.pointerId);
      surface.classList.add('dragging');
      pointerPosition(pad, event);
    });
    surface.addEventListener('pointermove', event => {
      if (event.pointerId !== pad.pointer) return;
      if (!canDrag()) { reset(pad); return; }
      event.preventDefault(); pointerPosition(pad, event);
    });
    const endPointer = event => {
      if (event.pointerId !== pad.pointer) return;
      reset(pad, event.type === 'pointerup'); pulse();
    };
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) surface.addEventListener(name, endPointer);
    const arrows = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
    const keyboardPosition = () => {
      const x = (pad.keys.has('ArrowRight') ? 1 : 0) - (pad.keys.has('ArrowLeft') ? 1 : 0);
      if (pad.side === 'left') {
        // 升降轴按5%步进且松键保持；偏航松键回零，不改变已设升降杆量。
        axes[pad.x] = x * Math.sqrt(Math.max(0, 1 - axes[pad.y] ** 2));
        paint(pad);
      } else {
        setAxes(pad, x, (pad.keys.has('ArrowUp') ? 1 : 0) - (pad.keys.has('ArrowDown') ? 1 : 0));
      }
    };
    surface.addEventListener('keydown', event => {
      if (!arrows.includes(event.key) || !canDrag() || pad.pointer !== null) return;
      event.preventDefault(); pad.keys.add(event.key);
      if (pad.side === 'left' && ['ArrowUp', 'ArrowDown'].includes(event.key)) {
        axes[pad.y] = Math.max(-1, Math.min(1, axes[pad.y] + (event.key === 'ArrowUp' ? .05 : -.05)));
      }
      keyboardPosition();
    });
    surface.addEventListener('keyup', event => {
      if (!arrows.includes(event.key)) return;
      event.preventDefault(); pad.keys.delete(event.key); keyboardPosition(); pulse();
    });
    surface.addEventListener('blur', () => { if (pad.keys.size) { reset(pad); pulse(); } });
  }

  async function pulse() {
    if (!session || inFlight) return;
    if (!flightReady()) { leave(); return; }
    const sendingSession = session;
    const controller = new AbortController();
    requestController = controller; inFlight = true;
    const timeout = setTimeout(() => controller.abort(), 350);
    try {
      const response = await fetch('/api/control/stick', {method: 'POST', cache: 'no-store',
        headers: {'Content-Type': 'application/json', 'X-Control-Request': '1', 'X-Client-ID': clientId},
        body: JSON.stringify({session: sendingSession, sequence: sequence++, axes: primed ? {...axes} : {throttle: 0, yaw: 0, pitch: 0, roll: 0}}),
        signal: controller.signal});
      const result = await response.json();
      if (!response.ok) throw new Error(result.message || '摇杆输入未被接收');
      if (session === sendingSession) {
        primed = true;
        if (lastSnapshot && result.joystick) {
          lastSnapshot.joystick = result.joystick;
          lastSnapshot.control = {...lastSnapshot.control, source: 'joystick'};
        }
      }
    } catch (error) {
      if (session === sendingSession) {
        leave();
        toast('摇杆输入中断，已停止旧输入：' + error.message);
      }
    } finally {
      clearTimeout(timeout);
      if (requestController === controller) requestController = null;
      inFlight = false;
      updateControlButtons();
    }
  }

  function render(snapshot, available) {
    const state = snapshot?.joystick || {}, px4 = snapshot?.px4 || {};
    if (session) {
      if (state.active && state.session === session) sawSession = true;
      if (!flightReady() || ((sawSession || Date.now() - startedAt > 1000) && (!state.active || state.session !== session))) leave();
    }
    const task = snapshot?.motion || {};
    const protectedTask = task.state === 'landing';
    const ownActive = !!session;
    $('toggle-joystick').disabled = actionBusy || !available || (!ownActive &&
      (state.active || px4.mode !== 'OFFBOARD' || protectedTask));
    $('toggle-joystick').textContent = ownActive ? '退出摇杆 / 保持位置' : task.state === 'running' && !protectedTask ? '摇杆接管 / 取消当前任务' : '启用双摇杆';
    $('toggle-joystick').setAttribute('aria-pressed', String(ownActive));
    $('joystick-badge').textContent = ownActive ? (primed ? '摇杆控制中' : '等待中立输入') : state.active ? '已有摇杆会话' : '未接管';
    $('joystick-note').textContent = !controlOwned ? '先开启上方实机控制，再启用双摇杆。' :
      !px4.connected ? '飞控状态未知，不能启用摇杆。' :
      px4.mode !== 'OFFBOARD' ? '先切换OFFBOARD，再启用双摇杆。' :
      protectedTask ? 'PX4正在执行降落，摇杆不混入降落指令。' :
      state.state === 'failed' ? state.message :
      px4.armed === true ? '已解锁：升降杆松手保持，拖回中线停止升降；其余轴松手回中。' :
      '未解锁地面调试：可发送摇杆设定值，不自动解锁；实际动作需手动解锁。';
    const names = {monitor: '只读监测', zero: '零速度', hold: '位置保持', task: '自动运动', joystick: '双摇杆', px4_land: 'PX4降落'};
    $('control-source').textContent = names[snapshot?.control?.source] || '未知';
    $('stick-age').textContent = px4.connected && Number.isFinite(state.input_age_s) ? state.input_age_s.toFixed(2) + ' s' : '—';
    $('stick-command').textContent = px4.connected && Array.isArray(state.command_ned) ?
      state.command_ned.slice(0, 3).map(n => Number.isFinite(n) ? n.toFixed(2) : '—').join(' / ') + ' m/s' : '—';
    for (const pad of pads) $('stick-' + pad.side).setAttribute('aria-disabled', String(!canDrag()));
  }
  $('toggle-joystick').onclick = async () => {
    if (session) { await actualAction({action: 'stick_stop', session}); return; }
    suspend();
    const result = await actualAction({action: 'stick_start'});
    if (result?.ok && result.session && flightReady()) {
      session = result.session; sequence = 0; startedAt = Date.now(); sawSession = false;
      pulse();
    }
  };
  window.addEventListener('blur', leave);
  window.addEventListener('pagehide', leave);
  document.addEventListener('visibilitychange', () => { if (document.visibilityState !== 'visible') leave(); });
  window.joystickPanel = {render, suspend, leave};
  pads.forEach(paint);
  updateControlButtons();
  setInterval(pulse, 100);
})();
