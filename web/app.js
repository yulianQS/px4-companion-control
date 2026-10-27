'use strict';
const $ = id => document.getElementById(id);
const pages = {
  overview: ['飞行总览', '看清状态，再开始下一项任务。'],
  motion: ['运动控制', '起降、视觉、摇杆与轨迹集中操作；右侧持续查看相机和任务状态。'],
  tests: ['测试中心', '每一项通过，都有对应的证据与范围。'],
  mission: ['比赛编排', '从单项动作，组合出你的比赛流程。'],
  parameters: ['参数工作台', '视觉策略与飞控控制器，分开理解、分开调节。'],
  records: ['本次记录', '只保留当前页面的操作摘要。']
};
const records = [];
let kind = 'move', toastTimer;
let observedMotion = '';
let statusSource = 'offline', lastSnapshot = null;
let controlOwned = false, actionBusy = false, controlUntil = 0;
let observedServerAction = '';
const uniqueId = () => Date.now().toString(36) + '-' + Array.from(crypto.getRandomValues(new Uint8Array(12)), n => n.toString(16).padStart(2, '0')).join('');
const clientId = uniqueId();

function toast(message) {
  $('toast').textContent = message;
  $('toast').classList.add('visible');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => $('toast').classList.remove('visible'), 3500);
}
function navigate(page) {
  if (page !== 'motion') window.joystickPanel?.leave();
  if (!pages[page]) page = 'motion';
  document.querySelectorAll('.page').forEach(el => el.classList.toggle('active', el.id === 'page-' + page));
  document.querySelectorAll('nav button').forEach(el => el.classList.toggle('active', el.dataset.page === page));
  $('breadcrumb').textContent = pages[page][0];
  $('page-title').replaceChildren(document.createTextNode(pages[page][0]));
  const dot = document.createElement('span'); dot.className = 'title-dot'; dot.textContent = '.';
  $('page-title').append(dot);
  $('page-subtitle').textContent = pages[page][1];
  $('open-motion').hidden = page === 'motion';
}
document.querySelectorAll('nav button').forEach(button => button.addEventListener('click', () => { location.hash = button.dataset.page; }));
window.addEventListener('hashchange', () => navigate(location.hash.slice(1)));
$('open-motion').onclick = $('next-motion').onclick = () => { location.hash = 'motion'; };
navigate(location.hash.slice(1));

function addRecord(message, type = 'browser_preview') {
  records.unshift({time: new Date().toISOString(), type, message});
  if (records.length > 50) records.pop();
  renderRecords();
}
function renderRecords() {
  $('records-list').replaceChildren();
  if (!records.length) {
    const el = document.createElement('div'); el.className = 'empty'; el.textContent = '还没有操作记录。模拟一项任务后，摘要会出现在这里。';
    $('records-list').append(el); return;
  }
  records.forEach(item => {
    const row = document.createElement('div'); row.className = 'record';
    const time = document.createElement('time'); time.textContent = new Date(item.time).toLocaleTimeString('zh-CN', {hour12: false});
    const text = document.createElement('span'); text.textContent = item.message;
    row.append(time, text); $('records-list').append(row);
  });
}
$('export-records').onclick = () => {
  const blob = new Blob([JSON.stringify({version: '0.2', records}, null, 2)], {type: 'application/json'});
  const url = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = url; a.download = 'flight-console-session.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  toast('已导出本页操作记录');
};
renderRecords();

// ==================== 数据展示：只显示新鲜真实遥测，不填演示值 ====================
function value(id, number, digits = 2, suffix = '') {
  $(id).textContent = Number.isFinite(number) ? number.toFixed(digits) + suffix : '—';
}
function readiness(rows) {
  $('readiness').replaceChildren();
  rows.forEach(([name, note, tag, style]) => {
    const row = document.createElement('div'); row.className = 'readiness-row';
    const icon = document.createElement('span'); icon.className = 'check-icon'; icon.textContent = style === 'cyan' ? '✓' : '·';
    const copy = document.createElement('div'), title = document.createElement('b'), detail = document.createElement('small');
    title.textContent = name; detail.textContent = note; copy.append(title, detail);
    const badge = document.createElement('span'); badge.className = 'tag ' + style; badge.textContent = tag;
    row.append(icon, copy, badge); $('readiness').append(row);
  });
}
function renderSnapshot(snapshot) {
  const px4 = snapshot.px4 || {}, vision = snapshot.vision || {};
  const fresh = (key, seconds = 3) => px4.connected && Number.isFinite(px4[key + '_age_s']) && px4[key + '_age_s'] < seconds;
  const position = fresh('position', 2) ? px4.local_position || {} : {};
  $('source-badge').textContent = px4.connected ? '真实飞控 · 实时' : '飞控离线 / 等待心跳';
  $('source-badge').className = 'tag ' + (px4.connected ? 'cyan' : 'amber');
  $('monitor-note').textContent = px4.message || '状态服务不可用，不显示演示数值。';
  $('metric-mode').textContent = px4.connected ? px4.mode || '未知' : '未知';
  $('metric-arm').textContent = !px4.connected ? '解锁状态未知' : px4.armed === false ? '未解锁' : px4.armed === true ? '已解锁' : '解锁状态未知';
  value('metric-battery', fresh('battery') ? px4.battery_v : null, 2);
  $('battery-caption').textContent = fresh('battery') ? '实际电压遥测' : '电压未知 / 过期';
  value('metric-height', Number.isFinite(position.z) ? -position.z : null);
  value('metric-heartbeat', px4.heartbeat_age_s, 2);
  $('link-caption').textContent = px4.connected ? '心跳新鲜' : '飞控心跳失联';
  ['roll', 'pitch', 'yaw'].forEach((id, index) => value(id, fresh('attitude') ? px4.attitude_deg?.[index] : null, 1, '°'));
  ['x', 'y', 'z'].forEach(axis => value('pos-' + axis, position[axis]));
  ['x', 'y', 'z'].forEach(axis => value('vel-' + axis, position['v' + axis]));
  value('battery-current', fresh('battery') ? px4.battery_current_a : null);
  value('battery-remaining', fresh('battery') ? px4.battery_remaining_pct : null, 0);
  value('cpu-load', fresh('battery') ? px4.cpu_load_pct : null, 1);
  value('range-m', fresh('range') ? px4.range_m : null);
  value('flow-quality', fresh('flow') ? px4.flow_quality : null, 0);
  $('vision-center').textContent = vision.target_found && Array.isArray(vision.center) ? '中心坐标 ' + vision.center.map(n => Math.round(n)).join(', ') : '中心坐标 —';
  $('target-text').textContent = 'APRILTAG 36h11 / ID ' + (vision.tag_id ?? 0) + (vision.target_found ? ' / 已识别' : '');
  // Vision controls and video are updated by their independent /api/vision/status poller.
  const estimatorRecent = fresh('estimator', 6) && fresh('position', 2);
  const velocityFlags = (px4.estimator_flags & 6) === 6 && !(px4.estimator_flags & 128);
  readiness([
    ['MAVLink 双向链路', 'UART7 / 115200', px4.bidirectional ? '已验证' : '未确认', px4.bidirectional ? 'cyan' : 'amber'],
    ['速度估计', '按新鲜遥测标志观察', estimatorRecent && velocityFlags ? '标志有效' : '需检查', estimatorRecent && velocityFlags ? 'cyan' : 'amber'],
    ['航向与定位适用性', '历史航向有效标志未通过', '待验证', 'amber'],
    ['视觉目标', 'AprilTag 36h11 / ID 0', vision.target_found ? '发现目标' : vision.camera_ready ? '未发现目标' : '画面不可用', vision.target_found ? 'cyan' : ''],
    ['Offboard 设定值流', snapshot.control?.zero_error || '由开发板后台以20Hz发送', snapshot.control?.zero_stream ? '发送中' : '停止', snapshot.control?.zero_stream ? 'cyan' : '']
  ]);
  const final = snapshot.control?.last_action;
  const fingerprint = JSON.stringify(final || null);
  if (final && fingerprint !== observedServerAction && !actionBusy) $('action-result').textContent = JSON.stringify(final, null, 2);
  if (final) observedServerAction = fingerprint;
  renderMotion(snapshot.motion, !!px4.connected);
  window.flightMap3D?.update(snapshot);
  updateControlButtons();
  window.motionWorkspace?.renderFlight(snapshot);
}
renderSnapshot({});
async function pollStatus() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store', signal: AbortSignal.timeout(3500)});
    if (!response.ok) throw new Error('状态服务不可用');
    const result = await response.json();
    if (result.source !== 'px4' || !result.data) throw new Error('状态源无效');
    statusSource = 'px4'; lastSnapshot = result.data; renderSnapshot(result.data);
  } catch {
    statusSource = 'offline'; lastSnapshot = null; renderSnapshot({});
    $('monitor-note').textContent = '连接不可用；真实状态未知，不自动重发控制请求。';
  }
  setTimeout(pollStatus, lastSnapshot?.joystick?.active || ['running', 'landing'].includes(lastSnapshot?.motion?.state) ? 500 : 1000);
}
pollStatus();
setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString('zh-CN', {hour12: false}); }, 1000);
$('camera-connect').onclick = () => {
  toast(lastSnapshot?.vision?.message || '启动 ros2 launch drone_vision vision.launch.py；参数见 config/vision.yaml');
};

// ==================== 真实操作：一个启用开关，后台串行提交 ====================
function updateControlButtons() {
  if (Date.now() >= controlUntil) controlOwned = false;
  const px4 = lastSnapshot?.px4 || {}, available = !window.missionPanel?.active() && controlOwned && px4.connected && px4.bidirectional && !actionBusy;
  $('toggle-control').disabled = actionBusy || !!window.missionPanel?.active();
  $('toggle-control').textContent = controlOwned ? '关闭实机控制' : '启用实机控制';
  $('toggle-control').setAttribute('aria-pressed', String(controlOwned));
  $('set-mode').disabled = !available || px4.armed !== false;
  $('arm-flight').disabled = !available || px4.armed !== false || px4.mode !== 'OFFBOARD';
  $('disarm-flight').disabled = !available || px4.armed !== true;
  const activeTask = ['running', 'landing'].includes(lastSnapshot?.motion?.state);
  const sticksActive = !!lastSnapshot?.joystick?.active;
  $('start-task').disabled = !available || px4.armed !== true || px4.mode !== 'OFFBOARD' || activeTask || sticksActive;
  const takeoffConfig = Number.isFinite(px4.takeoff_alt_m) && px4.takeoff_alt_m > 0 &&
    Number.isFinite(px4.takeoff_speed_m_s) && px4.takeoff_speed_m_s > 0 &&
    Number.isFinite(px4.takeoff_alt_age_s) && px4.takeoff_alt_age_s < 30 &&
    Number.isFinite(px4.takeoff_speed_age_s) && px4.takeoff_speed_age_s < 30;
  const requestedHeight = Number($('takeoff-height').value);
  const validHeight = Number.isFinite(requestedHeight) && requestedHeight >= .1 && requestedHeight <= 5;
  $('takeoff-height').disabled = actionBusy || activeTask || sticksActive;
  $('takeoff-speed').value = takeoffConfig ? px4.takeoff_speed_m_s.toFixed(2) : '';
  const autoTakeoff = activeTask && lastSnapshot?.motion?.params?.kind === 'auto_takeoff';
  $('takeoff-flight').disabled = !available || px4.armed !== true || activeTask || sticksActive ||
    px4.landed_state !== 1 || px4.mode === 'LAND' || !takeoffConfig || !validHeight;
  $('land-flight').disabled = !available || px4.armed !== true;
  $('cancel-task').disabled = !available || lastSnapshot?.motion?.state !== 'running' || sticksActive || autoTakeoff;
  $('control-badge').textContent = controlOwned ? (actionBusy ? '等待飞控反馈' : '实机控制已开启') : '只读观察';
  window.joystickPanel?.render(lastSnapshot, available && !autoTakeoff);
}
async function postControl(path, data) {
  const controller = new AbortController(), timeoutId = setTimeout(() => controller.abort(), 12000);
  try {
    const response = await fetch('/api/control/' + path, {method: 'POST', cache: 'no-store',
      headers: {'Content-Type': 'application/json', 'X-Control-Request': '1', 'X-Client-ID': clientId},
      body: JSON.stringify(data), signal: controller.signal});
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || '控制接口拒绝请求');
    return result;
  } finally { clearTimeout(timeoutId); }
}
$('toggle-control').onclick = async () => {
  if (controlOwned) { await actualAction({action: 'disable'}); return; }
  actionBusy = true; updateControlButtons();
  try {
    await postControl('claim', {}); controlOwned = true; controlUntil = Date.now() + 18000;
    toast('实机控制已开启');
  } catch (error) { controlOwned = false; toast(error.message); }
  actionBusy = false;
  updateControlButtons();
};
setInterval(async () => {
  if (!controlOwned) return;
  try { await postControl('renew', {}); controlUntil = Date.now() + 18000; }
  catch { controlOwned = false; toast('连接中断，实机控制已暂停；恢复后重新开启'); }
  updateControlButtons();
}, 5000);
async function actualAction(body) {
  if (window.missionPanel?.active()) { toast('比赛任务执行中，请在比赛编排中停止或降落'); return; }
  if (actionBusy) return;
  if (body.action === 'command' || ['land', 'disable', 'disarm', 'release', 'mode', 'arm', 'stick_stop'].includes(body.action)) window.joystickPanel?.suspend();
  actionBusy = true; updateControlButtons();
  $('action-result').textContent = '正在提交C++服务；命令接受、执行中与完成分别显示，完成需飞控反馈。';
  try {
    const result = await postControl('action', {...body, request_id: uniqueId()});
    $('action-result').textContent = JSON.stringify(result, null, 2);
    addRecord('实机 ' + body.action + '：' + result.message, 'hardware_action');
    if (['release', 'disable'].includes(body.action) && result.confirmed) controlOwned = false;
    toast(result.message);
    return result;
  } catch (error) {
    $('action-result').textContent = error.message + '\n若连接中断，执行结果可能未知；请观察新心跳，页面不会自动重发。';
    addRecord('实机请求异常：' + error.message, 'hardware_action');
  } finally {
    actionBusy = false; updateControlButtons();
  }
}
$('set-mode').onclick = () => actualAction({action: 'mode', mode: $('flight-mode').value});
$('arm-flight').onclick = () => actualAction({action: 'arm'});
$('disarm-flight').onclick = () => actualAction({action: 'disarm'});

// ==================== 实机运动：提交任务，进度完全来自后端遥测 ====================
const configs = {
  move: {directions: [['forward', '向前'], ['right', '向右'], ['back', '向后'], ['left', '向左']], distance: [1, 0.1, 5, 0.1], speed: [0.2, 0.000001, '', 'any'], tolerance: [0.05, 0.01, 0.5, 0.01], label: '目标距离 / m', speedLabel: '速度上限 / m/s', unit: 'm', text: '方向以任务开始时的机头为准；运动途中改变航向，不改变既定方向。'},
  rotate: {directions: [['clockwise', '向右转（顺时针，俯视）'], ['counterclockwise', '向左转（逆时针，俯视）']], distance: [90, 1, 360, 1], speed: [30, 0.000001, '', 'any'], tolerance: [3, 1, 15, 1], label: '相对转角 / °', speedLabel: '角速度上限 / °/s', unit: '°', text: '以任务开始时的航向为零点，按实际偏航反馈累计转角；不需要绝对北向。'},
  height: {directions: [['up', '上升'], ['down', '下降']], distance: [0.5, 0.1, 2, 0.1], speed: [0.1, 0.000001, '', 'any'], tolerance: [0.05, 0.01, 0.3, 0.01], label: '相对高度变化 / m', speedLabel: '垂直速度上限 / m/s', unit: 'm', text: '相对当前局部高度，不是离地高度；实机执行需要额外检查高度与地面限制。'}
};
function setInput(id, settings) { const el = $(id); ['value', 'min', 'max', 'step'].forEach((key, index) => { el[key] = settings[index]; }); }
function selectKind(next) {
  kind = next; const config = configs[kind];
  document.querySelectorAll('#task-types button').forEach(button => button.classList.toggle('selected', button.dataset.kind === kind));
  $('direction').replaceChildren();
  config.directions.forEach(([code, label]) => { const option = document.createElement('option'); option.value = code; option.textContent = label; $('direction').append(option); });
  setInput('distance', config.distance); setInput('speed', config.speed); setInput('tolerance', config.tolerance);
  $('distance-label').textContent = config.label; $('speed-label').textContent = config.speedLabel;
  $('frame-explanation').textContent = config.text;
}
document.querySelectorAll('#task-types button').forEach(button => button.onclick = () => selectKind(button.dataset.kind));
$('task-form').onsubmit = event => {
  event.preventDefault(); if (!$('task-form').reportValidity()) return;
  actualAction({action: 'task_start', task: {kind, direction: $('direction').value,
    distance: Number($('distance').value), speed: Number($('speed').value),
    tolerance: Number($('tolerance').value), timeout: Number($('timeout').value)}});
};
$('cancel-task').onclick = () => actualAction({action: 'task_cancel'});
$('takeoff-height').addEventListener('input', updateControlButtons);
$('takeoff-flight').onclick = () => actualAction({action: 'takeoff', task: {
  distance: Number($('takeoff-height').value), speed: Number($('takeoff-speed').value),
  tolerance: 0.05, timeout: 30}});
$('land-flight').onclick = () => actualAction({action: 'land'});

function renderMotion(motion, connected) {
  const labels = {idle: '待机', running: '真实执行中', landing: 'PX4降落中',
    succeeded: '反馈确认完成', cancelled: '已取消', timed_out: '超时', failed: '已停止'};
  if (!connected || (['running', 'landing'].includes(motion?.state) && (!Number.isFinite(motion.feedback_age_s) || motion.feedback_age_s > 1))) {
    $('task-state').textContent = '状态未知';
    $('task-summary').textContent = '飞控反馈不可用，不推算任务进度；检查上方链路状态。';
    $('task-percent').textContent = $('task-elapsed').textContent = '—';
    $('task-measured').textContent = $('task-error').textContent = '—';
    $('task-progress').style.width = '0%';
    return;
  }
  const current = motion || {state: 'idle'}, params = current.params || {};
  $('task-state').textContent = labels[current.state] || '未知';
  $('task-summary').textContent = current.message || '先启用实机控制、进入OFFBOARD并解锁；起飞后可平移、转向和升降。';
  const unit = params.kind === 'rotate' ? '°' : 'm';
  value('task-measured', current.measured, 2, unit);
  value('task-error', current.error, 2, unit);
  value('task-elapsed', current.elapsed_s, 1, ' s');
  const ratio = Number.isFinite(current.progress) ? Math.max(0, Math.min(1, current.progress)) : null;
  $('task-progress').style.width = (ratio === null ? 0 : ratio * 100) + '%';
  $('task-percent').textContent = ratio === null ? '—' : Math.round(ratio * 100) + '%';
  const fingerprint = current.id + ':' + current.state + ':' + current.message;
  if (current.id && !['running', 'landing'].includes(current.state) && fingerprint !== observedMotion) {
    addRecord('真实任务：' + current.message, 'hardware_task');
  }
  observedMotion = fingerprint;
}

// ==================== 当前验证入口 ====================
const tests = [
  ['C++ 比赛任务链', '虚拟串口验证', '起飞、轨迹、识别、对准、距离保持与降落；API断开后仍由C++调度。', 'scripts/verify_mission_cpp.py', 'cyan'],
  ['视觉控制', '虚拟串口验证', '视觉PID、前后方向、丢失与恢复；不连接实机串口。', 'scripts/verify_vision_control.py / verify_vision_distance.py', 'cyan'],
  ['共享参数与网页接口', '离线验证', '检查视觉默认值持久化、API兼容及编排编辑。', 'scripts/verify_vision_defaults.py / verify_web_control.py', 'cyan'],
  ['真实飞行性能', '需实飞验证', '虚拟串口不模拟机体动力学，无法代替实飞检查识别稳定性与PID响应。', '当前页面不将离线检查当作实飞通过', 'amber']
];
tests.forEach(([title, state, description, evidence, color]) => {
  const card = document.createElement('article'); card.className = 'card test-card';
  const badge = document.createElement('span'); badge.className = 'tag ' + color; badge.textContent = state;
  const heading = document.createElement('h2'); heading.textContent = title;
  const copy = document.createElement('p'); copy.textContent = description;
  const footer = document.createElement('div'); footer.className = 'evidence'; footer.textContent = evidence;
  card.append(badge, heading, copy, footer); $('test-grid').append(card);
});

$('open-vision-defaults').onclick = () => { location.hash = 'motion'; };
