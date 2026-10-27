(() => {
  'use strict';
  const canvas = document.getElementById('camera-image');
  const context = canvas.getContext('2d', {alpha: false});
  const placeholder = document.getElementById('camera-placeholder');
  const badge = document.getElementById('camera-badge');
  const missionCanvas = document.getElementById('mission-camera-image');
  const missionContext = missionCanvas.getContext('2d', {alpha: false});
  const missionPlaceholder = document.getElementById('mission-camera-placeholder');
  const missionBadge = document.getElementById('mission-camera-badge');
  const translations = {
    camera_missing_or_stale: '摄像头未连接或图像过期',
    target_stream_stale: '二维目标消息已过期',
    invalid_target_measurement: '目标测量无效，建议归零',
    target_not_found_or_ambiguous: '未发现唯一目标，建议归零',
    confirm_camera_upright_before_preview: '画面方向尚未确认，偏航建议保持零',
    preview_disabled: '视觉偏航建议已停用',
    yaw_preview_only: '二维偏航跟随建议；未发送飞控'
  };
  // Parse actual MJPEG frames: img.onload/error cannot reliably report multipart EOF or a frozen image.
  class MjpegDecoder {
    constructor() { this.buffer = new Uint8Array(0); this.length = null; }
    push(chunk) {
      if (this.buffer.length + chunk.length > 2 * 1024 * 1024 + 8192) throw new Error('图像流超出大小限制');
      const joined = new Uint8Array(this.buffer.length + chunk.length);
      joined.set(this.buffer); joined.set(chunk, this.buffer.length); this.buffer = joined;
      let latest = null;
      while (true) {
        if (this.length === null) {
          let end = -1;
          for (let i = 0; i + 3 < this.buffer.length; i++) {
            if (this.buffer[i] === 13 && this.buffer[i+1] === 10 && this.buffer[i+2] === 13 && this.buffer[i+3] === 10) { end = i; break; }
          }
          if (end < 0) {
            if (this.buffer.length > 4096) throw new Error('图像流头无效');
            return latest;
          }
          if (end > 4096) throw new Error('图像流头过长');
          const header = new TextDecoder().decode(this.buffer.subarray(0, end));
          const match = /(?:^|\r\n)Content-Length:\s*(\d+)\s*(?:\r\n|$)/i.exec(header);
          if (!match || !/Content-Type:\s*image\/jpeg/i.test(header)) throw new Error('图像流格式无效');
          this.length = Number(match[1]);
          if (this.length < 4 || this.length > 2 * 1024 * 1024) throw new Error('JPEG 大小无效');
          this.buffer = this.buffer.subarray(end + 4);
        }
        if (this.buffer.length < this.length) return latest;
        const jpeg = this.buffer.slice(0, this.length);
        this.buffer = this.buffer.subarray(this.length); this.length = null;
        if (jpeg[0] !== 255 || jpeg[1] !== 216 || jpeg[jpeg.length-2] !== 255 || jpeg[jpeg.length-1] !== 217) throw new Error('JPEG 不完整');
        latest = jpeg; // Drop older complete frames in the same read; never build a decode queue.
      }
    }
  }
  let controller = null, lastFrame = 0, connectingSince = 0, retryMs = 500;
  let fpsStart = 0, frames = 0, receiveFps = 0, reconnects = 0;
  const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
  function unavailable(text) {
    canvas.hidden = true; placeholder.hidden = false;
    badge.textContent = text;
    missionCanvas.hidden = true; missionPlaceholder.hidden = false;
    missionPlaceholder.textContent = text; missionBadge.textContent = text;
  }
  function showFrame(bitmap) {
    if (bitmap.width < 2 || bitmap.height < 2 || bitmap.width > 4096 || bitmap.height > 4096) {
      bitmap.close(); throw new Error('图像尺寸无效');
    }
    if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
      canvas.width = bitmap.width; canvas.height = bitmap.height;
    }
    if (missionCanvas.width !== bitmap.width || missionCanvas.height !== bitmap.height) {
      missionCanvas.width = bitmap.width; missionCanvas.height = bitmap.height;
    }
    context.drawImage(bitmap, 0, 0);
    missionContext.drawImage(bitmap, 0, 0); bitmap.close();
    const tick = performance.now(); lastFrame = tick; retryMs = 500;
    if (!fpsStart) { fpsStart = tick; frames = 0; } else frames++;
    if (tick - fpsStart >= 2000) { receiveFps = frames * 1000 / (tick - fpsStart); fpsStart = tick; frames = 0; }
    canvas.hidden = false; placeholder.hidden = true;
    badge.textContent = (receiveFps ? receiveFps.toFixed(1) + ' FPS' : '画面已连接') + (reconnects ? ' · 重连 ' + reconnects : '');
    missionCanvas.hidden = false; missionPlaceholder.hidden = true;
    missionBadge.textContent = badge.textContent;
  }
  async function streamLoop() {
    while (true) {
      if (document.hidden) { await delay(500); continue; }
      const own = new AbortController(); controller = own;
      connectingSince = performance.now(); lastFrame = 0; fpsStart = 0; frames = 0; receiveFps = 0;
      let reader;
      try {
        const response = await fetch('/api/vision/stream.mjpg', {signal: own.signal, cache: 'no-store'});
        if (!response.ok || !response.body || !(response.headers.get('Content-Type') || '').startsWith('multipart/')) throw new Error('图像服务不可用');
        reader = response.body.getReader(); const decoder = new MjpegDecoder();
        while (!own.signal.aborted) {
          const part = await reader.read();
          if (part.done) throw new Error('图像流已断开');
          const jpeg = decoder.push(part.value);
          if (jpeg) {
            const bitmap = await createImageBitmap(new Blob([jpeg], {type: 'image/jpeg'}));
            if (own.signal.aborted || document.hidden) { bitmap.close(); break; }
            showFrame(bitmap);
          }
        }
      } catch (error) {
        if (!document.hidden) unavailable('画面中断，自动重连中');
      } finally {
        own.abort();
        if (reader) { try { await reader.cancel(); } catch (_) {} }
        if (controller === own) controller = null;
      }
      if (!document.hidden) { reconnects++; await delay(retryMs); retryMs = Math.min(8000, retryMs * 2); }
    }
  }
  // Frame watchdog, independent of HTTP flight status and ROS target validity.
  setInterval(() => {
    if (!document.hidden && controller && performance.now() - (lastFrame || connectingSince) > 2000) {
      unavailable('超过 2 秒无图像，正在重连'); controller.abort();
    }
  }, 250);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && controller) { controller.abort(); unavailable('页面后台，画面已暂停'); }
  });
  window.rosVisionPanel = {render(vision) {
    window.motionWorkspace?.renderVision(vision);
    document.getElementById('vision-message').textContent =
      (translations[vision.message] || vision.message || '等待ROS2视觉节点') +
      (vision.camera_message ? ' · ' + vision.camera_message : '');
    document.getElementById('mission-camera-message').textContent =
      document.getElementById('vision-message').textContent;
    const error = Array.isArray(vision.normalized_error) ?
      vision.normalized_error.map(n => Number(n).toFixed(3)).join(', ') : '—';
    const yaw = Number.isFinite(vision.yaw_preview_rad_s) ? vision.yaw_preview_rad_s : 0;
    document.getElementById('vision-preview').textContent =
      '归一化误差 ' + error + '；偏航建议 ' + yaw.toFixed(3) +
      ' rad/s（建议）；' + (vision.calibrated ?
        '相机内参已加载' + (vision.pose_valid && Number.isFinite(vision.distance_m) ?
          ' · 相机前向距离 ' + vision.distance_m.toFixed(3) + ' m' : ' · 等待米制位姿') : '等待相机内参');
  }};
  // Vision status does not wait for a flight-service request or share its failure path.
  async function statusLoop() {
    const own = new AbortController(); const timeout = setTimeout(() => own.abort(), 1500);
    try {
      const response = await fetch('/api/vision/status', {signal: own.signal, cache: 'no-store'});
      if (!response.ok) throw new Error('视觉状态不可用');
      window.rosVisionPanel.render(await response.json());
    } catch (_) { window.rosVisionPanel.render({message: '视觉状态连接中断'}); }
    finally { clearTimeout(timeout); setTimeout(statusLoop, document.hidden ? 2000 : 500); }
  }
  streamLoop(); statusLoop();
})();
