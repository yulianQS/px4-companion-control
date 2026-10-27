'use strict';
// 这里只同步CSV记录状态；图表仅在用户点击后加载，不绘制实时位置。
(() => {
  const byId = id => document.getElementById(id);
  const frame = byId('trajectory-frame');
  let generation = 0, timer = null;
  function update(snapshot) {
    const record = snapshot?.trajectory || {};
    byId('trajectory-recording').textContent = record.error || (record.recording ?
      `开发板记录中 · ${record.points || 0} 点；结束后生成回放` : record.latest_complete ?
      '上次CSV已保存 · 点击生成轨迹' : '等待解锁记录 · 上锁结束后生成');
    byId('trajectory-recording').className = 'muted' + (record.error ? ' trajectory-error' : '');
    byId('trajectory-download').hidden = !record.latest_complete;
  }
  byId('map3d-generate').onclick = () => {
    clearTimeout(timer);
    const request = String(++generation);
    byId('map3d-note').textContent = '正在从开发板CSV生成Plotly三维轨迹…';
    frame.hidden = false;
    byId('trajectory-placeholder').hidden = true;
    frame.src = '/trajectory-view?request=' + request;
    timer = setTimeout(() => {
      if (request === String(generation)) byId('map3d-note').textContent =
        '图表加载尚未完成，请检查开发板网络后重新点击生成；CSV仍保存在开发板。';
    }, 30000);
  };
  byId('map3d-clear').onclick = () => {
    generation++;
    clearTimeout(timer);
    frame.src = 'about:blank';
    frame.hidden = true;
    byId('trajectory-placeholder').hidden = false;
    byId('map3d-note').textContent = '仅清除网页显示，开发板CSV保留。';
  };
  window.addEventListener('message', event => {
    if (event.origin !== location.origin || event.source !== frame.contentWindow ||
        event.data?.type !== 'trajectory-result' || event.data.request !== String(generation)) return;
    clearTimeout(timer);
    byId('map3d-note').textContent = event.data.message;
  });
  window.flightMap3D = {update};
  update(typeof lastSnapshot === 'undefined' ? {} : lastSnapshot);
})();
