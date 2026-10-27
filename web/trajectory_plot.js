'use strict';
// 独立只读回放页：只读取已完成CSV的点集，不访问串口、不提交控制请求。
(() => {
  const graph = document.getElementById('trajectory-plot');
  const status = document.getElementById('plot-status');
  const request = new URLSearchParams(location.search).get('request') || '';
  const cameras = {
    iso: {eye:{x:1.5,y:1.5,z:1.15}, up:{x:0,y:0,z:1}, projection:{type:'perspective'}},
    top: {eye:{x:0,y:0,z:2.5}, up:{x:0,y:1,z:0}, projection:{type:'orthographic'}},
    side: {eye:{x:0,y:-2.5,z:0}, up:{x:0,y:0,z:1}, projection:{type:'orthographic'}}
  };
  let ready = false, axisRanges = null, dataset = null, activePoints = [], times = [], playing = false, playClock = 0, previousFrame = null, generation = 0;
  const el = id => document.getElementById(id);
  function download(text, name, type) {
    const url = URL.createObjectURL(new Blob([text],{type}));
    const a = document.createElement('a'); a.href = url; a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(url),1000);
  }
  function atTime(time) {
    let lo = 0, hi = times.length;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (times[mid] <= time) lo = mid + 1; else hi = mid; }
    return Math.max(1,lo);
  }
  function actualTrace(points) {
    const coordinates = [[],[],[]], customdata = [], colors = [];
    let previous = null;
    for (let index = 0; index < points.length; index++) {
      const point = points[index];
      if (previous && (point.segment !== previous.segment || point.break_before)) {
        coordinates.forEach(axis => axis.push(null)); customdata.push(null); colors.push('#0099a1');
      }
      coordinates[0].push(point.xyz[0]); coordinates[1].push(point.xyz[1]); coordinates[2].push(activePoints[0].xyz[2]-point.xyz[2]);
      customdata.push([index+1,point.sample_ms,point.xyz[2],point.segment,point.valid?'有效':'未确认']);
      colors.push(point.valid?'#0099a1':'#d99a38'); previous=point;
    }
    return {x:[coordinates[0]],y:[coordinates[1]],z:[coordinates[2]],customdata:[customdata],'marker.color':[colors]};
  }
  function scrub(time) {
    if (!ready) return;
    playClock=Math.max(0,Math.min(times.at(-1),time));
    const count=atTime(playClock), point=activePoints[count-1];
    Plotly.restyle(graph,actualTrace(activePoints.slice(0,count)),[0]);
    Plotly.restyle(graph,{x:[[point.xyz[0]]],y:[[point.xyz[1]]],z:[[activePoints[0].xyz[2]-point.xyz[2]]] },[2]);
    el('replay-time').value=String(playClock);
    el('replay-clock').textContent=`${playClock.toFixed(2)} / ${times.at(-1).toFixed(2)} s · ${count}/${activePoints.length} 点`;
  }
  function frame(stamp) {
    if (!playing) return;
    if (previousFrame !== null) scrub(playClock + (stamp-previousFrame)/1000*Number(el('replay-speed').value));
    previousFrame=stamp;
    if (playClock >= times.at(-1)) { playing=false; return; }
    requestAnimationFrame(frame);
  }
  el('replay-play').onclick=()=>{if(!ready||playing)return;if(playClock>=times.at(-1))scrub(0);playing=true;previousFrame=null;requestAnimationFrame(frame);};
  el('replay-pause').onclick=()=>{playing=false;previousFrame=null;};
  el('replay-time').oninput=()=>{playing=false;scrub(Number(el('replay-time').value));};
  el('analysis-export').onclick=()=>download(JSON.stringify({filename:dataset.filename,...dataset.analysis},null,2),'flight-analysis.json','application/json');
  el('errors-export').onclick=()=>download('elapsed_s,segment,actual_x,actual_y,actual_z,target_x,target_y,target_z,error_3d_m\n'+activePoints.map(p=>[p.elapsed_s,p.segment,...p.xyz,...(p.target||['','','']),p.error??''].join(',')).join('\n'),'flight-errors.csv','text/csv');
  el('csv-import').onchange=async()=>{
    const file=el('csv-import').files[0];if(!file)return;
    try {
      if(file.size>12*1024*1024)throw new Error('CSV超过12MB');
      const response=await fetch('/api/trajectory/analyze',{method:'POST',headers:{'Content-Type':'application/json','X-Control-Request':'1'},body:JSON.stringify({csv:await file.text(),filename:file.name})});
      const result=await response.json();if(!response.ok)throw new Error(result.message||'CSV分析失败');
      await load(result);
    } catch(error){report('导入失败：'+error.message,false);}
  };
  function report(message, ok) {
    status.textContent = message;
    status.className = ok ? '' : 'plot-error';
    if (window.parent !== window) window.parent.postMessage(
      {type:'trajectory-result', request, message, ok}, location.origin);
  }
  function view(name) {
    if (!ready) return;
    const change = {'scene.camera': cameras[name === 'fit' ? 'iso' : name]};
    if (name === 'fit') for (let axis=0; axis<3; axis++) change['scene.' + ['xaxis','yaxis','zaxis'][axis] + '.range'] = axisRanges[axis];
    Plotly.relayout(graph, change).catch(error => report('视角调整失败：' + error.message, false));
  }
  for (const [id, name] of [['view-3d','iso'],['view-top','top'],['view-side','side'],['view-fit','fit']])
    document.getElementById(id).onclick = () => view(name);

  async function load(imported = null) {
    const currentGeneration = ++generation; playing=false; ready=false;
    try {
      if (typeof Plotly === 'undefined') throw new Error('本地Plotly库未加载，请检查网络并重新生成');
      let result = imported;
      if (!result) {
        const response = await fetch('/api/trajectory/latest', {cache:'no-store', signal:AbortSignal.timeout(15000)});
        result = await response.json();
        if (!response.ok) throw new Error(result.message || 'CSV读取失败');
      }
      if(currentGeneration!==generation)return;
      const points = result.points;
      if (!Array.isArray(points) || !points.length || !points.every(point =>
        Array.isArray(point.xyz) && point.xyz.length === 3 && point.xyz.every(Number.isFinite)))
        throw new Error('CSV没有可绘制的有限坐标点');
      dataset=result; activePoints=points;
      const originTime=Number.isFinite(points[0].elapsed_s)?points[0].elapsed_s:points[0].sample_ms/1000;
      times=points.map((p,index)=>Math.max(0,(Number.isFinite(p.elapsed_s)?p.elapsed_s:p.sample_ms/1000)-originTime));
      for(let i=1;i<times.length;i++)times[i]=Math.max(times[i],times[i-1]);
      const first = points[0], last = points[points.length-1];
      const coordinates = [[],[],[]], customdata = [], colors = [];
      const low = [Infinity,Infinity,Infinity], high = [-Infinity,-Infinity,-Infinity];
      let previous = null, segments = 1, invalid = 0;
      for (let index=0; index<points.length; index++) {
        const point = points[index];
        if (previous && (point.segment !== previous.segment || point.break_before)) {
          for (const axis of coordinates) axis.push(null);
          customdata.push(null); colors.push('#0099a1'); segments++;
        }
        // X/Y保留CSV局部NED数值，H向上；hover同时保留原始Z，避免误认为离地高度。
        const xyz = [point.xyz[0],point.xyz[1],first.xyz[2]-point.xyz[2]];
        for (let axis=0; axis<3; axis++) {
          coordinates[axis].push(xyz[axis]);
          low[axis] = Math.min(low[axis], xyz[axis]); high[axis] = Math.max(high[axis], xyz[axis]);
        }
        customdata.push([index+1,point.sample_ms,point.xyz[2],point.segment,point.valid ? '有效' : '未确认']);
        colors.push(point.valid ? '#0099a1' : '#d99a38');
        if (!point.valid) invalid++;
        previous = point;
      }
      const span = Math.max(.1, ...high.map((value, axis) => value-low[axis]));
      axisRanges = low.map((value, axis) => {
        const padding = Math.max(.02, span*.06);
        return [value-padding,high[axis]+padding];
      });
      const hover = '点 %{customdata[0]} · 段 %{customdata[3]}<br>X: %{x:.3f} m<br>Y: %{y:.3f} m<br>相对H: %{z:.3f} m<br>原始NED Z: %{customdata[2]:.3f} m<br>飞控启动时间: %{customdata[1]} ms<br>估计有效性: %{customdata[4]}<extra></extra>';
      const data = [{type:'scatter3d',mode:'lines+markers',name:'CSV轨迹',
        x:coordinates[0],y:coordinates[1],z:coordinates[2],customdata,
        connectgaps:false,line:{color:'#0099a1',width:4},marker:{size:2.5,color:colors},hovertemplate:hover}];
      for (const [point, name, color, symbol] of [[first,'起点','#16865b','circle'],[last,'终点','#ce685c','diamond']])
        data.push({type:'scatter3d',mode:'markers',name,x:[point.xyz[0]],y:[point.xyz[1]],z:[first.xyz[2]-point.xyz[2]],
          marker:{size:6,color,symbol},hovertemplate:name+'<br>X: %{x:.3f} m<br>Y: %{y:.3f} m<br>H: %{z:.3f} m<extra></extra>'});
      const targets=[[],[],[]];let previousTarget=null;
      for(const point of points){
        if(!point.target || (previousTarget && (point.segment!==previousTarget.segment || point.break_before))){targets.forEach(axis=>axis.push(null));}
        if(point.target){targets[0].push(point.target[0]);targets[1].push(point.target[1]);targets[2].push(first.xyz[2]-point.target[2]);}
        previousTarget=point;
      }
      data.push({type:'scatter3d',mode:'lines',name:'C++目标轨迹',x:targets[0],y:targets[1],z:targets[2],connectgaps:false,line:{color:'#efa638',width:4,dash:'dash'},hovertemplate:'目标 X %{x:.3f} m<br>Y %{y:.3f} m<br>H %{z:.3f} m<extra></extra>'});
      const axes = ['X / 局部NED (m)' ,'Y / 局部NED (m)','H / 相对记录起点向上 (m)'].map((text, axis) =>
        ({title:{text},range:axisRanges[axis],showbackground:true,backgroundcolor:'#f4f8fa',gridcolor:'#d9e5eb',zerolinecolor:'#a4bcc8',tickformat:'.2f'}));
      await Plotly.newPlot(graph, data, {margin:{l:0,r:0,t:12,b:0},paper_bgcolor:'#f7fafb',
        font:{family:'Segoe UI, Microsoft YaHei, sans-serif',color:'#152e37'},
        legend:{orientation:'h',x:0,y:1},uirevision:result.filename,
        scene:{xaxis:axes[0],yaxis:axes[1],zaxis:axes[2],aspectmode:'data',camera:cameras.iso,dragmode:'orbit'}},
        {responsive:true,scrollZoom:true,displaylogo:false,showLink:false,
          modeBarButtonsToRemove:['sendDataToCloud','sendChartToCloud'],toImageButtonOptions:{format:'png',filename:'flight-trajectory',scale:2}});
      if(currentGeneration!==generation)return;
      ready = true;
      for(const id of ['replay-play','replay-pause','replay-time','analysis-export','errors-export'])el(id).disabled=false;
      el('replay-time').max=String(times.at(-1));
      const metrics=result.analysis||{};
      el('analysis-summary').textContent=Number.isFinite(metrics.rmse_3d_m)?`Python分析 · 三维RMSE ${metrics.rmse_3d_m.toFixed(4)} m · XYZ ${metrics.rmse_xyz_m.map(n=>n.toFixed(4)).join(' / ')} m · 最大误差 ${metrics.max_error_m.toFixed(4)} m · 比较 ${metrics.compared_points} 点`:'此CSV没有可比较的有效目标/实际坐标，RMSE不可用。历史轨迹仍可回放。';
      scrub(times.at(-1));
      report(`${result.filename} · CSV ${result.total_points} 点，绘制 ${points.length} 点 / ${segments} 段。` +
        (invalid ? ` ${invalid} 点有效性未确认（黄色）。` : '') +
        (result.started_mid_session ? ' 记录从解锁会话中途开始。' : '') +
        (result.dropped_events ? ` 写入队列丢失 ${result.dropped_events} 个事件。` : '') +
        ' 重置/断档处分段；H不是离地高度，坐标为飞控估计值。', true);
    } catch (error) {
      report('无法生成轨迹：' + error.message, false);
    }
  }
  load();
})();
