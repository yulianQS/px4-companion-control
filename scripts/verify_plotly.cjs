// 只读回放数据转换与控制台集成检查；Plotly替身不代替实际WebGL浏览器验证。
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const root=path.resolve(__dirname,'..');
const read=name=>fs.readFileSync(path.join(root,'web',name),'utf8');
const settle=()=>new Promise(resolve=>setImmediate(resolve));
async function plot(result, options={}) {
  const ids=['trajectory-plot','plot-status','view-3d','view-top','view-side','view-fit','replay-play','replay-pause','replay-speed','replay-time','replay-clock','csv-import','analysis-export','errors-export','analysis-summary'];
  const elements=Object.fromEntries(ids.map(id=>[id,{textContent:'',className:''}]));
  const calls=[],messages=[],views=[],restyles=[];let fetches=0;
  const context={document:{getElementById:id=>elements[id]},URLSearchParams,AbortSignal,requestAnimationFrame:()=>1,
    location:{origin:'http://board',search:'?request=3'},window:{parent:{postMessage:(data,origin)=>messages.push({data,origin})}},
    fetch:async(url,config)=>{fetches++;assert.equal(url,'/api/trajectory/latest');assert(!config.method);return {ok:options.ok!==false,json:async()=>result};},
    Plotly:{newPlot:async(...args)=>{if(options.webglError)throw Error('WebGL unavailable');calls.push(args);},relayout:async(...args)=>views.push(args),restyle:async(...args)=>restyles.push(args)}};
  if(options.missingLibrary)delete context.Plotly;
  vm.runInNewContext(read('trajectory_plot.js'),context);await settle();
  return {elements,calls,messages,views,fetches,restyles};
}
(async()=>{
  const points=[{xyz:[1,2,-3],sample_ms:100,segment:0,valid:true},{xyz:[1,2,-3.5],sample_ms:120,segment:0,valid:true},
    {xyz:[0,0,-3.4],sample_ms:140,segment:1,valid:false},{xyz:[.1,.2,-3.2],sample_ms:160,segment:1,break_before:true,valid:true}];
  const result={filename:'flight_test.csv',total_points:5,points,analysis:{compared_points:2,rmse_3d_m:.01,rmse_xyz_m:[.01,0,0],max_error_m:.02}}; points[0].target=[1,2,-3.1];
  const p=await plot(result);assert.equal(p.fetches,1);assert.equal(p.calls.length,1);
  const [,data,layout,config]=p.calls[0],track=data[0];
  assert.equal(data.length,4); assert.equal(data[3].name,'C++目标轨迹');assert.equal(p.elements['replay-play'].disabled,false);assert(p.elements['analysis-summary'].textContent.includes('RMSE')); p.elements['replay-time'].value='0';p.elements['replay-time'].oninput();assert(p.restyles.at(-2)[1].x[0].length===1);
  assert.equal(track.type,'scatter3d');assert.equal(track.connectgaps,false);
  assert.deepEqual(Array.from(track.x),[1,1,null,0,null,.1]);
  assert.equal(track.z[1],.5);assert.equal(track.customdata[1][2],-3.5);
  assert.equal(track.marker.color[3],'#d99a38');assert.equal(layout.scene.aspectmode,'data');assert(config.scrollZoom);
  assert(track.hovertemplate.includes('customdata[2]'));assert(p.messages[0].data.message.includes('3 段'));
  for(const id of ['view-top','view-side','view-fit']){p.elements[id].onclick();await settle();}
  assert.equal(p.views.length,3);assert(p.views[2][1]['scene.xaxis.range']);
  const many=await plot({filename:'many.csv',total_points:12000,points:Array.from({length:12000},(_,i)=>({xyz:[i*.001,0,-1],segment:0,sample_ms:i,valid:true}))});
  assert.equal(many.calls[0][1][0].x.length,12000);
  for(const [answer,opts] of [[{message:'暂无完整飞行记录'},{ok:false}],[{points:[]},{}],[result,{missingLibrary:true}],[result,{webglError:true}]]){
    const failed=await plot(answer,opts);assert.equal(failed.calls.length,0);assert(failed.elements['plot-status'].textContent.startsWith('无法生成轨迹'));assert.equal(failed.messages[0].data.ok,false);
  }
  const elements={};for(const id of ['trajectory-frame','trajectory-recording','trajectory-download','map3d-generate','map3d-clear','trajectory-placeholder','map3d-note'])elements[id]={contentWindow:{},src:'about:blank'};
  let message;const context={document:{getElementById:id=>elements[id]},window:{addEventListener:(name,fn)=>{assert.equal(name,'message');message=fn;}},location:{origin:'http://board'},lastSnapshot:null,setTimeout:()=>1,clearTimeout:()=>{}};
  vm.runInNewContext(read('map3d.js'),context);
  for(let i=0;i<100;i++)context.window.flightMap3D.update({trajectory:{latest_complete:'flight.csv'},px4:{local_position:{x:i,y:0,z:0}}});
  assert.equal(elements['trajectory-frame'].src,'about:blank');
  elements['map3d-generate'].onclick();assert.equal(elements['trajectory-frame'].src,'/trajectory-view?request=1');
  const event={origin:'http://board',source:elements['trajectory-frame'].contentWindow,data:{type:'trajectory-result',request:'1',message:'60 points'}};
  message({...event,origin:'http://other'});assert(!elements['map3d-note'].textContent.includes('60 points'));
  message(event);assert.equal(elements['map3d-note'].textContent,'60 points');
  elements['map3d-clear'].onclick();message(event);assert.equal(elements['trajectory-frame'].src,'about:blank');assert(elements['trajectory-frame'].hidden);assert(!elements['map3d-note'].textContent.includes('60 points'));
  if(process.argv[2]){const actual=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));const r=await plot(actual);assert.equal(r.calls[0][1][0].x.filter(v=>v!==null).length,actual.points.length);console.log('Actual CSV point set:',actual.filename,actual.points.length);}
  console.log('PASS: Plotly逐点/分段/NED高度转换/hover/视角/12000点/错误提示；不实时绘图、无控制请求、清除与迟到通知');
})().catch(error=>{console.error(error);process.exitCode=1;});
