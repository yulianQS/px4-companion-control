/* Incremental UI for the existing #page-mission. The backend schedules every block. */
(() => {
  'use strict';
  const M=window.MissionModel, el=id=>document.getElementById(id), storage='drone-competition-v1';
  const labels={pending:'待执行',running:'执行中',waiting:'等待中',succeeded:'已完成',redirected:'已跳转恢复',failed:'失败',cancelled:'已取消',skipped:'未执行',stopping:'停止中',stopped:'已停止',idle:'待机'};
  let document_=M.empty(), catalogue=[], categories=[], category='全部', mode='list', run=null, ready=false, statusAvailable=false, uiBusy=false, selected=null, dragged=null, lastRunFingerprint='', observedRunId=null, runGraphMatches=false;
  const controls=new Map();
  const sharedFields={yaw_gain:'偏航 P',yaw_ki:'偏航 I',yaw_kd:'偏航 D',yaw_speed_rad_s:'偏航速度幅值 / rad/s（0 为不限幅）',
    vertical_gain:'升降 P',vertical_ki:'升降 I',vertical_kd:'升降 D',vertical_speed_m_s:'升降速度幅值 / m/s（0 为不限幅）',
    distance_gain:'距离 P',distance_ki:'距离 I',distance_kd:'距离 D',forward_speed_m_s:'前进速度幅值 / m/s（0 为不限幅）',
    lateral_gain:'横滚/横移 P',lateral_ki:'横滚/横移 I',lateral_kd:'横滚/横移 D',lateral_speed_m_s:'横移速度幅值 / m/s（0 为不限幅）',lateral_enabled:'启用横移控制',
    tag_size_m:'标签实测边长 / m',target_distance_m:'期望保持距离 / m（相机前向）'};
  const fieldHelp={
    _timeout:'整个积木最多运行多久；超时与达标完成是两回事。',
    speed:'参考运动速度；不设置要飞多远。实际速度受轨迹加速度及 PX4 控制影响。',
    acceleration:'参考速度变化的快慢；0 表示关闭参考速度斜坡。',
    angular_acceleration:'参考角速度变化的快慢；0 表示关闭参考角速度斜坡。',
    tolerance:'实际位置或角度与目标的允许误差，用于判断是否到位。',
    level_angle_deg:'相对位移前，横滚和俯仰的绝对值都必须小于此角度。',
    level_speed_m_s:'回正时先制动；实际三轴合速度低于此值才允许开始位移。',
    level_settle_s:'角度和速度同时达标并连续稳定这么久，才开始计算位移距离。',
    level_timeout_s:'回正等待超过此时间则失败并保持位置；等待也计入整个积木超时。',
    height:'目标局部高度，对应 NED 的 Z = −高度；不是实时测得的离地高度。',
    x:'局部 NED 的 X 坐标，单位 cm；是坐标目标，不是相对位移。',
    y:'局部 NED 的 Y 坐标，单位 cm。',z:'局部 NED 的 Z 向下为正，负值在局部原点上方；单位 cm。',
    waypoints:'按顺序填写 [[X,Y,Z],…]，使用局部 NED 坐标，单位 cm。',
    fixed_yaw_deg:'航点飞行时的固定航向；留空则沿用任务原航向。',
    repetitions:'完整航点序列重复执行的次数。',
    scan_angle:'广域搜索向左右扫描的角度范围。',scan_speed:'广域扫描时的旋转角速度。',
    forward_distance:'一轮广域扫描后向前移动的距离，不是最终保持距离。',
    approach_speed:'首次发现目标后缓慢接近的前向速度。',
    approach_acceleration:'接近速度变化的快慢；0 表示不作速度斜坡。',
    settle_seconds:'误差同时满足阈值并保持多久才算达标；识别类任务则累计真实识别时间。',
    horizontal_threshold:'目标中心距画面中心的水平允许误差；越小要求越精确。',
    vertical_threshold:'启用升降控制时的垂直达标误差；粗对准交接不等待此条件。',
    coarse_horizontal_threshold:'后接距离保持/跟随时，水平误差进入此范围即可交接；垂直对准在靠近时同步完成。',
    coarse_settle_seconds:'粗水平对准需持续的时间；后接距离保持时不等待垂直居中。',
    max_distance:'超过此距离的目标按暂不可用处理；不是期望保持距离，也不是前进总路程。',
    distance_tolerance:'距离达标的允许误差。例如期望 0.5 m、容差 0.1 m，0.4～0.6 m 算距离达标。',
    lost_timeout:'目标不可用后多久执行丢失处理；距离保持/跟随阶段会在此期间沿上次运动方向续行。',
    lost_target:'丢失达到设定时间后转去的积木；留空则报告失败。',
    target_distance_m:'决定前后距离 PID 要到达的目标值；不是 PID 增益或最大对准距离。与其他视觉任务共用。',
    tag_size_m:'填写 AprilTag 黑色外框的实测边长，不含白色留边；填错会使米制测距按比例偏差。',
    distance_gain:'前后距离响应的比例增益；改变靠近/后退的力度，不改变期望距离。',
    yaw_gain:'按水平画面误差调节偏航响应。',vertical_gain:'按垂直画面误差调节升降响应。',
    lateral_gain:'启用横移后，按水平画面误差调节左右运动。',
    forward_speed_m_s:'前后 PID 输出的速度幅值；0 不作工程限幅，不改变期望距离。',
    _next:'达标完成后进入的积木；默认按列表下一项执行。'
  };
  const visualTypes=new Set(['align','center','follow','distance']);
  let sharedConfig=null,paramSaving=false;
  async function sharedRequest(body){
    const response=await fetch('/api/vision/control-config',{method:body===undefined?'GET':'POST',cache:'no-store',
      ...(body===undefined?{}:{headers:{'Content-Type':'application/json','X-Control-Request':'1','X-Client-ID':clientId},body:JSON.stringify(body)}),signal:AbortSignal.timeout(5000)});
    const value=await response.json();if(!response.ok||!value.ok)throw new Error(value.message||'视觉默认参数读取/保存失败');return value.parameters;
  }
  async function syncShared(){
    const value=await sharedRequest();if(active())return;
    sharedConfig=value;
    document_.blocks.filter(b=>visualTypes.has(b.type)).forEach(b=>Object.assign(b.params,{adjust_speed:value.forward_speed_m_s,yaw_speed:value.yaw_speed_rad_s,target_distance:value.target_distance_m}));
    changed();
  }
  window.addEventListener('vision-defaults-changed',()=>{syncShared().catch(error=>result(error.message));});
  try{const saved=JSON.parse(localStorage.getItem(storage));if(saved?.version===1&&Array.isArray(saved.blocks))document_=saved;}catch(_){}
  const active=()=>!!run?.active;
  const definition=b=>catalogue.find(d=>d.type===b.type);
  const node=(tag,text,className)=>{const item=document.createElement(tag);if(text!==undefined)item.textContent=text;if(className)item.className=className;return item;};
  function remember(){try{localStorage.setItem(storage,JSON.stringify(document_));}catch(_){el('mission-result').textContent='浏览器草稿保存失败；请点击保存或导出 JSON。';}}
  function result(text){el('mission-result').textContent=text;}
  async function request(path,body){
    const response=await fetch('/api/mission/'+path,{method:body===undefined?'GET':'POST',cache:'no-store',
      ...(body===undefined?{}:{headers:{'Content-Type':'application/json','X-Control-Request':'1','X-Client-ID':clientId},body:JSON.stringify(body)}),
      signal:AbortSignal.timeout(8000)});
    const value=await response.json();if(!response.ok||value.ok===false)throw new Error(value.message||'比赛接口请求失败');return value;
  }
  async function operation(callback){
    if(uiBusy)return;uiBusy=true;buttons();
    try{await callback();}catch(error){result(error.message);toast(error.message);}finally{uiBusy=false;buttons();}
  }
  function changed(){remember();render();}
  function option(select,value,label){const item=node('option',label);item.value=value??'';select.append(item);}
  function references(select,value,empty='结束 / 无目标',exclude=null){
    select.replaceChildren();option(select,'',empty);
    document_.blocks.filter(b=>b.id!==exclude).forEach((b,i)=>option(select,b.id,`${i+1}. ${b.name} · ${b.id.slice(-8)}`));
    select.value=value||'';
  }
  function capability(b){const d=definition(b);return d?.executable&&!('target_id' in b.params&&b.params.target_id!==0)?'可实机执行':'仅编辑 · 待接入';}
  function renderLibrary(){
    const list=el('mission-blocks');list.replaceChildren();
    const query=el('mission-search').value.trim().toLowerCase();
    catalogue.filter(d=>(category==='全部'||d.category===category)&&[d.name,d.category,d.description].join(' ').toLowerCase().includes(query)).forEach(d=>{
      const button=node('button',undefined,'mission-block');button.type='button';button.disabled=active();button.draggable=!active();button.dataset.type=d.type;
      const body=node('div',d.name);body.append(node('small',d.description),node('span',d.executable?'可实机执行':'仅编辑 · 待接入','capability'+(d.executable?'':' draft')));
      button.append(node('span','◇'),body,node('i','+'));button.setAttribute('aria-label','添加'+d.name);
      button.onclick=()=>add(d.type);button.addEventListener('dragstart',event=>{event.dataTransfer.setData('application/x-mission-type',d.type);event.dataTransfer.effectAllowed='copy';dragged=null;});
      list.append(button);
    });
    if(!list.children.length)list.append(node('p',ready?'没有匹配的积木。':'正在读取积木定义…','muted'));
  }
  function renderCategories(){
    el('mission-categories').replaceChildren();
    ['全部',...categories].forEach(name=>{const b=node('button',name,category===name?'selected':'');b.type='button';b.setAttribute('aria-pressed',String(category===name));b.onclick=()=>{category=name;renderCategories();renderLibrary();};el('mission-categories').append(b);});
  }
  function add(type){if(active()||document_.blocks.length>=200)return;document_.blocks.push(M.make(catalogue.find(d=>d.type===type)));M.wire(document_);changed();}
  function summary(b){
    const d=definition(b);if(!d)return '未知积木类型，请重新导入有效任务';
    return d.fields.slice(0,4).map(f=>{const v=b.params[f.key];return f.label.split('（')[0]+': '+(v===null?'自动':typeof v==='object'?JSON.stringify(v):v===''?'未配置':v);}).join(' · ')||d.description;
  }
  function stateFor(b){return runGraphMatches?run.blocks?.[b.id]:null;}
  function renderList(){
    const graph=d=>JSON.stringify([d.entry,d.blocks.map(b=>[b.id,b.type,b.params,b.next,b.branches,b.timeout])]);
    runGraphMatches=!!run?.document&&run.mission_id===document_.id&&graph(document_)===graph(run.document);
    const list=el('mission-list');list.replaceChildren();
    document_.blocks.forEach((b,index)=>{
      const state=stateFor(b), status=state?.status||'pending';
      const row=node('li',undefined,status);row.dataset.id=b.id;row.draggable=!active();
      const main=node('div',undefined,'mission-row-main');main.tabIndex=0;main.setAttribute('role','button');main.setAttribute('aria-label','配置'+b.name);
      main.append(node('b',b.name),node('small',definition(b)?.description||'','mission-purpose'),node('p',summary(b)),node('span',b.id,'mission-row-id'),node('span',labels[status]||status,'mission-state '+status));
      if(state?.visits)main.append(node('small',` · 执行 ${state.visits} 次`));
      const cap=node('span',capability(b),'capability'+(definition(b)?.executable?'':' draft'));main.append(document.createTextNode(' '),cap);
      main.onclick=()=>configure(b.id);main.onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();configure(b.id);}};
      const actions=node('div',undefined,'mission-row-actions');
      const action=(label,title,callback,disabled=false)=>{const button=node('button',label);button.type='button';button.title=title;button.setAttribute('aria-label',title);button.disabled=active()||disabled;button.onclick=callback;actions.append(button);};
      action('配置','配置 '+b.name,()=>configure(b.id));
      action('复制','复制 '+b.name,()=>{const copy=M.clone(b);copy.id=M.id();copy.name+=' · 副本';copy.status='pending';copy.auto_next=true;document_.blocks.splice(index+1,0,copy);M.wire(document_);changed();},document_.blocks.length>=200);
      action('删除','删除 '+b.name,()=>{M.remove(document_,b.id);changed();result('已删除；被引用的区间或跳转需要重新配置。');});
      action('↑','上移 '+b.name,()=>{M.reorder(document_,b.id,document_.blocks[index-1]?.id);changed();},index===0);
      action('↓','下移 '+b.name,()=>{M.reorder(document_,document_.blocks[index+1]?.id,b.id);changed();},index===document_.blocks.length-1);
      row.append(main,actions);row.addEventListener('dragstart',event=>{dragged=b.id;event.dataTransfer.setData('application/x-mission-id',b.id);event.dataTransfer.effectAllowed='move';});
      row.addEventListener('dragover',event=>{if(!active())event.preventDefault();});
      row.addEventListener('drop',event=>{event.preventDefault();event.stopPropagation();drop(event,b.id);});list.append(row);
    });
  }
  function drop(event,before=null){
    if(active())return;
    const type=event.dataTransfer.getData('application/x-mission-type');
    if(type&&catalogue.some(d=>d.type===type)){
      add(type);const added=document_.blocks.at(-1);if(before){M.reorder(document_,added.id,before);changed();}
    }else{const id_=event.dataTransfer.getData('application/x-mission-id')||dragged;M.reorder(document_,id_,before);changed();}
    dragged=null;
  }
  el('mission-list').addEventListener('dragover',event=>{if(!active())event.preventDefault();});
  el('mission-list').addEventListener('drop',event=>{event.preventDefault();drop(event);});
  function renderFlow(){
    const host=el('mission-flow');host.replaceChildren();
    if(!document_.blocks.length){host.append(node('p','还没有任务。点击左侧积木添加。','empty'));return;}
    const NS='http://www.w3.org/2000/svg', svg=document.createElementNS(NS,'svg');
    const height=Math.ceil(document_.blocks.length/3)*160+50;
    svg.setAttribute('viewBox',`0 0 870 ${height}`);svg.setAttribute('role','img');svg.setAttribute('aria-label','任务连接关系，点击节点配置');
    const shape=(tag,attrs,text)=>{const item=document.createElementNS(NS,tag);Object.entries(attrs).forEach(([k,v])=>item.setAttribute(k,String(v)));if(text!==undefined)item.textContent=text;return item;};
    const defs=shape('defs',{}), marker=shape('marker',{id:'mission-arrow',viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:6,markerHeight:6,orient:'auto-start-reverse'});marker.append(shape('path',{d:'M 0 0 L 10 5 L 0 10 z',fill:'#76a4aa'}));defs.append(marker);svg.append(defs);
    const positions=new Map(document_.blocks.map((b,i)=>[b.id,{x:25+(i%3)*285,y:30+Math.floor(i/3)*160}]));
    const edge=(from,to,label,color)=>{
      if(!to||!positions.has(to)||!positions.has(from))return;const a=positions.get(from),b=positions.get(to),x=a.x+120,y=a.y+84,tx=b.x+120,ty=b.y;
      const route=ty>y?`M ${x} ${y} C ${x} ${y+40}, ${tx} ${ty-40}, ${tx} ${ty}`:`M ${a.x+240} ${a.y+42} C ${a.x+280} ${a.y+42}, ${b.x+260} ${b.y-12}, ${tx} ${ty}`;
      svg.append(shape('path',{d:route,fill:'none',stroke:color,'stroke-width':1.5,'marker-end':'url(#mission-arrow)','stroke-dasharray':label==='区间'?'4 3':''}));
      svg.append(shape('text',{x:x+5,y:y+17,fill:color,'font-size':10},label));
    };
    for(const b of document_.blocks){
      edge(b.id,b.next,'下一步','#76a4aa');
      if(b.params.lost_target)edge(b.id,b.params.lost_target,'丢失恢复','#ce9850');
    }
    document_.blocks.forEach((b,i)=>{
      const p=positions.get(b.id),status=stateFor(b)?.status||'pending';
      const group=shape('g',{class:'flow-node',tabindex:0,role:'button','aria-label':b.name});
      group.append(shape('rect',{x:p.x,y:p.y,width:240,height:84,rx:7,fill:status==='running'?'#e3f6f4':'white',stroke:status==='failed'?'#c87764':status==='succeeded'?'#91c5a3':'#a7c7cb'}));
      group.append(shape('text',{x:p.x+12,y:p.y+24,fill:'#173d42','font-size':13},`${i+1}. ${b.name.slice(0,16)}`));
      group.append(shape('text',{x:p.x+12,y:p.y+46,fill:'#76909a','font-size':10},`${labels[status]} · ${capability(b)}`));
      group.append(shape('text',{x:p.x+12,y:p.y+66,fill:'#76909a','font-size':9},b.id.slice(-20)));
      group.onclick=()=>configure(b.id);group.onkeydown=e=>{if(e.key==='Enter')configure(b.id);};svg.append(group);
    });host.append(svg);
  }
  function render(){
    el('mission-name').value=document_.name;references(el('mission-entry'),document_.entry,'请选择入口');
    renderList();if(mode==='flow')renderFlow();renderLibrary();buttons();
  }
  function buttons(){
    document.querySelectorAll('#page-mission [data-mission-edit]').forEach(item=>item.disabled=active()||uiBusy||!ready);
    el('mission-start').disabled=active()||uiBusy||!ready||!statusAvailable||!document_.blocks.length;
    el('mission-stop').disabled=!active()||uiBusy;el('mission-land').disabled=!active()||uiBusy;
    el('mission-adopt-run').hidden=!run?.document||run.mission_id===document_.id;
    el('mission-adopt-run').textContent=active()?'查看正在运行的编排':'查看上次运行的编排';
    el('mission-start').textContent=uiBusy?'正在处理…':'启动比赛任务';
  }
  async function configure(id_){
    if(active()){result('比赛运行中，参数只读；停止后可以重新编辑。');return;}
    const b=document_.blocks.find(b=>b.id===id_),d=definition(b);if(!b||!d)return;
    if(visualTypes.has(b.type)){
      try{sharedConfig=await sharedRequest();}catch(error){result(error.message);return;}
      if(active())return;
    }
    selected=id_;controls.clear();el('mission-params-title').textContent=b.name+' · 参数';
    el('mission-params-capability').textContent=d.description+'\n'+(d.executable?'实际执行复用现有控制链路。':d.reason)+'\n实例 ID：'+b.id;
    const host=el('mission-params-fields');host.replaceChildren();
    const addField=(key,label,kind,value,meta={})=>{
      const wrap=node('label',label,kind==='json'?'wide':'');let input;
      if(kind==='select'||kind==='ref'){
        input=node('select');if(kind==='ref')references(input,value,meta.empty||'无 / 未配置',meta.exclude);
        else{meta.options.forEach(v=>option(input,v,v));input.value=value;}
      }else if(kind==='boolean'){input=node('input');input.type='checkbox';input.checked=!!value;}
      else if(kind==='json'){input=node('textarea');input.value=JSON.stringify(value,null,2);}
      else{input=node('input');input.type=['number','integer','optional_number'].includes(kind)?'number':'text';input.value=value??'';input.step=kind==='integer'?'1':'any';if(meta.min!==undefined)input.min=meta.min;if(meta.max!==undefined)input.max=meta.max;input.required=kind!=='optional_number'&&kind!=='ref'&&kind!=='text';if(kind==='text')input.maxLength=2000;}
      input.id='mission-param-'+key;wrap.htmlFor=input.id;wrap.append(input);
      const helpKey=key.replace(/^_shared_/,'');
      if(helpKey==='forward_speed_m_s'&&b.type==='distance'){
        wrap.firstChild.textContent='恒定前后速度 / m/s（0 使用距离 PID）';
        meta={...meta,help:'大于 0 时按该速度前进或后退，进入距离容差后停止。4 表示下发前向或后向 4 m/s 的速度目标，不受距离 P/I/D 或搜索加速度斜坡影响；0 沿用距离 PID。实际速度由 PX4 跟踪。'};
      }
      let help=meta.help||fieldHelp[helpKey];
      if(helpKey==='height'&&b.type==='takeoff')help='相对本次起飞地面的上升量；例如 0.5 m 表示上升半米。';
      if(helpKey==='duration')help=visualTypes.has(b.type)?'有效跟随/保持的最短执行时间；还需同时满足误差与连续达标条件，目标不可用时间不计入。':'本积木最短执行时长；到位且稳定，并满足此时长后继续。';
      if(help)wrap.append(node('small',help,'mission-field-help'));
      host.append(wrap);controls.set(key,{input,kind});
    };
    addField('_name','显示名称','text',b.name);controls.get('_name').input.maxLength=100;controls.get('_name').input.required=true;
    addField('_timeout','积木超时 / s','number',b.timeout,{min:2,max:3600});
    const promotedShared=['distance','follow'].includes(b.type)?new Set(['target_distance_m','tag_size_m']):new Set();
    if(promotedShared.size){
      host.append(node('p','距离目标 · 与其他视觉任务共用，保存后同步','wide mission-param-section'));
      for(const key of promotedShared)addField('_shared_'+key,sharedFields[key],'number',sharedConfig[key],{min:0.000001});
      host.append(node('p','本积木的达标与丢失处理','wide mission-param-section'));
    }
    d.fields.filter(f=>!visualTypes.has(b.type)||!['adjust_speed','yaw_speed','target_distance','coarse_vertical_threshold'].includes(f.key)).forEach(f=>addField(f.key,f.label,f.kind,b.params[f.key]??f.default,f));
    if(visualTypes.has(b.type)){
      host.append(node('p','共享视觉控制参数 · 修改会同步其他视觉任务并保存为默认值。PID 调节响应，不设置最终保持距离。','wide mission-param-section'));
      Object.entries(sharedFields).filter(([key])=>!promotedShared.has(key)).forEach(([key,label])=>addField('_shared_'+key,label,key.endsWith('_enabled')?'boolean':'number',sharedConfig[key],key.includes('speed_')?{min:0}:key.endsWith('_m')?{min:0.000001}:{}));
    }
    addField('_next','执行结束后的任务','ref',b.auto_next===false?b.next:'__auto__',{empty:'结束任务'});
    const next=controls.get('_next').input;option(next,'__auto__','按列表下一项');next.value=b.auto_next===false?(b.next||''):'__auto__';
    el('mission-params-error').textContent='';el('mission-params-dialog').showModal();
  }
  const close=()=>el('mission-params-dialog').close();el('mission-params-close').onclick=el('mission-params-cancel').onclick=close;
  el('mission-params-form').onsubmit=async event=>{
    event.preventDefault();if(active()||paramSaving)return;const original=document_.blocks.find(b=>b.id===selected);if(!original)return;
    paramSaving=true;
    try{
      const value=key=>{const {input,kind}=controls.get(key);if(kind==='boolean')return input.checked;if(kind==='json')return JSON.parse(input.value);if(kind==='optional_number'&&input.value==='')return null;return ['number','integer','optional_number'].includes(kind)?Number(input.value):input.value;};
      const b=M.clone(original);b.name=value('_name');b.timeout=value('_timeout');
      definition(b).fields.forEach(f=>{if(controls.has(f.key))b.params[f.key]=value(f.key);});
      if(visualTypes.has(b.type)){
        const parameters={...await sharedRequest()};
        Object.keys(sharedFields).forEach(key=>parameters[key]=value('_shared_'+key));
        sharedConfig=await sharedRequest(parameters);
        b.params.adjust_speed=sharedConfig.forward_speed_m_s;b.params.yaw_speed=sharedConfig.yaw_speed_rad_s;b.params.target_distance=sharedConfig.target_distance_m;
        window.dispatchEvent(new CustomEvent('vision-defaults-changed'));
      }
      b.auto_next=value('_next')==='__auto__';b.next=b.auto_next?document_.blocks[document_.blocks.indexOf(original)+1]?.id||null:value('_next')||null;
      document_.blocks[document_.blocks.indexOf(original)]=b;close();changed();
      result(visualTypes.has(b.type)?'视觉参数已同步保存为开发板默认值；任务连接等编辑保留在草稿，点击保存任务持久化。':'参数已写入浏览器草稿；点击保存持久化到工程。启动前会验证字段、范围和所有 ID 引用。');
    }catch(error){el('mission-params-error').textContent='参数保存失败：'+error.message;}finally{paramSaving=false;}
  };
  el('mission-search').oninput=renderLibrary;
  el('mission-collapse').onclick=()=>{const hidden=!el('mission-library').hidden;el('mission-library').hidden=hidden;el('mission-collapse').textContent=hidden?'展开积木库':'折叠积木库';el('mission-collapse').setAttribute('aria-expanded',String(!hidden));};
  el('mission-name').oninput=()=>{document_.name=el('mission-name').value;remember();};
  el('mission-entry').onchange=()=>{document_.entry=el('mission-entry').value||null;changed();};
  function replace(next){if(active())return;M.mergeSearchRecognition(next);document_=next;changed();syncShared().catch(error=>result(error.message));}
  el('clear-mission').onclick=()=>{replace(M.empty());result('已清空当前草稿；已保存任务保留。');};
  el('preset-mission').onclick=()=>replace(M.template('classic',catalogue));
  el('mission-load-template').onclick=()=>replace(M.template(el('mission-template').value,catalogue));
  el('mission-copy').onclick=()=>{replace(M.duplicate(document_));result('已复制为新任务，所有内部 ID 引用已重新映射。');};
  const setMode=next=>{mode=next;el('mission-list').hidden=mode!=='list';el('mission-flow').hidden=mode!=='flow';for(const key of ['list','flow']){el('mission-'+key+'-mode').classList.toggle('selected',mode===key);el('mission-'+key+'-mode').setAttribute('aria-pressed',String(mode===key));}if(mode==='flow')renderFlow();};
  el('mission-list-mode').onclick=()=>setMode('list');el('mission-flow-mode').onclick=()=>setMode('flow');
  async function savedList(){
    const response=await request('list');const select=el('mission-saved'),old=select.value;select.replaceChildren();option(select,'','选择已保存任务');response.missions.forEach(m=>option(select,m.id,`${m.name} · ${m.blocks} 步`));select.value=old;
  }
  el('mission-save').onclick=()=>operation(async()=>{const value=await request('save',document_);document_=value.document;changed();await savedList();el('mission-saved').value=document_.id;result(value.message);});
  el('mission-load').onclick=()=>operation(async()=>{if(!el('mission-saved').value)throw new Error('请选择已保存任务');const value=await request('load?id='+encodeURIComponent(el('mission-saved').value));replace(value.document);result('已加载 '+document_.name);});
  el('mission-export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(document_,null,2)],{type:'application/json'})),a=node('a');a.href=url;a.download='competition-'+document_.id+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  el('mission-import-button').onclick=()=>el('mission-import').click();
  el('mission-import').onchange=()=>operation(async()=>{const file=el('mission-import').files[0];if(!file)return;if(file.size>262144)throw new Error('任务 JSON 超过 256KB');const imported=JSON.parse(await file.text());M.prune(imported);const value=await request('validate',imported);replace(value.document);result('JSON 已导入。'+(value.issues.length?'\n待接入：\n'+value.issues.join('\n'):''));el('mission-import').value='';});
  el('preview-mission').onclick=()=>operation(async()=>{
    const value=await request('validate',document_);document_=value.document;changed();
    const names=new Map(document_.blocks.map(b=>[b.id,b.name+' ['+b.id.slice(-8)+']'])),name=id_=>names.get(id_)||'结束';
    const lines=document_.blocks.map((b,i)=>`${i+1}. ${name(b.id)} → ${name(b.next)}`+
      (b.params.lost_target?'\n   连续丢失 → '+name(b.params.lost_target):''));
    result('入口：'+name(document_.entry)+'\n'+lines.join('\n')+(value.issues.length?'\n\n不可实机执行：\n'+value.issues.join('\n'):'\n\n已通过配置验证；实际执行仍需飞控反馈。')+'\n预览未发送控制命令。');
  });
  el('mission-start').onclick=()=>operation(async()=>{
    await syncShared();
    const checked=await request('validate',document_);if(checked.issues.length)throw new Error(checked.issues.join('\n'));
    document_=checked.document;changed();const value=await request('start',document_);run=value.mission;render();renderMonitor();result(value.message);
  });
  for(const [id_,stopMode] of [['mission-stop','hold'],['mission-land','land']])el(id_).onclick=()=>operation(async()=>{const value=await request('stop',{mode:stopMode});run=value.mission;renderMonitor();result(value.message);});
  el('mission-kill').onclick=async()=>{
    const button=el('mission-kill');if(button.disabled)return;
    button.disabled=true;result('正在请求强制上锁停转电机…');
    try{const value=await request('emergency-stop',{});result(value.message);toast(value.message);}
    catch(error){result('停转未确认：'+error.message);toast('停转未确认：'+error.message);}
    finally{button.disabled=false;}
  };
  el('mission-adopt-run').onclick=()=>{if(run?.document){document_=M.clone(run.document);changed();}};
  function renderMonitor(){
    const snapshot=typeof lastSnapshot==='undefined'?{}:lastSnapshot||{},px=snapshot.px4||{},v=snapshot.vision||{};
    const fresh=px.connected&&Number.isFinite(px.position_age_s)&&px.position_age_s<1;
    const current=run?.document?.blocks.find(b=>b.id===run.current_id);
    const retain=active()&&run.blocks?.[run.current_id]?.status==='waiting'&&run.retained_target&&Number.isFinite(current?.params.lost_timeout)&&(run.lost_elapsed_s||0)<current.params.lost_timeout;
    const tracking=snapshot.vision_tracking||{};
    const target=tracking.target_found?tracking:v.target_found?v:retain?run.retained_target:{};
    const retainedNote=target.target_retained||(retain&&!v.target_found)?'（保留）':'';
    const position=fresh?px.local_position||{}:{},center=Array.isArray(target.center)?target.center:null;
    const fmt=(n,unit='')=>Number.isFinite(n)?n.toFixed(2)+unit:'—';
    const speed=[position.vx,position.vy,position.vz].every(Number.isFinite)?Math.hypot(position.vx,position.vy,position.vz):null;
    const metrics=[['当前任务',run?.name||document_.name],['当前积木',run?.current_name||'—'],['积木状态',labels[run?.blocks?.[run?.current_id]?.status]||'—'],
      ['已完成 / 执行次数',`${Object.values(run?.blocks||{}).filter(b=>b.status==='succeeded').length} / ${run?.transitions||0}`],
      ['识别目标 ID',target.tag_id!==undefined?String(target.tag_id)+retainedNote:'—'],['中心偏差 / px',center?fmt(center[0]-(target.width-1)/2)+' / '+fmt(center[1]-(target.height-1)/2)+retainedNote:'—'],
      ['目标距离',target.pose_valid?fmt(target.distance_m,' m')+retainedNote:'—'],['相对起飞点高度',fmt(fresh?snapshot.control?.height_above_takeoff_ground_m:null,' m')],['当前速度',fmt(speed,' m/s')],
      ['偏航角',px.connected&&Number.isFinite(px.attitude_age_s)&&px.attitude_age_s<.5?fmt(px.attitude_deg?.[2],'°'):'—'],
      ['前后速度目标',fmt(snapshot.motion?.state==='running'?snapshot.motion?.vision_output?.forward_m_s:null,' m/s')],
      ['ROS2 通信',v.source==='ros2_cpp'&&v.camera_ready?'快照新鲜':'未就绪 / 过期'],['PX4 Offboard',px.connected?(px.mode==='OFFBOARD'?'OFFBOARD 在线':px.mode||'未知'):'飞控离线'],
      ['比赛用时',fmt(run?.elapsed_s,' s')],['目标丢失计时',fmt(run?.lost_elapsed_s||0,' s')],
      ['目标跟踪',tracking.target_observed?'真实检测':tracking.target_retained?'短暂保留':'等待检测'],
      ['有效识别累计',fmt(current?.type==='search'?run?.confirmation_seconds:tracking.confirmation_seconds,' s')],
      ['搜索策略',current?.type==='search'?({wide_scan:'广域扫描',search_approach:'连续缓慢接近',search_memory_forward:'沿记忆方向接近',search_reacquire:'原会话等待重捕获',confirmed:'目标已确认'}[run?.search_phase]||'搜索中'):'—']];
    el('mission-metrics').replaceChildren(...metrics.map(([label,value])=>{const item=node('span',label);item.append(node('b',value));return item;}));
    el('mission-run-title').textContent=run?.name?run.name+(active()?'':' · 上次运行'):'比赛任务运行监控';el('mission-run-state').textContent=labels[run?.state]||'待机';el('mission-run-state').className='mission-state '+(run?.state||'idle');
    el('mission-progress').style.width=Math.round(Math.max(0,Math.min(1,run?.progress||0))*100)+'%';
    el('mission-run-note').textContent=run?.message||'未启动比赛任务。编排保存与预览不会执行飞行。';
    const recording=snapshot.trajectory||{};
    el('mission-log-files').textContent='任务日志：'+(run?.log_file||'—')+'\n飞行 CSV：'+(recording.filename||recording.latest_complete||'—')+
      (run?.log_error?'\n任务日志保存异常：'+run.log_error:'')+(recording.error?'\nCSV 保存异常：'+recording.error:'');
    el('mission-events').replaceChildren(...(run?.events||[]).slice(-20).reverse().map(e=>node('li',`${fmt(e.elapsed_s,'s')} · ${e.event} · ${e.message||e.command||e.block_id?.slice(-8)||e.state||''}`)));
    buttons();
  }
  async function statusLoop(){
    try{
      const value=await request('status');run=value.mission;statusAvailable=true;
      if(active()&&run.run_id!==observedRunId&&run.document&&run.mission_id!==document_.id){
        document_=M.clone(run.document);render();
      }
      observedRunId=run.run_id;
      const fingerprint=JSON.stringify([run.mission_id,run.active,run.blocks]);
      if(fingerprint!==lastRunFingerprint){lastRunFingerprint=fingerprint;renderList();if(mode==='flow')renderFlow();renderLibrary();if(active()&&el('mission-params-dialog').open)close();}
      renderMonitor();updateControlButtons();
    }catch(error){statusAvailable=false;el('mission-run-note').textContent='比赛状态连接中断：'+error.message+'。显示的是上次状态；后端任务可能仍在运行，请恢复连接确认。';el('mission-run-state').textContent='状态连接中断';el('mission-run-state').className='mission-state waiting';el('mission-start').disabled=true;}
    setTimeout(statusLoop,active()?500:1000);
  }
  window.missionPanel={active};
  async function init(){
    try{const value=await request('catalog');catalogue=value.blocks;categories=value.categories;const removed=M.prune(document_);if(removed.length){remember();result('已删除不再保留的积木：'+removed.join('、')+'。请重新预览确认任务顺序。');}const merged=M.mergeSearchRecognition(document_);if(merged.length){remember();result('已将搜索与识别合并为“搜索并确认目标”，原识别时长已保留。');}ready=true;renderCategories();render();await savedList();await syncShared();}
    catch(error){result('比赛模块尚未连接：'+error.message);setTimeout(init,3000);}
  }
  renderMonitor();init();statusLoop();
})();
