/* Shared mission graph for list/flow views, persistence and templates. No control code. */
((root) => {
  'use strict';
  const clone = value => JSON.parse(JSON.stringify(value));
  const id = () => 'm-' + Date.now().toString(36) + '-' + Array.from(crypto.getRandomValues(new Uint8Array(10)), n => n.toString(16).padStart(2, '0')).join('');
  const empty = () => ({version:1,id:id(),name:'室内比赛任务',entry:null,blocks:[]});
  function make(definition) {
    return {id:id(),type:definition.type,name:definition.name,
      params:Object.fromEntries(definition.fields.map(f => [f.key,clone(f.default)])),
      status:'pending',next:null,branches:{},timeout:120,auto_next:true};
  }
  function wire(document) {
    document.blocks.forEach((b,i) => { if(b.auto_next !== false)b.next=document.blocks[i+1]?.id || null; });
    if(!document.blocks.some(b=>b.id===document.entry))document.entry=document.blocks[0]?.id || null;
    return document;
  }
  function reorder(document, dragged, before) {
    const entryWasFirst=document.entry===document.blocks[0]?.id;
    const index=document.blocks.findIndex(b=>b.id===dragged);
    if(index<0||dragged===before)return;
    const [item]=document.blocks.splice(index,1);
    const target=document.blocks.findIndex(b=>b.id===before);
    document.blocks.splice(target<0?document.blocks.length:target,0,item);wire(document);
    if(entryWasFirst)document.entry=document.blocks[0]?.id||null;
  }
  function remove(document,id_) {
    document.blocks=document.blocks.filter(b=>b.id!==id_);
    for(const b of document.blocks){
      if(b.next===id_){b.next=null;b.auto_next=true;}
      if(b.params.lost_target===id_)b.params.lost_target='';
    }
    wire(document);
  }
  function duplicate(document) {
    const result=clone(document), mapping=new Map(result.blocks.map(b=>[b.id,id()]));
    result.id=id();result.name+=' · 副本';result.entry=mapping.get(result.entry)||null;
    for(const b of result.blocks){
      b.id=mapping.get(b.id);b.status='pending';b.next=mapping.get(b.next)||null;
      if(b.params.lost_target)b.params.lost_target=mapping.get(b.params.lost_target)||'';
    }
    return result;
  }
  function template(name,definitions) {
    const catalogue=new Map(definitions.map(b=>[b.type,b]));
    const document=empty();
    const types={classic:['search','align','distance'],
      indoor:['takeoff','trajectory','search','align','distance','land']}[name]||[];
    document.name={classic:'搜索 → 对准 → 接近',indoor:'室内单目标飞行与视觉流程'}[name]||'示例任务';
    document.blocks=types.map(t=>make(catalogue.get(t)));wire(document);
    if(name==='indoor'){
      const search=document.blocks.find(b=>b.type==='search');
      document.blocks.forEach(b=>{if('lost_target' in b.params)b.params.lost_target=search.id;});
    }
    return document;
  }
  const removedTypes=new Set(['ring','inspection','checkpoint','sort_targets','contact','timed','action','condition','loop','wait_event','delay','jump','subtask','end','record_trajectory','record_target','timer','score','log']);
  function prune(document) {
    const original=document.blocks, removed=original.filter(b=>removedTypes.has(b.type));
    if(!removed.length)return [];
    const lookup=new Map(original.map(b=>[b.id,b]));
    const resolve=id_=>{const visited=new Set();while(id_&&removedTypes.has(lookup.get(id_)?.type)){if(visited.has(id_))return null;visited.add(id_);id_=lookup.get(id_)?.next;}return id_||null;};
    document.blocks=original.filter(b=>!removedTypes.has(b.type));
    document.entry=resolve(document.entry);
    document.blocks.forEach(b=>{b.next=resolve(b.next);if(b.params?.lost_target)b.params.lost_target=resolve(b.params.lost_target)||'';});
    wire(document);return removed.map(b=>b.name);
  }
  function mergeSearchRecognition(document) {
    const lookup=new Map(document.blocks.map(b=>[b.id,b])),merged=[];
    for(const search of document.blocks){
      const recognition=lookup.get(search.next);
      if(search.type!=='search'||recognition?.type!=='recognize')continue;
      // Preserve separately addressed recognition nodes and different targets.
      if(document.entry===recognition.id||document.blocks.some(b=>b!==search&&(b.next===recognition.id||b.params?.lost_target===recognition.id)))continue;
      if((search.params.target_id??0)!==(recognition.params.target_id??0)||(search.params.target_type??'apriltag36h11')!==(recognition.params.target_type??'apriltag36h11'))continue;
      search.params.confirm_seconds=recognition.params.settle_seconds??0.5;
      search.name='搜索并确认目标';search.next=recognition.next;
      search.auto_next=recognition.auto_next;
      document.blocks=document.blocks.filter(b=>b!==recognition);
      merged.push(recognition.name);
    }
    return merged;
  }
  const api={prune,mergeSearchRecognition,clone,id,empty,make,wire,reorder,remove,duplicate,template};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.MissionModel=api;
})(globalThis);
