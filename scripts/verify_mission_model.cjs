const assert=require('node:assert/strict');
const M=require('../web/mission_model.js');
const catalogue=require('../config/mission_blocks.json').blocks;
assert.equal(catalogue.length,17);
for(const key of ['classic','indoor']){
 const doc=M.template(key,catalogue),copy=M.duplicate(doc),ids=new Set(copy.blocks.map(b=>b.id));
 assert(ids.has(copy.entry));assert.notEqual(copy.entry,doc.entry);
 for(const b of copy.blocks){if(b.next)assert(ids.has(b.next));if(b.params.lost_target)assert(ids.has(b.params.lost_target));assert(catalogue.some(d=>d.type===b.type));}
 assert(!doc.blocks.some(b=>b.type==='recognize'||b.type==='lock'));
}
const legacy=M.template('classic',catalogue),search=legacy.blocks[0],align=legacy.blocks[1];
const recognition=M.make(catalogue.find(b=>b.type==='recognize'));
recognition.params.settle_seconds=.8;recognition.next=align.id;
legacy.blocks.splice(1,0,recognition);search.next=recognition.id;
assert.equal(M.mergeSearchRecognition(legacy).length,1);
assert.equal(legacy.blocks.length,3);assert.equal(search.next,align.id);
assert.equal(search.params.confirm_seconds,.8);
assert.deepEqual(M.mergeSearchRecognition(legacy),[]);
const addressed=M.clone(legacy);addressed.blocks.splice(1,0,recognition);
addressed.blocks[0].next=recognition.id;addressed.entry=recognition.id;
assert.deepEqual(M.mergeSearchRecognition(addressed),[]);
const doc=M.template('classic',catalogue),first=doc.blocks[0];
doc.blocks.splice(1,0,{id:'old-score',type:'score',name:'比赛计分',next:doc.blocks[1].id});first.next='old-score';
assert.deepEqual(M.prune(doc),['比赛计分']);assert.equal(doc.blocks.length,3);assert.equal(first.next,doc.blocks[1].id);
M.reorder(doc,doc.blocks[2].id,doc.blocks[0].id);assert.equal(doc.entry,doc.blocks[0].id);
console.log('PASS: 17 flight/vision blocks, two templates, copy IDs, retired draft cleanup and sorting');
