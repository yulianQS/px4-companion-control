const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(require('path').resolve(__dirname,'../web/vision_ros.js'),'utf8');
const jpeg=Buffer.from([255,216,1,2,255,217]);
const packet=Buffer.concat([Buffer.from('--visionframe\r\nContent-Type: image/jpeg\r\nContent-Length: 6\r\n\r\n'),jpeg,Buffer.from('\r\n')]);
const elements={};let draws=0,connections=0,aborts=0;const requests=[];
const document={hidden:false,getElementById(id){return elements[id]??=(id==='camera-image'?{width:640,height:480,getContext:()=>({drawImage(){draws++;}})}:{});},addEventListener(){}};
const timers=new Set();
const scope={document,window:{},Uint8Array,TextDecoder,Blob,AbortController,performance,console,
 setTimeout(fn,ms){const t=setTimeout(fn,ms);timers.add(t);return t;},clearTimeout,
 setInterval(fn,ms){const t=setInterval(fn,ms);timers.add(t);return t;},
 createImageBitmap:async()=>({width:640,height:480,close(){}}),
 async fetch(url,{signal}) {
  requests.push(url);
  if(url.endsWith('/status')) return {ok:true,json:async()=>({message:'yaw_preview_only'})};
  connections++;const n=connections;let reads=0;
  return {ok:true,headers:{get:()=> 'multipart/x-mixed-replace; boundary=visionframe'},body:{getReader:()=>({
   async read(){
    reads++;
    if(n===1){if(reads===1)return {value:packet,done:false};return {done:true};}
    if(n>=3 && reads===1)return {value:packet,done:false};
    return new Promise((resolve,reject)=>{if(signal.aborted)return reject(new Error('aborted'));signal.addEventListener('abort',()=>{aborts++;reject(new Error('aborted'));},{once:true});});
   },async cancel(){}
  })}};
 }};
vm.createContext(scope);
// Exercise the shipped parser against arbitrary transport splits, combined frames, malformed lengths.
vm.runInContext(source.replace('  let controller =', '  window.TestDecoder = MjpegDecoder;\n  let controller ='),scope);
const Decoder=scope.window.TestDecoder;
let d=new Decoder(),result=null;
for(let i=0;i<packet.length;i++){const x=d.push(new Uint8Array(packet.subarray(i,i+1)));if(x)result=x;}
assert.deepStrictEqual(Buffer.from(result),jpeg);
d=new Decoder();assert.deepStrictEqual(Buffer.from(d.push(new Uint8Array(Buffer.concat([packet,packet])))),jpeg);
assert.throws(()=>new Decoder().push(new TextEncoder().encode('--visionframe\r\nContent-Type: image/jpeg\r\nContent-Length: 9999999\r\n\r\n')));
setTimeout(()=>{
 try{
  assert(connections>=3,`expected EOF+stall reconnects, got ${connections}`);
  assert(aborts>=1,'frame stall did not abort transport');assert(draws>=2,'did not recover rendering');
  assert(elements['vision-preview'].textContent.includes('0.000'));
  assert(requests.every(url=>['/api/vision/status','/api/vision/stream.mjpg'].includes(url)), 'Display failure must never send a flight/control/config request');
  console.log('PASS: split/combined MJPEG, invalid size rejection, silent EOF reconnect, 2s stall watchdog, rendering recovery');
 }catch(e){console.error(e);process.exitCode=1;}
 finally{document.hidden=true;for(const t of timers){clearTimeout(t);clearInterval(t);}}
},3900);
