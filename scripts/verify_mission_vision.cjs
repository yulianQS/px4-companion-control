// Local browser only: camera frames and emergency-stop requests are mocked.
const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'F:/codex_pro/cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe'});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1100}});
  const errors=[];page.on('pageerror',error=>errors.push(error.message));
  await page.addInitScript(()=>{
   const original=window.fetch;
   window.testStreamRequests=0;window.testFreeze=false;
   window.fetch=(input,options)=>{
    if(input!=='/api/vision/stream.mjpg')return original(input,options);
    window.testStreamRequests++;
    const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
    const context=canvas.getContext('2d');context.fillStyle='#16808a';context.fillRect(0,0,640,480);
    const jpeg=Uint8Array.from(atob(canvas.toDataURL('image/jpeg').split(',')[1]),c=>c.charCodeAt(0));
    const header=new TextEncoder().encode('--visionframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+jpeg.length+'\r\n\r\n');
    const part=new Uint8Array(header.length+jpeg.length+2);part.set(header);part.set(jpeg,header.length);part.set([13,10],part.length-2);
    let timer;
    return Promise.resolve(new Response(new ReadableStream({start(controller){
     timer=setInterval(()=>{if(!window.testFreeze)controller.enqueue(part);},100);
     options.signal.addEventListener('abort',()=>{clearInterval(timer);controller.error(new DOMException('Aborted','AbortError'));});
    },cancel(){clearInterval(timer);}}),{headers:{'Content-Type':'multipart/x-mixed-replace; boundary=visionframe'}}));
   };
  });
  let stops=0;
  await page.route('**/api/mission/emergency-stop',route=>{stops++;return route.fulfill({json:{ok:true,message:'mock emergency-stop received'}});});
  await page.goto('http://127.0.0.1:8082/#mission');
  await page.waitForFunction(()=>!document.getElementById('mission-camera-image').hidden);
  assert.equal(await page.evaluate(()=>window.testStreamRequests),1);
  const pixels=await page.evaluate(()=>['camera-image','mission-camera-image'].map(id=>Array.from(document.getElementById(id).getContext('2d').getImageData(1,1,1,1).data)));
  assert.deepEqual(pixels[0],pixels[1]);assert(pixels[1][1]>80);
  assert(await page.locator('#mission-kill').isVisible());
  await page.locator('#mission-kill').click();assert.equal(stops,1);
  await page.waitForFunction(()=>!document.getElementById('mission-kill').disabled);
  await page.setViewportSize({width:390,height:844});
  assert(await page.locator('#mission-camera-image').isVisible());
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
  await page.evaluate(()=>{window.testFreeze=true;});
  await page.waitForFunction(()=>document.getElementById('mission-camera-image').hidden,{},{timeout:5000});
  assert(await page.locator('#mission-camera-placeholder').isVisible());
  assert(await page.locator('#mission-kill').isEnabled());
  assert.deepEqual(errors,[]);
  console.log('PASS: shared single stream, matching frames, stale-frame hiding, mobile layout and mocked emergency stop');
 }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
