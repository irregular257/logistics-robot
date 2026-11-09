'use strict';
// Start a dedicated local simulated server; no physical device is contacted.
const assert = require('node:assert/strict');
const {spawn} = require('node:child_process');
const path = require('node:path');
const net = require('node:net');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

(async () => {
  const port = await new Promise(resolve => {
    const probe = net.createServer();
    probe.listen(0, '127.0.0.1', () => {
      const value = probe.address().port;
      probe.close(() => resolve(value));
    });
  });
  const child = spawn(process.execPath, [path.resolve(__dirname, '../ESP32/网页预览/server.cjs')], {
    env: {...process.env, ROBOT_PREVIEW_PORT:String(port)}, windowsHide:true,
    stdio:['ignore','pipe','pipe']
  });
  try {
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error('Preview did not start')),5000);
      child.stdout.once('data', () => { clearTimeout(timer); resolve(); });
      child.once('exit', code => {clearTimeout(timer); reject(new Error('Preview exit '+code));});
    });
    const base = 'http://127.0.0.1:'+port;
    let token='',seq=0;
    async function api(route, fields={}, method='POST') {
      const response = await fetch(base+route,{method,headers:{'Content-Type':'application/x-www-form-urlencoded','X-Token':token},body:method==='POST'?new URLSearchParams({seq:++seq,...fields}):undefined});
      return {code:response.status,data:await response.json()};
    }
    assert.equal((await api('/api/claw-angle',{angle:125})).code,403);
    assert.equal((await api('/api/tray-angle',{angle:120})).code,403);
    token=(await api('/api/claim')).data.token;
    assert.equal((await api('/api/claw-angle',{angle:125})).code,409);
    assert.equal((await api('/api/tray-angle',{angle:120})).code,409);
    assert.equal((await api('/api/mode',{value:1})).code,202);
    for(const angle of [0,270])assert.equal((await api('/api/gimbal',{angle})).code,202);
    for(const angle of [-1,271,1.5,''])assert.equal((await api('/api/gimbal',{angle})).code,400);
    for(const angle of [0,120,270])assert.equal((await api('/api/tray-angle',{angle})).code,202);
    for(const angle of [-1,271,1.5,'','NaN'])assert.equal((await api('/api/tray-angle',{angle})).code,400);
    let trayStatus=(await api('/api/status',{},'GET')).data;
    assert.equal(trayStatus.traySupported,true); assert.equal(trayStatus.trayAngle,270); assert.equal(trayStatus.lastAckCmd,13);
    assert.equal(trayStatus.gimbalAngle,270,'Tray control must not move the gimbal');
    assert.equal((await api('/api/claw-angle',{angle:125})).code,202);
    let status=(await api('/api/status',{},'GET')).data;
    assert.equal(status.clawAngle,125); assert.equal(status.lastAckCmd,11);
    const presets={clawClose:20,clawOpen:125,gimbalIn:240,gimbalOut:170};
    assert.equal((await api('/api/servo-presets',presets)).code,202);
    status=(await api('/api/status',{},'GET')).data;
    assert.deepEqual(status.presets,presets);assert.equal(status.presetsSaved,true);assert.equal(status.lastAckCmd,12);
    assert.equal(status.gimbalAngle,270,'Saving an angle must not move the simulated servo');
    assert.equal(status.clawAngle,125);
    assert.equal((await api('/api/claw',{value:0})).code,202);
    assert.equal((await api('/api/status',{},'GET')).data.clawAngle,20);
    const lift={x:0,y:0,turn:0,lift:1};
    assert.equal((await api('/api/jog',lift)).code,202);
    assert.equal((await api('/api/gimbal',{angle:170})).code,409,'Lift movement interlocks servo actions');
    assert.equal((await api('/api/tray-angle',{angle:120})).code,409,'Lift movement interlocks tray actions');
    await delay(250);
    assert.equal((await api('/api/status',{},'GET')).data.lift,0,'A finite jog completes while held');
    await api('/api/jog',lift);
    assert.equal((await api('/api/status',{},'GET')).data.lift,0,'Held repetition cannot start another finite jog');
    await api('/api/jog',{...lift,lift:0});await api('/api/jog',lift);
    assert.equal((await api('/api/status',{},'GET')).data.lift,1,'Release rearms the next finite jog');
    assert.equal((await api('/api/jog',{...lift,y:20})).code,400);
    assert.equal((await api('/api/claw-angle',{},'GET')).code,405);
    assert.equal((await api('/api/tray-angle',{},'GET')).code,405);
    await api('/api/jog',{x:0,y:0,turn:0,lift:0});
    const plan={color1:4,shape1:1,color2:7,shape2:4,color3:3,shape3:3};
    assert.equal((await api('/api/start',{positioned:1,...plan})).code,409,'Task must be submitted first');
    for(const bad of [{shape1:2},{shape1:7},{color3:5},{color3:6},{color1:1},{color3:7},{color2:3},{shape3:''}]) {
      assert.equal((await api('/api/task',{...plan,...bad})).code,400);
    }
    assert.equal((await api('/api/task',plan)).code,202);
    status=(await api('/api/status',{},'GET')).data;
    assert.equal(status.taskAccepted,true);assert.equal(status.lastAckCmd,7);assert.deepEqual(status.taskPlan,[4,1,7,4,3,3]);
    assert.equal((await api('/api/start',{positioned:1,...plan,color2:1})).code,409,'Edited plan must be resent');
    assert.equal((await api('/api/start',{...plan})).code,400,'Initial pose must be confirmed');
    assert.equal((await api('/api/start',{positioned:1,...plan})).code,202);
    assert.equal((await api('/api/task',plan)).code,409,'Running task cannot be edited');
    assert.equal((await api('/api/status',{},'GET')).data.trayAngle,-1,'Automatic ownership clears the manual target');
    assert.equal((await api('/api/tray-angle',{angle:120})).code,409);
    assert.equal((await api('/api/mode',{value:1})).code,409,'Manual mode cannot take over an automatic task');
    await api('/api/manual-release');
    status=(await api('/api/status',{},'GET')).data;
    assert.equal(status.mode,2,'Page lifecycle releases manual ownership while the main controller keeps running');
    assert.equal(status.autoState,1);
    assert.equal(status.ownerActive,false);
    assert.equal(status.taskAccepted,true);
    assert.deepEqual(status.taskPlan,[4,1,7,4,3,3]);
    token=(await api('/api/claim')).data.token;
    await api('/api/release');
    assert.equal((await api('/api/status',{},'GET')).data.mode,2,'Exit control preserves the automatic task');
    token=(await api('/api/claim')).data.token;
    assert.equal((await api('/api/pause')).code,202);
    assert.equal((await api('/api/status',{},'GET')).data.autoState,14,'Explicit cancel ends automatic execution');
    assert.equal((await api('/api/start',{positioned:1,...plan})).code,202,'Explicit cancel retains the accepted plan');
    await api('/api/stop');
    status=(await api('/api/status',{},'GET')).data;
    assert.equal(status.lift,0);
    assert.equal(status.autoState,14,'Explicit STOP remains global');
    assert.equal((await api('/api/servo-presets',presets)).code,403);
    console.log('PASS local preview API: ordered task parameters/validation/start guards, tray/servo bounds, ownership, presets, interlocks and finite lift');
  } finally { child.kill(); }
})().catch(error => {console.error(error);process.exitCode=1;});
