'use strict';

// Run with Node.js. This exercises the embedded page in a simulated browser;
// it never contacts an ESP32 or sends commands to physical hardware.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const pagePath = path.resolve(__dirname, '../ESP32/robot_WebControl/web_page.h');
const source = fs.readFileSync(pagePath, 'utf8');
const match = source.match(/<script>([\s\S]*?)<\/script>/);
assert(match, 'The embedded page must contain its JavaScript');
const script = match[1];
new vm.Script(script, { filename: 'embedded-web-page.js' });
assert(!/(?:src|href)\s*=\s*["']https?:/i.test(source), 'The page must not load external resources');

class Element {
  constructor(id) {
    this.id = id;
    this.value = '';
    this.disabled = false;
    this.textContent = '';
    this.dataset = {};
    this.handlers = new Map();
    this.classes = new Set();
    this.classList = {
      add: name => this.classes.add(name),
      remove: name => this.classes.delete(name),
      toggle: (name, on) => on ? this.classes.add(name) : this.classes.delete(name)
    };
  }

  addEventListener(type, handler) {
    if (!this.handlers.has(type)) this.handlers.set(type, []);
    this.handlers.get(type).push(handler);
  }

  emit(type, extra = {}) {
    const event = { pointerId: 1, button: 0, preventDefault() {}, ...extra };
    for (const handler of this.handlers.get(type) || []) handler(event);
  }

  setPointerCapture(id) { this.capture = id; }
  hasPointerCapture(id) { return this.capture === id; }
  releasePointerCapture(id) {
    this.capture = null;
    this.emit('lostpointercapture', { pointerId: id });
  }
}

function response(data, status = 200) {
  return { ok: status >= 200 && status < 300, status, text: async () => JSON.stringify(data) };
}

function harness() {
  const elements = {};
  for (const [, id] of source.matchAll(/\bid="([^"]+)"/g)) elements[id] = new Element(id);
  elements.speed.value = '20';
  elements.gimbalAngle.value = '54';
  elements.clawAngle.value = '45';
  elements.trayAngle.value = '120';
  for (let i=1;i<=3;++i) { elements['color'+i].value='4'; elements['shape'+i].value='1'; }
  const buttons = Array.from(source.matchAll(/data-motion="([^"]+)"/g), ([, name]) => {
    const element = new Element(name);
    element.dataset.motion = name;
    return element;
  });
  const document = {
    hidden: false, handlers: new Map(),
    getElementById: id => elements[id], querySelectorAll: () => buttons,
    addEventListener(type, handler) {
      if (!this.handlers.has(type)) this.handlers.set(type, []);
      this.handlers.get(type).push(handler);
    },
    emit(type) { for (const handler of this.handlers.get(type) || []) handler(); }
  };
  const window = {
    handlers: new Map(),
    addEventListener(type, handler) {
      if (!this.handlers.has(type)) this.handlers.set(type, []);
      this.handlers.get(type).push(handler);
    },
    emit(type, event) { for (const handler of this.handlers.get(type) || []) handler(event); }
  };
  const server = {
    state: {
      link: true, claimed: false, mine: false, mode: 0, armed: false,
      fault: 0, claw: 2, liftEnabled: false, gimbalSupported: true, gimbalAngle: -1, pending: false,
      clawAngle: -1, servoPresetsSupported: true, presetStorageReady: true, presetsSaved: false,
      traySupported: true, trayAngle: -1,
      presets: {clawClose: 10, clawOpen: 70, gimbalIn: 54, gimbalOut: 170},
      x: 0, y: 0, turn: 0, lift: 0,
      lastAckSequence: 0, lastAckCommand: 0, lastAckResult: 0,
      taskSupported:true, taskAccepted:false, taskPending:false, taskPlan:[0,0,0,0,0,0]
    },
    deferClaim: false, deferStatus: false, deferJog: false, deferGimbal: false, deferTray: false,
    ignoreJogAbort: false, failStatus: false, claims: 0
  };
  let now = 1000, timerId = 0, uartSequence = 10;
  const requests = [], intervals = [], timeouts = new Map();
  function deferred(request, data, code, ignoreAbort = false, beforeResolve = () => {}) {
    return new Promise((resolve, reject) => {
      request.resolve = () => { beforeResolve(); resolve(response(data, code)); };
      request.reject = reject;
      if (!ignoreAbort) request.signal.addEventListener('abort', () => {
        const error = new Error('Aborted');
        error.name = 'AbortError';
        reject(error);
      }, { once: true });
    });
  }
  function revoke() {
    server.state.claimed = server.state.mine = server.state.armed = server.state.pending = false;
    server.state.mode = 0;
  }
  function fetchMock(url, options) {
    const request = {
      path: url, method: options.method, headers: options.headers,
      fields: Object.fromEntries(new URLSearchParams(options.body || '')),
      signal: options.signal
    };
    requests.push(request);
    if (url === '/api/status') {
      const snapshot = { ...server.state };
      if (server.failStatus) {
        server.failStatus = false;
        return Promise.reject(new Error('Simulated connection failure'));
      }
      if (server.deferStatus) {
        server.deferStatus = false;
        return deferred(request, snapshot, 200);
      }
      return Promise.resolve(response(snapshot));
    }
    if (url === '/api/claim') {
      const token = (0x1234567890abcdefn + BigInt(server.claims++)).toString(16).padStart(16, '0');
      const claim = () => { server.state.claimed = server.state.mine = true; };
      if (server.deferClaim) {
        server.deferClaim = false;
        return deferred(request, { ok: true, token }, 200, false, claim);
      }
      claim();
      return Promise.resolve(response({ ok: true, token }));
    }
    if (url === '/api/mode') {
      server.state.pending = true;
      return Promise.resolve(response({ ok: true, queued: true }, 202));
    }
    if (url === '/api/stop' || url === '/api/release') {
      revoke();
      return Promise.resolve(response({ ok: true }));
    }
    if (url === '/api/jog' && server.deferJog) {
      server.deferJog = false;
      return deferred(request, { ok: true, queued: true }, 202, server.ignoreJogAbort);
    }
    if (url === '/api/task') {
      request.uartSeq = ++uartSequence;
      return Promise.resolve(response({ok:true,queued:true,uartSeq:request.uartSeq},202));
    }
    if (url === '/api/claw') {
      request.uartSeq = ++uartSequence;
      return Promise.resolve(response({ ok: true, queued: true, uartSeq: request.uartSeq }, 202));
    }
    if (url === '/api/gimbal') {
      const angle = Number(request.fields.angle);
      assert(Number.isInteger(angle) && angle >= (server.state.servoPresetsSupported ? 0 : 54) && angle <= (server.state.servoPresetsSupported ? 270 : 229));
      assert(server.state.gimbalSupported && server.state.mode === 1 && server.state.armed);
      request.uartSeq = ++uartSequence;
      if (server.deferGimbal) {
        server.deferGimbal = false;
        return deferred(request, { ok: true, queued: true, uartSeq: request.uartSeq }, 202);
      }
      return Promise.resolve(response({ ok: true, queued: true, uartSeq: request.uartSeq }, 202));
    }
    if (url === '/api/claw-angle' || url === '/api/servo-presets') {
      assert(server.state.servoPresetsSupported && server.state.mode === 1 && server.state.armed);
      const angles = url === '/api/claw-angle' ? [request.fields.angle] : ['clawClose', 'clawOpen', 'gimbalIn', 'gimbalOut'].map(key => request.fields[key]);
      assert(angles.every(value => /^\d+$/.test(value) && Number(value) <= 270));
      request.uartSeq = ++uartSequence;
      return Promise.resolve(response({ ok: true, queued: true, uartSeq: request.uartSeq }, 202));
    }
    if (url === '/api/tray-angle') {
      assert(server.state.traySupported && server.state.mode === 1 && server.state.armed);
      assert(/^\d+$/.test(request.fields.angle) && Number(request.fields.angle) <= 270);
      request.uartSeq = ++uartSequence;
      if (server.deferTray) {
        server.deferTray = false;
        return deferred(request, {ok:true, queued:true, uartSeq:request.uartSeq}, 202);
      }
      return Promise.resolve(response({ok:true, queued:true, uartSeq:request.uartSeq}, 202));
    }
    if (url === '/api/start') {
      assert.equal(request.fields.positioned, '1');
      server.state.mode=2; server.state.armed=false; server.state.autoState=1;
      request.uartSeq = ++uartSequence;
      server.state.lastAckSequence=request.uartSeq; server.state.lastAckCommand=8; server.state.lastAckResult=0; server.state.ackValid=true;
      return Promise.resolve(response({ok:true,queued:true,uartSeq:request.uartSeq},202));
    }
    if (url === '/api/pause') {
      server.state.mode=0; server.state.autoState=14;
      return Promise.resolve(response({ok:true,queued:true},202));
    }
    return Promise.resolve(response({ ok: true }, url === '/api/jog' || url === '/api/claw' ? 202 : 200));
  }
  const context = {
    document, window, fetch: fetchMock, URLSearchParams, AbortController, Blob,
    Date: { now: () => now },
    navigator: {
      sendBeacon(url) { requests.push({ path: url, beacon: true }); revoke(); return true; }
    },
    setInterval(fn, ms) { intervals.push({ fn, ms }); },
    setTimeout(fn, ms) { const id = ++timerId; timeouts.set(id, { fn, ms }); return id; },
    clearTimeout(id) { timeouts.delete(id); }
  };
  // No persistent browser storage is supplied: any attempted token persistence
  // would fail the tests rather than silently restore control on a reload.
  vm.runInNewContext(script, context);
  const env = {
    elements, buttons, document, window, server, requests, timeouts,
    motion: name => buttons.find(button => button.dataset.motion === name),
    matching: url => requests.filter(request => request.path === url),
    async acknowledge(request, result = 0) {
      server.state.lastAckSequence = request.uartSeq;
      server.state.lastAckCommand = {'/api/claw':6, '/api/gimbal':10, '/api/claw-angle':11, '/api/servo-presets':12, '/api/tray-angle':13}[request.path];
      server.state.lastAckResult = result;
      server.state.ackValid = true;
      if (result === 0 && request.path === '/api/claw-angle') server.state.clawAngle = Number(request.fields.angle);
      if (result === 0 && request.path === '/api/tray-angle') server.state.trayAngle = Number(request.fields.angle);
      if (result === 0 && request.path === '/api/servo-presets') {
        server.state.presets = Object.fromEntries(Object.entries(request.fields).filter(([key]) => key !== 'seq').map(([key, value]) => [key, Number(value)]));
        server.state.presetsSaved = true;
      }
      await this.tick(300);
    },
    async flush() { for (let n = 0; n < 30; ++n) await Promise.resolve(); },
    async tick(ms) {
      now += ms;
      const interval = intervals.find(item => item.ms === ms);
      assert(interval, `Expected an interval of ${ms} ms`);
      interval.fn();
      await this.flush();
    },
    async manual() {
      await this.flush();
      elements.claim.emit('click');
      await this.flush();
      await this.tick(300);
      assert.equal(elements.manual.disabled, false, 'The owner may request manual mode');
      elements.manual.emit('click');
      await this.flush();
      assert.equal(this.motion('forward').disabled, true, 'HTTP 202 is not a main-controller acknowledgement');
      server.state.mode = 1;
      server.state.armed = true;
      server.state.pending = false;
      ++server.state.lastAckSequence;
      await this.tick(300);
      assert.equal(this.motion('forward').disabled, false, 'An acknowledged manual mode permits movement');
    }
  };
  return env;
}

const vector = request => ['x', 'y', 'turn', 'lift'].map(key => Number(request.fields[key]));
const tests = [];
function test(name, run) { tests.push({ name, run }); }

test('ACK gating, authenticated status, six directions and zero release', async () => {
  const env = harness();
  await env.flush();
  assert.equal(env.motion('forward').disabled, true);
  await env.manual();
  assert.equal(env.matching('/api/status').find(request => request.headers['X-Token']).headers['X-Token'], '1234567890abcdef');
  const directions = {
    forward: [0, 20, 0, 0], backward: [0, -20, 0, 0],
    left: [-20, 0, 0, 0], right: [20, 0, 0, 0],
    turnLeft: [0, 0, 20, 0], turnRight: [0, 0, -20, 0]
  };
  for (const [name, expected] of Object.entries(directions)) {
    env.motion(name).emit('pointerdown');
    await env.flush();
    const moving = env.matching('/api/jog').at(-1);
    assert.deepEqual(vector(moving), expected, name);
    env.motion(name).emit('pointerup');
    await env.flush();
    const zero = env.matching('/api/jog').at(-1);
    assert.deepEqual(vector(zero), [0, 0, 0, 0]);
    assert(Number(zero.fields.seq) > Number(moving.fields.seq));
    assert.equal(env.server.state.mode, 1, 'A normal release keeps manual mode');
    assert.equal(env.elements.release.disabled, false, 'A normal release keeps ownership');
  }
  const controlled = env.requests.filter(request => request.path === '/api/mode' || request.path === '/api/jog');
  controlled.forEach((request, index) => {
    assert.equal(request.method, 'POST');
    assert.equal(request.headers['Content-Type'], 'application/x-www-form-urlencoded');
    if (index) assert(Number(request.fields.seq) > Number(controlled[index - 1].fields.seq));
  });
  assert.equal(env.matching('/api/stop').length, 0);
});

test('Lift is disabled by default and uses a separate direction when enabled', async () => {
  const env = harness();
  await env.manual();
  assert.equal(env.motion('liftUp').disabled, true);
  assert.equal(env.motion('liftDown').disabled, true);
  env.server.state.liftEnabled = true;
  await env.tick(300);
  for (const [name, lift] of [['liftUp', 1], ['liftDown', -1]]) {
    assert.equal(env.motion(name).disabled, false);
    env.motion(name).emit('pointerdown');
    await env.flush();
    assert.deepEqual(vector(env.matching('/api/jog').at(-1)), [0, 0, 0, lift]);
    env.motion(name).emit('pointerup');
    await env.flush();
  }
});

test('A second pointer cannot combine motion or release the first pointer', async () => {
  const env = harness();
  await env.manual();
  env.motion('forward').emit('pointerdown');
  await env.flush();
  const count = env.requests.length;
  env.motion('left').emit('pointerdown', { pointerId: 2 });
  await env.flush();
  assert.equal(env.requests.length, count);
  env.motion('left').emit('pointerup', { pointerId: 2 });
  await env.flush();
  assert.equal(env.requests.length, count);
  env.motion('forward').emit('pointercancel');
  await env.flush();
  assert.equal(env.requests.length, count + 1, 'Capture loss during release must not send a duplicate zero');
  assert.deepEqual(vector(env.matching('/api/jog').at(-1)), [0, 0, 0, 0]);
});

test('Zero release passes a pending JOG and a late response cannot revive motion', async () => {
  const env = harness();
  await env.manual();
  env.server.deferJog = env.server.ignoreJogAbort = true;
  env.motion('forward').emit('pointerdown');
  await env.flush();
  const old = env.matching('/api/jog').at(-1);
  const count = env.requests.length;
  await env.tick(100);
  await env.tick(100);
  assert.equal(env.requests.length, count, 'Movement requests cannot accumulate');
  env.motion('forward').emit('pointerup');
  await env.flush();
  const zero = env.matching('/api/jog').at(-1);
  assert.deepEqual(vector(zero), [0, 0, 0, 0]);
  assert(Number(zero.fields.seq) > Number(old.fields.seq));
  old.resolve();
  await env.flush();
  assert.equal(env.motion('forward').classes.has('pressed'), false);
  const afterRelease = env.matching('/api/jog').length;
  await env.tick(100);
  assert.equal(env.matching('/api/jog').length, afterRelease, 'A late response cannot resume the old direction');
});

test('Ping refreshes ownership without a sequence or movement command', async () => {
  const env = harness();
  await env.manual();
  const count = env.matching('/api/jog').length;
  await env.tick(150);
  const ping = env.matching('/api/ping').at(-1);
  assert(ping.headers['X-Token']);
  assert.deepEqual(ping.fields, {});
  assert.equal(env.matching('/api/jog').length, count);
});

test('Background lifecycle sends one STOP, clears ownership and never auto-resumes', async () => {
  const env = harness();
  await env.manual();
  env.motion('forward').emit('pointerdown');
  await env.flush();
  env.document.hidden = true;
  env.document.emit('visibilitychange');
  env.window.emit('pagehide');
  env.window.emit('blur');
  await env.flush();
  assert.equal(env.matching('/api/stop').length, 1);
  assert.equal(env.elements.release.disabled, true);
  const jogCount = env.matching('/api/jog').length;
  const pingCount = env.matching('/api/ping').length;
  env.document.hidden = false;
  env.document.emit('visibilitychange');
  await env.flush();
  await env.tick(100);
  await env.tick(150);
  assert.equal(env.elements.manual.disabled, true);
  assert.equal(env.matching('/api/jog').length, jogCount);
  assert.equal(env.matching('/api/ping').length, pingCount);
});

test('The 500 ms JOG timeout stops the device and invalidates the token', async () => {
  const env = harness();
  await env.manual();
  env.server.deferJog = true;
  env.motion('right').emit('pointerdown');
  await env.flush();
  const timeout = Array.from(env.timeouts.values()).find(item => item.ms === 500);
  assert(timeout, 'The pending request must have a 500 ms deadline');
  timeout.fn();
  await env.flush();
  assert.equal(env.elements.release.disabled, true);
  assert.equal(env.motion('right').disabled, true);
  assert.equal(env.requests.at(-1).path, '/api/stop');
});

test('A status connection failure stops control', async () => {
  const env = harness();
  await env.manual();
  env.server.failStatus = true;
  await env.tick(300);
  assert.equal(env.elements.release.disabled, true);
  assert.equal(env.motion('forward').disabled, true);
  assert.equal(env.elements.gimbalApply.disabled, true);
  assert.equal(env.requests.at(-1).path, '/api/stop');
});

test('A delayed unauthenticated status during claim cannot revoke the new token', async () => {
  const env = harness();
  await env.flush();
  env.server.deferClaim = true;
  env.elements.claim.emit('click');
  await env.flush();
  const claim = env.matching('/api/claim').at(-1);
  env.server.deferStatus = true;
  await env.tick(300);
  const oldStatus = env.matching('/api/status').at(-1);
  assert.equal(oldStatus.headers['X-Token'], undefined);
  claim.resolve();
  await env.flush();
  oldStatus.resolve();
  await env.flush();
  assert.equal(env.matching('/api/stop').length, 0);
  assert.equal(env.elements.release.disabled, false);
  await env.tick(300);
  assert.equal(env.elements.manual.disabled, false);
});

test('A failed old JOG cannot erase a newly acquired control session', async () => {
  const env = harness();
  await env.manual();
  env.server.deferJog = env.server.ignoreJogAbort = true;
  env.motion('forward').emit('pointerdown');
  await env.flush();
  const old = env.matching('/api/jog').at(-1);
  env.elements.stop.emit('click');
  await env.flush();
  await env.manual();
  const stopCount = env.matching('/api/stop').length;
  old.reject(new Error('Late failure from an old request'));
  await env.flush();
  assert.equal(env.matching('/api/stop').length, stopCount);
  assert.equal(env.elements.release.disabled, false);
  assert.equal(env.motion('forward').disabled, false);
});

test('STOP works without ownership; release immediately clears ownership', async () => {
  const env = harness();
  await env.flush();
  env.elements.stop.emit('click');
  await env.flush();
  assert.equal(env.requests.at(-1).path, '/api/stop');
  assert.equal(env.requests.at(-1).headers['X-Token'], undefined);
  await env.manual();
  env.elements.release.emit('click');
  assert.equal(env.elements.release.disabled, true);
  await env.flush();
  const release = env.matching('/api/release').at(-1);
  assert(release.headers['X-Token']);
  assert.equal(release.fields.seq, undefined);
  assert.equal(env.server.state.mode, 0);
});

test('Task selection sends six fields and waits for the exact STM32 ACK', async () => {
  const env = harness();
  await env.manual();
  env.server.state.autoSupported = true;
  env.elements.shape2.value='4'; env.elements.shape2.emit('input'); env.elements.color2.value='7';
  env.elements.shape3.value='3'; env.elements.shape3.emit('input'); env.elements.color3.value='3';
  env.elements.save.emit('click'); await env.flush();
  const save=env.matching('/api/task').at(-1);
  assert.deepEqual(['color1','shape1','color2','shape2','color3','shape3'].map(key=>save.fields[key]),['4','1','7','4','3','3']);
  assert.equal(env.matching('/api/start').length,0);
  assert(env.elements.notice.textContent.includes('等待主控确认'));
  env.elements.positioned.checked=true;env.elements.positioned.emit('change');
  assert.equal(env.elements.autoStart.disabled,true);
  Object.assign(env.server.state,{lastAckCommand:7,lastAckSequence:save.uartSeq+1,lastAckResult:0,ackValid:true});
  await env.tick(300);assert.equal(env.elements.autoStart.disabled,true);
  Object.assign(env.server.state,{lastAckSequence:save.uartSeq,taskAccepted:true,taskPlan:[4,1,7,4,3,3]});
  await env.tick(300);assert.equal(env.elements.autoStart.disabled,false);
  assert(env.elements.notice.textContent.includes('主控已接受抓取顺序'));
  assert(env.elements.taskStatus.textContent.includes('黄色五棱柱'));
  env.elements.color2.value='1';env.elements.color2.emit('input');assert.equal(env.elements.autoStart.disabled,true);
  const count=env.matching('/api/task').length;
  env.elements.shape3.value='2';env.elements.shape3.emit('input');env.elements.save.emit('click');await env.flush();
  assert.equal(env.matching('/api/task').length,count,'Unsupported shape must not be sent');
  env.elements.shape3.value='3';env.elements.color3.value='7';env.elements.save.emit('click');await env.flush();
  assert.equal(env.matching('/api/task').length,count,'Unsupported color/shape pair must not be sent');
});

test('Rejected, missing and unsupported task confirmation never permits start', async () => {
  const env=harness();await env.manual();env.server.state.autoSupported=true;
  env.elements.save.emit('click');await env.flush();const req=env.matching('/api/task').at(-1);
  Object.assign(env.server.state,{lastAckCommand:7,lastAckSequence:req.uartSeq,lastAckResult:1,ackValid:true});
  await env.tick(300);assert.equal(env.elements.autoStart.disabled,true);assert(env.elements.notice.textContent.includes('主控拒绝'));
  const timeout=harness();await timeout.manual();timeout.elements.save.emit('click');await timeout.flush();
  for(let i=0;i<5;++i) await timeout.tick(300);assert(timeout.elements.notice.textContent.includes('没有收到主控确认'));assert.equal(timeout.elements.autoStart.disabled,true);
  const old=harness();old.server.state.taskSupported=false;await old.manual();assert.equal(old.elements.save.disabled,true);assert.equal(old.elements.autoStart.disabled,true);
});

test('Automatic route requires setup confirmation, locks manual controls and can be cancelled', async () => {
  const env=harness();
  env.server.state.autoSupported=true; env.server.state.taskAccepted=true;env.server.state.taskPlan=[4,1,7,4,3,3];
  env.server.state.autoXcm=30; env.server.state.autoYcm=30;
  await env.manual();
  assert.equal(env.elements.autoStart.disabled,true);
  env.elements.positioned.checked=true; env.elements.positioned.emit('change');
  assert.equal(env.elements.autoStart.disabled,false);
  env.elements.autoStart.emit('click'); await env.flush();
  assert.equal(env.matching('/api/start').length,1);
  assert.equal(env.elements.positioned.checked,false);
  assert.equal(env.elements.mode.textContent,'自动搬运');
  assert.equal(env.motion('forward').disabled,true);
  assert.equal(env.elements.clawOpen.disabled,true);
  assert.equal(env.elements.gimbalApply.disabled,true);
  env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length,0);
  assert.equal(env.elements.autoCancel.disabled,false);
  assert.equal(env.elements.autoStart.disabled,true);
  env.elements.autoCancel.emit('click'); await env.flush();
  assert.equal(env.matching('/api/pause').length,1);
  assert(env.elements.autoStatus.textContent.includes('已停止'));
  assert.equal(env.elements.autoStart.disabled,true);
});

test('Gimbal requires manual ownership and the firmware capability', async () => {
  const env = harness();
  await env.flush();
  assert.equal(env.elements.gimbalApply.disabled, true);
  env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, 0);
  await env.manual();
  assert.equal(env.elements.gimbalApply.disabled, false);
  assert(env.elements.gimbalState.textContent.includes('目标角度未知'));
  delete env.server.state.gimbalSupported;
  await env.tick(300);
  assert.equal(env.elements.gimbalApply.disabled, true);
  assert(env.elements.gimbalState.textContent.includes('需更新主控固件'));
  env.elements.gimbalRetract.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, 0);
  env.server.state.gimbalSupported = true;
  env.server.state.gimbalAngle = 145;
  await env.tick(300);
  assert.equal(env.elements.gimbalAngle.value, '145');
  assert(env.elements.gimbalState.textContent.includes('目标角度 145°'));
  env.server.state.gimbalAngle = 271;
  await env.tick(300);
  assert(env.elements.gimbalState.textContent.includes('目标角度未知'), 'An invalid firmware target is not displayed as a valid angle');
});

test('Gimbal sends authenticated selected angles and presets without fabricating measured position', async () => {
  const env = harness();
  env.server.state.gimbalAngle = 54;
  await env.manual();
  env.elements.gimbalAngle.value = '145';
  env.elements.gimbalAngle.emit('input');
  assert.equal(env.elements.gimbalValue.textContent, '145°');
  assert.equal(env.matching('/api/gimbal').length, 0, 'Dragging the slider must not send motion');
  env.elements.gimbalApply.emit('click'); await env.flush();
  const selected = env.matching('/api/gimbal').at(-1);
  assert.equal(selected.fields.angle, '145');
  assert(selected.headers['X-Token']);
  assert(Number(selected.fields.seq) > 0);
  assert(env.elements.gimbalState.textContent.includes('目标角度 54°'), 'HTTP queuing must not invent a confirmed target');
  env.server.state.gimbalAngle = 145; env.server.state.pending = false;
  await env.acknowledge(selected);
  assert(env.elements.gimbalState.textContent.includes('目标角度 145°'));
  for (const [id, angle] of [['gimbalRetract', 54], ['gimbalExtend', 170]]) {
    env.elements[id].emit('click'); await env.flush();
    const request = env.matching('/api/gimbal').at(-1);
    assert.equal(request.fields.angle, String(angle));
    assert(Number(request.fields.seq) > Number(selected.fields.seq));
    assert.equal(env.elements.gimbalAngle.value, String(angle));
    await env.acknowledge(request);
  }
  const count = env.matching('/api/gimbal').length;
  for (const value of ['', '-1', '271', '54.5', 'abc']) {
    env.elements.gimbalAngle.value = value;
    env.elements.gimbalApply.emit('click'); await env.flush();
  }
  assert.equal(env.matching('/api/gimbal').length, count, 'Invalid and out-of-range angles must not be transmitted');
});

test('Gimbal does not overlap held motion, pending zero release or another command', async () => {
  const env = harness();
  await env.manual();
  env.motion('forward').emit('pointerdown'); await env.flush();
  assert.equal(env.elements.gimbalExtend.disabled, true);
  env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, 0);
  env.server.deferJog = true;
  env.motion('forward').emit('pointerup'); await env.flush();
  const zero = env.matching('/api/jog').at(-1);
  assert.equal(env.elements.gimbalExtend.disabled, true);
  env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, 0);
  zero.resolve(); await env.flush();
  assert.equal(env.elements.gimbalExtend.disabled, false);
  env.server.deferGimbal = true;
  env.elements.gimbalExtend.emit('click'); await env.flush();
  const request = env.matching('/api/gimbal').at(-1);
  assert.equal(env.elements.gimbalRetract.disabled, true);
  assert.equal(env.motion('forward').disabled, true);
  env.elements.gimbalRetract.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, 1);
  request.resolve(); await env.flush();
  env.elements.release.emit('click'); await env.flush();
  assert.equal(env.elements.gimbalApply.disabled, true);
});

test('Servo commands wait for matching UART ACK and never treat HTTP queuing as motion', async () => {
  const env = harness();
  await env.manual();
  env.elements.clawOpen.emit('click'); await env.flush();
  const request = env.matching('/api/claw').at(-1);
  assert.equal(env.elements.clawClose.disabled, true);
  assert.equal(env.elements.gimbalApply.disabled, true);
  assert.equal(env.motion('forward').disabled, true);
  assert(env.elements.clawState.textContent.includes('目标未知'));
  env.server.state.lastAckSequence = request.uartSeq;
  env.server.state.lastAckCommand = 5;
  env.server.state.ackValid = true;
  await env.tick(300);
  assert.equal(env.elements.clawClose.disabled, true, 'ACK from a different UART command must not confirm the claw');
  await env.acknowledge(request);
  assert.equal(env.elements.clawClose.disabled, false);
  assert(env.elements.notice.textContent.includes('主控已接受夹爪张开目标'));
  assert(env.elements.notice.textContent.includes('没有舵机位置反馈'));
});

test('Servo actions wait for actual motor feedback to become zero', async () => {
  const env = harness();
  await env.manual();
  env.server.state.y = 20;
  await env.tick(300);
  assert.equal(env.elements.clawOpen.disabled, true);
  assert.equal(env.elements.gimbalExtend.disabled, true);
  env.elements.clawOpen.emit('click'); env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/claw').length + env.matching('/api/gimbal').length, 0);
  env.server.state.y = 0;
  await env.tick(300);
  assert.equal(env.elements.clawOpen.disabled, false);
  assert.equal(env.elements.gimbalExtend.disabled, false);
});

test('Rejected servo ACK and missing ACK give actionable errors and revoke control', async () => {
  const rejected = harness();
  await rejected.manual();
  rejected.elements.gimbalExtend.emit('click'); await rejected.flush();
  await rejected.acknowledge(rejected.matching('/api/gimbal').at(-1), 5);
  assert(rejected.elements.notice.textContent.includes('运动互锁或设备状态不满足要求'));
  assert.equal(rejected.matching('/api/stop').length, 1);
  const timed = harness();
  await timed.manual();
  timed.elements.clawClose.emit('click'); await timed.flush();
  for (let i = 0; i < 5; ++i) await timed.tick(300);
  assert(timed.elements.notice.textContent.includes('夹爪指令没有收到主控确认'));
  assert.equal(timed.matching('/api/stop').length, 1);
  assert.equal(timed.elements.clawOpen.disabled, true);
});

test('Visible servo hints explain disabled ownership, manual mode, motion and ACK conditions', async () => {
  const env = harness();
  await env.flush();
  assert.equal(env.elements.clawHint.textContent, '请先申请控制。');
  assert.equal(env.elements.gimbalHint.textContent, '请先申请控制。');
  env.elements.claim.emit('click'); await env.flush(); await env.tick(300);
  assert.equal(env.elements.clawHint.textContent, '请先启用手动。');
  env.elements.manual.emit('click'); await env.flush();
  assert.equal(env.elements.clawHint.textContent, '等待手动模式确认。');
  env.server.state.mode = 1; env.server.state.armed = true; env.server.state.pending = false;
  await env.tick(300);
  assert.equal(env.elements.clawHint.textContent, '可以操作夹爪。');
  env.motion('forward').emit('pointerdown'); await env.flush();
  assert.equal(env.elements.clawHint.textContent, '请先松开方向按钮。');
  assert.equal(env.elements.gimbalHint.textContent, '请先松开方向按钮。');
  env.server.deferJog = true;
  env.motion('forward').emit('pointerup'); await env.flush();
  assert.equal(env.elements.clawHint.textContent, '等待小车停止。');
  env.matching('/api/jog').at(-1).resolve(); await env.flush();
  env.server.state.y = 20; await env.tick(300);
  assert.equal(env.elements.gimbalHint.textContent, '等待小车停止。');
  env.server.state.y = 0; await env.tick(300);
  env.elements.clawOpen.emit('click'); await env.flush();
  assert.equal(env.elements.clawHint.textContent, '等待夹爪指令确认。');
  assert.equal(env.elements.gimbalHint.textContent, '等待夹爪指令确认。');
  await env.acknowledge(env.matching('/api/claw').at(-1));
  assert.equal(env.elements.clawHint.textContent, '可以操作夹爪。');
  env.server.state.gimbalSupported = false; await env.tick(300);
  assert.equal(env.elements.gimbalHint.textContent, '主控暂不支持云台，请更新固件。');
  assert.equal(env.elements.clawHint.textContent, '可以操作夹爪。');
  assert.equal(env.elements.gimbalApply.disabled, true);
  assert.equal(env.elements.clawOpen.disabled, false);
  env.server.state.mode = 2; env.server.state.armed = false; await env.tick(300);
  assert.equal(env.elements.clawHint.textContent, '自动搬运中，暂不可手动操作。');
  env.server.state.link = false; await env.tick(300);
  assert.equal(env.elements.clawHint.textContent, '等待主控连接。');
});

test('Full 270-degree range requires new firmware; old firmware keeps its established bounds', async () => {
  const env = harness();
  await env.manual();
  for (const angle of [0, 270]) {
    env.elements.gimbalAngle.value = String(angle);
    env.elements.gimbalApply.emit('click'); await env.flush();
    const request = env.matching('/api/gimbal').at(-1);
    assert.equal(request.fields.angle, String(angle));
    await env.acknowledge(request);
  }
  env.server.state.servoPresetsSupported = false;
  await env.tick(300);
  assert.equal(env.elements.gimbalAngle.min, '54');
  assert.equal(env.elements.gimbalAngle.max, '229');
  assert.equal(env.elements.clawApply.disabled, true);
  assert.equal(env.elements.gimbalSetRetract.disabled, true);
  const count = env.matching('/api/gimbal').length;
  env.elements.gimbalAngle.value = '0'; env.elements.gimbalApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').length, count);
  env.elements.gimbalExtend.emit('click'); await env.flush();
  assert.equal(env.matching('/api/gimbal').at(-1).fields.angle, '229');
});

test('Claw angle calibration waits for ACK and preserves untested open/close presets', async () => {
  const env = harness();
  await env.manual();
  assert.equal(env.elements.clawClose.textContent, '闭合目标 · 10°');
  assert.equal(env.elements.clawOpen.textContent, '张开目标 · 70°');
  env.elements.clawAngle.value = '125'; env.elements.clawAngle.emit('input');
  assert.equal(env.matching('/api/claw-angle').length, 0);
  env.elements.clawApply.emit('click'); await env.flush();
  const request = env.matching('/api/claw-angle').at(-1);
  assert.equal(request.fields.angle, '125');
  assert.equal(env.elements.clawSetOpen.disabled, true);
  assert.equal(env.motion('liftUp').disabled, true);
  assert(!env.elements.clawState.textContent.includes('125°'));
  await env.acknowledge(request);
  assert(env.elements.clawState.textContent.includes('125°'));
  assert(env.elements.notice.textContent.includes('主控已接受夹爪目标角度 125°'));
  assert.equal(env.elements.clawOpen.textContent, '张开目标 · 70°', 'An angle trial must not silently change an automatic action preset');
  const count = env.matching('/api/claw-angle').length;
  for (const value of ['', '-1', '271', '1.5']) {
    env.elements.clawAngle.value = value; env.elements.clawApply.emit('click'); await env.flush();
  }
  assert.equal(env.matching('/api/claw-angle').length, count);
});

test('Saving retract/open targets changes confirmed presets without moving a servo', async () => {
  const env = harness();
  await env.manual();
  env.elements.gimbalAngle.value = '230'; env.elements.gimbalSetRetract.emit('click'); await env.flush();
  const retract = env.matching('/api/servo-presets').at(-1);
  assert.deepEqual(retract.fields, {seq: retract.fields.seq, clawClose:'10', clawOpen:'70', gimbalIn:'230', gimbalOut:'170'});
  assert.equal(env.elements.gimbalRetract.textContent, '收回 · 54°', 'HTTP queuing is not a confirmed preset update');
  assert.equal(env.matching('/api/gimbal').length, 0);
  await env.acknowledge(retract);
  assert.equal(env.elements.gimbalRetract.textContent, '收回 · 230°');
  assert(env.elements.presetHint.textContent.includes('预设已保存在 ESP32'));
  env.elements.gimbalRetract.emit('click'); await env.flush();
  const move = env.matching('/api/gimbal').at(-1);
  assert.equal(move.fields.angle, '230');
  await env.acknowledge(move);
  env.elements.clawAngle.value = '150'; env.elements.clawSetOpen.emit('click'); await env.flush();
  const open = env.matching('/api/servo-presets').at(-1);
  assert.equal(open.fields.clawOpen, '150'); assert.equal(open.fields.gimbalIn, '230');
  assert.equal(env.matching('/api/claw-angle').length, 0);
  await env.acknowledge(open);
  assert.equal(env.elements.clawOpen.textContent, '张开目标 · 150°');
});

test('Tray is independently gated by ownership, manual mode and the new firmware capability', async () => {
  const env = harness();
  await env.flush();
  assert.equal(env.elements.trayApply.disabled, true);
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  await env.manual();
  assert.equal(env.elements.trayApply.disabled, false);
  assert(env.elements.trayState.textContent.includes('目标角度未知'));
  delete env.server.state.traySupported;
  await env.tick(300);
  assert.equal(env.elements.trayApply.disabled, true);
  assert(env.elements.trayHint.textContent.includes('请同时更新'));
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  env.server.state.traySupported = true;
  env.server.state.trayAngle = 145;
  await env.tick(300);
  assert.equal(env.elements.trayAngle.value, '145');
  assert(env.elements.trayState.textContent.includes('目标角度 145°'));
  env.server.state.trayAngle = 271; await env.tick(300);
  assert(env.elements.trayState.textContent.includes('目标角度未知'));
  env.server.state.mode = 2; env.server.state.armed = false; await env.tick(300);
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.elements.trayApply.disabled, true);
  assert.equal(env.matching('/api/tray-angle').length, 0);
});

test('Tray selection does not move a servo; bounds and matching ACK are enforced', async () => {
  const env = harness();
  env.server.state.trayAngle = 120;
  await env.manual();
  env.elements.trayAngle.value = '1'; env.elements.trayAngle.emit('input');
  env.elements.trayMinus.emit('click');
  assert.equal(env.elements.trayAngle.value, '0');
  env.elements.trayPlus.emit('click');
  assert.equal(env.elements.trayValue.textContent, '5°');
  assert.equal(env.matching('/api/tray-angle').length, 0);
  env.elements.trayAngle.value = '269'; env.elements.trayPlus.emit('click');
  assert.equal(env.elements.trayAngle.value, '270');
  env.elements.trayApply.emit('click'); await env.flush();
  const request = env.matching('/api/tray-angle').at(-1);
  assert.equal(request.fields.angle, '270');
  assert(request.headers['X-Token']);
  assert(env.elements.trayState.textContent.includes('目标角度 120°'));
  assert.equal(env.elements.clawOpen.disabled, true);
  assert.equal(env.elements.gimbalApply.disabled, true);
  assert.equal(env.motion('forward').disabled, true);
  env.server.state.ackValid = true; env.server.state.lastAckSequence = request.uartSeq;
  env.server.state.lastAckCommand = 10; await env.tick(300);
  assert.equal(env.elements.trayApply.disabled, true, 'ACK for another actuator cannot release the pending command');
  await env.acknowledge(request);
  assert(env.elements.notice.textContent.includes('主控已接受转盘目标角度 270°'));
  assert(env.elements.trayState.textContent.includes('目标角度 270°'));
  const count = env.matching('/api/tray-angle').length;
  for (const value of ['', '-1', '271', '1.5', 'NaN']) {
    env.elements.trayAngle.value = value; env.elements.trayApply.emit('click'); await env.flush();
  }
  assert.equal(env.matching('/api/tray-angle').length, count);
  env.elements.trayAngle.value = '0'; env.elements.trayApply.emit('click'); await env.flush();
  await env.acknowledge(env.matching('/api/tray-angle').at(-1));
  assert(env.elements.trayState.textContent.includes('目标角度 0°'));
});

test('Tray does not overlap a held drive, pending release or another actuator', async () => {
  const env = harness(); await env.manual();
  env.motion('forward').emit('pointerdown'); await env.flush();
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  env.server.deferJog = true;
  env.motion('forward').emit('pointerup'); await env.flush();
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  env.matching('/api/jog').at(-1).resolve(); await env.flush();
  env.server.state.lift = 1; await env.tick(300);
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  env.server.state.lift = 0; await env.tick(300);
  env.elements.clawOpen.emit('click'); await env.flush();
  env.elements.trayApply.emit('click'); await env.flush();
  assert.equal(env.matching('/api/tray-angle').length, 0);
  await env.acknowledge(env.matching('/api/claw').at(-1));
  env.elements.trayApply.emit('click'); await env.flush();
  const request = env.matching('/api/tray-angle').at(-1);
  env.elements.stop.emit('click'); await env.flush();
  await env.acknowledge(request);
  assert.equal(env.elements.trayApply.disabled, true, 'A late ACK must not restore ownership after STOP');
});

test('Tray rejected or missing ACK reports a fault and stops control', async () => {
  const rejected = harness(); await rejected.manual();
  rejected.elements.trayApply.emit('click'); await rejected.flush();
  await rejected.acknowledge(rejected.matching('/api/tray-angle').at(-1), 5);
  assert(rejected.elements.notice.textContent.includes('转盘指令被主控拒绝'));
  assert.equal(rejected.elements.release.disabled, true);
  const missing = harness(); await missing.manual();
  missing.elements.trayApply.emit('click'); await missing.flush();
  for (let i = 0; i < 5; i++) await missing.tick(300);
  assert(missing.elements.notice.textContent.includes('转盘指令没有收到主控确认'));
  assert.equal(missing.elements.release.disabled, true);
});

test('Manual lift faults revoke ownership with a concrete diagnostic; no held motion resumes', async () => {
  for (const [fault, word] of [[8, '反馈'], [9, '进展'], [10, '时限']]) {
    const env = harness(); env.server.state.liftEnabled = true;
    await env.manual(); env.motion('liftUp').emit('pointerdown'); await env.flush();
    env.server.state.fault = fault; await env.tick(300);
    assert(env.elements.notice.textContent.includes(word));
    assert.equal(env.matching('/api/stop').length, 1);
    assert.equal(env.motion('liftUp').classes.has('pressed'), false);
    const count = env.matching('/api/jog').length;
    await env.tick(100);
    assert.equal(env.matching('/api/jog').length, count);
  }
});

test('A fault cleared by STOP remains visible and explains lost ownership without blocking a new session', async () => {
  const env=harness(); await env.manual();
  Object.assign(env.server.state,{fault:0,lastFault:9,lastFaultAgeMs:100,claimed:false,mine:false,armed:false,mode:0});
  await env.tick(300);
  assert(env.elements.notice.textContent.includes('没有进展'));
  assert(env.elements.fault.textContent.includes('最近：'));
  await env.manual();
  assert.equal(env.motion('forward').disabled,false,'A historical fault is not an active fault');
});

(async () => {
  for (const item of tests) {
    await item.run();
    console.log(`PASS ${item.name}`);
  }
  console.log(`\n${tests.length} browser simulation tests passed; JavaScript syntax and external-resource checks passed.`);
  console.log('Physical ESP32/STM32 hardware was not contacted.');
})().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
