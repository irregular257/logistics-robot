'use strict';

// Local interface preview. All /api requests use in-memory simulated state.
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const sourcePath = path.join(__dirname, '../robot_WebControl/web_page.h');
const port = Number(process.env.ROBOT_PREVIEW_PORT || 8765);
let token = '';
let uartSequence = 0;
let liftLatched = false, liftTimer;
let state = {
  stmOnline: true, ownerActive: false, armed: false, mode: 0, fault: 0,
  claw: 0, clawAngle: -1, liftEnabled: true, gimbalSupported: true, gimbalAngle: -1, pending: false, ackValid: false,
  traySupported: true, trayAngle: -1,
  servoPresetsSupported: true, presetStorageReady: true, presetsSaved: false, presetsPending: false,
  presets: {clawClose: 10, clawOpen: 70, gimbalIn: 54, gimbalOut: 170},
  lastAckSeq: 0, lastAckCmd: 0, lastAckResult: 0, taskSupported: true, taskAccepted: false, taskPending: false, taskPlan: [0,0,0,0,0,0], autoSupported: true, autoState: 0, autoDrop: 0,
  autoError: 0, autoXcm: 30, autoYcm: 30, x: 0, y: 0, turn: 0, lift: 0
};

function version() { return fs.statSync(sourcePath).mtimeMs.toString(); }
function page() {
  const source = fs.readFileSync(sourcePath, 'utf8');
  const match = source.match(/R"HTML\(([\s\S]*?)\)HTML";/);
  if (!match) throw new Error('Cannot extract WEB_PAGE from web_page.h');
  const banner = '<div style="padding:10px 16px;text-align:center;background:#f3cc69;color:#292018;font:14px system-ui">工作区预览 · 设备状态为模拟数据 · 按钮不会控制实车</div>';
  const reload = `<script>(()=>{let v=${JSON.stringify(version())};setInterval(async()=>{try{const r=await fetch('/preview/version',{cache:'no-store'});if(r.ok&&await r.text()!==v)location.reload();}catch(_){}},2000);})();</script>`;
  const html = match[1].replace('<body>', '<body>' + banner).replace('</body>', reload + '</body>');
  fs.writeFileSync(path.join(__dirname, 'index.html'), html, 'utf8');
  return html;
}
function stop() {
  Object.assign(state, { armed: false, mode: 0, ownerActive: false, x: 0, y: 0, turn: 0, lift: 0 });
  if (state.autoState > 0 && state.autoState < 13) state.autoState = 14;
  token = '';
  clearTimeout(liftTimer); liftLatched = false;
}
const server = http.createServer(async (req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  const route = new URL(req.url, 'http://localhost').pathname;
  const reply = (data, code = 200) => {
    res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8' });
    res.end(JSON.stringify(data));
  };
  try {
    if (route === '/' || route === '/index.html') {
      res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      return res.end(page());
    }
    if (route === '/preview/version') {
      res.writeHead(200, { 'Content-Type': 'text/plain' });
      return res.end(version());
    }
    if (route === '/api/status') return req.method === 'GET' ? reply({ ...state, mine: !!token && req.headers['x-token'] === token }) : reply({error:'请使用 GET 请求'},405);
    if (route.startsWith('/api/') && req.method !== 'POST') return reply({ error: '请使用 POST 请求' }, 405);
    if (route === '/api/stop') { stop(); return reply({ ok: true }); }
    if (route === '/api/claim') {
      if (token) return reply({ error: '已有预览页面持有控制权' }, 409);
      token = crypto.randomBytes(8).toString('hex');
      state.ownerActive = true;
      return reply({ ok: true, token });
    }
    if (!route.startsWith('/api/')) return reply({ error: 'Not found' }, 404);
    if (!token || req.headers['x-token'] !== token) return reply({ error: '请先申请控制' }, 403);
    let body = '';
    for await (const chunk of req) {
      body += chunk;
      if (body.length > 4096) return reply({ error: 'Request too large' }, 413);
    }
    const fields = new URLSearchParams(body);
    let command = 0;
    const taskFields = ['color1','shape1','color2','shape2','color3','shape3'];
    const shapeColors = {1:[4],3:[1,2,3,4],4:[1,2,7],5:[1,2,3,4,7],6:[1,2,3,4,7]};
    const readPlan = () => {
      const plan = taskFields.map(key => /^[1-7]$/.test(fields.get(key) || '') ? Number(fields.get(key)) : NaN);
      return plan.every((value,i) => Number.isInteger(value) && (i%2 ? shapeColors[value] && shapeColors[value].includes(plan[i-1]) : [1,2,3,4,7].includes(value))) ? plan : null;
    };
    const taskIdle = () => state.stmOnline && !state.fault && state.mode !== 2 && !state.x && !state.y && !state.turn && !state.lift;
    switch (route) {
      case '/api/ping': break;
      case '/api/release': stop(); break;
      case '/api/mode': state.mode = 1; state.armed = true; command = 4; break;
      case '/api/jog': {
        if (state.mode !== 1 || !state.armed || !state.stmOnline || state.fault) return reply({ error: '请先启用手动并确认主控状态' }, 409);
        const vector = ['x', 'y', 'turn', 'lift'].map(key => Number(fields.get(key)));
        if (vector.some((value, index) => !fields.has(['x', 'y', 'turn', 'lift'][index]) || !Number.isInteger(value) || Math.abs(value) > (index === 3 ? 1 : 50))) return reply({error:'运动参数无效'},400);
        if (vector[3] && (vector[0] || vector[1] || vector[2])) return reply({error:'底盘和升降不能同时运动'},400);
        if (vector[3] && !state.liftEnabled) return reply({error:'等待轴 5 驱动器反馈'},409);
        [state.x, state.y, state.turn] = vector;
        if (!vector[3]) { state.lift = 0; liftLatched = false; clearTimeout(liftTimer); }
        else if (!liftLatched) {
          liftLatched = true; state.lift = vector[3];
          // Simulate one finite jog. Repeated held JOG cannot launch another move without a zero release.
          liftTimer = setTimeout(() => { state.lift = 0; }, 200);
        }
        command = 5;
        break;
      }
      case '/api/claw': {
        if (req.method !== 'POST') return reply({ error: '请使用 POST 请求' }, 405);
        if (!state.stmOnline || state.mode !== 1 || !state.armed || state.fault) return reply({ error: '请先启用手动' }, 409);
        if (state.x || state.y || state.turn || state.lift) return reply({ error: '请先停止底盘和升降运动' }, 409);
        const value = fields.get('value');
        if (value !== '0' && value !== '1') return reply({ error: '夹爪参数无效' }, 400);
        state.claw = Number(value); state.clawAngle = value === '0' ? state.presets.clawClose : state.presets.clawOpen; command = 6;
        break;
      }
      case '/api/gimbal': {
        if (req.method !== 'POST') return reply({ error: '请使用 POST 请求' }, 405);
        if (!state.stmOnline || state.mode !== 1 || !state.armed || state.fault || !state.gimbalSupported) return reply({ error: '请先启用手动并确认主控支持云台' }, 409);
        if (state.x || state.y || state.turn || state.lift) return reply({ error: '请先停止底盘和升降运动' }, 409);
        const text = fields.get('angle') || '';
        const angle = Number(text);
        if (!/^\d+$/.test(text) || !Number.isInteger(angle) || angle < 0 || angle > 270) return reply({ error: '云台角度须为 0° 至 270° 的整数' }, 400);
        state.gimbalAngle = angle;
        command = 10;
        break;
      }
      case '/api/claw-angle': {
        if (!state.stmOnline || state.mode !== 1 || !state.armed || state.fault || !state.servoPresetsSupported) return reply({error:'请先启用手动并确认主控支持角度试调'},409);
        if (state.x || state.y || state.turn || state.lift) return reply({error:'请先停止底盘和升降运动'},409);
        const text = fields.get('angle') || '', angle = Number(text);
        if (!/^\d+$/.test(text) || !Number.isInteger(angle) || angle < 0 || angle > 270) return reply({error:'夹爪角度须为 0° 至 270° 的整数'},400);
        state.clawAngle = angle; state.claw = 2; command = 11;
        break;
      }
      case '/api/tray-angle': {
        if (!state.stmOnline || state.mode !== 1 || !state.armed || state.fault || !state.traySupported) return reply({error:'请先启用手动并确认主控支持转盘'},409);
        if (state.x || state.y || state.turn || state.lift) return reply({error:'请先停止底盘和升降运动'},409);
        const text = fields.get('angle') || '', angle = Number(text);
        if (!/^\d+$/.test(text) || !Number.isInteger(angle) || angle < 0 || angle > 270) return reply({error:'转盘角度须为 0° 至 270° 的整数'},400);
        state.trayAngle = angle; command = 13;
        break;
      }
      case '/api/servo-presets': {
        if (!state.stmOnline || state.mode === 2 || state.fault || !state.servoPresetsSupported) return reply({error:'请等待主控待机或手动状态'},409);
        if (state.x || state.y || state.turn || state.lift) return reply({error:'请先停止底盘和升降运动'},409);
        const next = {};
        for (const key of ['clawClose', 'clawOpen', 'gimbalIn', 'gimbalOut']) {
          const text = fields.get(key) || '', value = Number(text);
          if (!/^\d+$/.test(text) || !Number.isInteger(value) || value < 0 || value > 270) return reply({error:'预设角度须为 0° 至 270° 的整数'},400);
          next[key] = value;
        }
        state.presets = next; state.presetsSaved = true; command = 12;
        break;
      }
      case '/api/task': {
        if (!taskIdle()) return reply({error:'请先停止运动后发送任务'},409);
        const plan = readPlan();
        if (!plan) return reply({error:'请为三件物料选择有效的颜色和形状'},400);
        state.taskPlan = plan; state.taskAccepted = true; command = 7;
        break;
      }
      case '/api/start': {
        if (!taskIdle()) return reply({error:'请先停止运动后启动'},409);
        const plan = readPlan();
        if (fields.get('positioned') !== '1' || !plan) return reply({error:'请确认起点及三件物料参数'},400);
        if (!state.taskAccepted || !plan.every((value,i) => value === state.taskPlan[i])) return reply({error:'请先发送当前抓取顺序并等待主控确认'},409);
        state.mode = 2; state.armed = false; state.autoState = 1; state.gimbalAngle = -1; state.trayAngle = -1; command = 8;
        break;
      }
      case '/api/pause': state.mode = 0; state.armed = false; state.autoState = 14; command = 9; break;
      default: return reply({ error: 'Not found' }, 404);
    }
    if (command) {
      uartSequence = (uartSequence + 1) & 0xFFFF;
      state.lastAckSeq = uartSequence; state.lastAckCmd = command;
      state.lastAckResult = 0; state.ackValid = true;
      return reply({ ok: true, queued: true, uartSeq: uartSequence }, 202);
    }
    reply({ ok: true });
  } catch (error) { reply({ error: error.message }, 500); }
});
page();
server.listen(port, '127.0.0.1', () => console.log(`robot workspace preview: http://127.0.0.1:${port}/`));
server.on('error', error => { console.error(error.message); process.exitCode = 1; });
