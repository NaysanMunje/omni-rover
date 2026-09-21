#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// Three motors, same motion pattern as gem_test / l982n_test:
// forward 2s, stop 1s, reverse 2s, stop 1s. Serial 115200.
// Connects to home WiFi. Serial prints IP. Also try http://omnirover.local
//
// Wiring (ESP32-S3). Motor windings = red / white. Encoder = blue 3.3V, black GND,
// yellow = A, green = B. Common GND: 12V-, ESP32, TB6612, L298N.
//
// --- TB6612 (motors 1 and 2) ---
// VM -> 12V+    VCC -> ESP32 3.3V    GND -> common GND
// STBY -> GPIO3 (driven HIGH in setup, same as gem_test)
// Direction pairs sit on neighboring header pins (right row: 42-41, 40-39, 47-21).
// Motor 1 (channel A):
//   red -> AO1    white -> AO2
//   PWMA -> GPIO48    AIN1 -> GPIO41    AIN2 -> GPIO42
//   yellow -> GPIO1    green -> GPIO2
// Motor 2 (channel B):
//   red -> BO1    white -> BO2
//   PWMB -> GPIO18    BIN1 -> GPIO40    BIN2 -> GPIO39
//   yellow -> GPIO38   green -> GPIO37
//
// --- L298N (motor 3; l982n_test logic, pins remapped for this board) ---
// 12V+ -> L298N +12V/VMS     GND -> common GND
// Remove the ENA jumper so GPIO PWM can control speed.
// Logic 5V: onboard 5V jumper from 12V, or ESP32 5V to L298N 5V.
// Motor 3:
//   red -> OUT1    white -> OUT2
//   ENA -> GPIO15    IN1 -> GPIO47    IN2 -> GPIO21
//   yellow -> GPIO13   green -> GPIO14
// Unused on purpose: PSRAM 35–36, USB 19–20, UART 43–44, BOOT 0, straps 45–46.
// GPIO37 is octal PSRAM — do not enable OPI PSRAM while this encoder is wired.

#include "secrets.h"

// Motor 1 TB6612 A (gem_test)
const int PIN_PWMA = 48;
const int PIN_AIN1 = 41;
const int PIN_AIN2 = 42;
const int PIN_STBY = 3;
const int PIN_ENC1_A = 1;
const int PIN_ENC1_B = 2;

// Motor 2 TB6612 B
const int PIN_PWMB = 18;
const int PIN_BIN1 = 40;
const int PIN_BIN2 = 39;
const int PIN_ENC2_A = 38;
const int PIN_ENC2_B = 37;

// Motor 3 L298N (l982n_test roles: ENA speed, IN1/IN2 direction)
const int PIN_ENA = 15;
const int PIN_IN1 = 47;
const int PIN_IN2 = 21;
const int PIN_ENC3_A = 13;
const int PIN_ENC3_B = 14;

const int PWM_FREQ = 5000;
const int PWM_RES = 8;

volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0;
volatile long encoderTicks3 = 0;

WebServer server(80);
bool testRunning = false;
bool kiwiFb = false;
enum TestPhase { PHASE_FWD, PHASE_PAUSE_A, PHASE_REV, PHASE_PAUSE_B };
TestPhase testPhase = PHASE_FWD;
unsigned long phaseStartMs = 0;
unsigned long lastTickPrintMs = 0;
int speed1 = 180;
int speed2 = 180;
int speed3 = 180;
bool invert1 = true;
bool invert2 = true;
bool invert3 = false;
int strafeMix = 0;
const int TB_OFFSET = 80;
const unsigned long TB_RAMP_MS = 350;
bool tbRamping = false;
unsigned long tbRampStartMs = 0;
int tbRampFrom1 = 0;
int tbRampFrom2 = 0;
bool tbRampDir1 = true;
bool tbRampDir2 = true;
int lastTbOut1 = 0;
int lastTbOut2 = 0;
int lastTbOut3 = 0;
int tbRampFrom3 = 0;
bool tbRampDir3 = true;
bool ctrlActive = false;
unsigned long lastDriveMs = 0;

int tbPwm(int slider);
void startTbCoastRamp();
void updateTbCoastRamp();

void setMotor1(int speed, bool forward);
void setMotor2(int speed, bool forward);
void setMotor3(int speed, bool forward);
void driveKiwiFb(bool bodyForward);
void driveTbSigned(int motor, float signedCmd);
void applyBody(float vx, float vy, float wz);
void driveM3Signed(float signedCmd);
void IRAM_ATTR readEncoder1();
void IRAM_ATTR readEncoder2();
void IRAM_ATTR readEncoder3();

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Omni Rover</title>
  <style>
    body { font-family: sans-serif; text-align: center; margin: 48px 16px; background: #111; color: #eee; }
    button { font-size: 1.4rem; padding: 16px 40px; border: 0; border-radius: 8px; }
    #go { background: #2a8; color: #111; }
    #go.stop { background: #c44; color: #fff; }
    #kiwi { background: #48a; color: #fff; margin-left: 8px; }
    #kiwi.stop { background: #c44; color: #fff; }
    .row { margin-bottom: 16px; }
    label { display: block; margin: 20px auto; max-width: 320px; text-align: left; }
    input[type=range] { width: 100%; }
    .dir { font-size: 0.9rem; padding: 8px 14px; margin-top: 8px; background: #444; color: #eee; }
    .dir.on { background: #86f; color: #111; }
    .tabs { margin: 0 auto 20px; display: flex; flex-wrap: wrap; justify-content: center; gap: 8px; }
    .tab { font-size: 1rem; padding: 10px 16px; margin: 0; background: #333; color: #eee; touch-action: manipulation; }
    .tab.on { background: #2a8; color: #111; }
    .ico { width: 28px; height: 28px; display: block; margin: 0 auto; stroke: currentColor; fill: none; stroke-width: 2.2; stroke-linecap: round; stroke-linejoin: round; }
    .pad { display: grid; grid-template-columns: 72px 72px 72px; gap: 8px; justify-content: center; margin: 20px auto; user-select: none; -webkit-user-select: none; touch-action: none; }
    .pad button, .spin {
      font-size: 1.2rem; padding: 16px 0; background: #333; color: #eee;
      user-select: none; -webkit-user-select: none; -webkit-touch-callout: none;
      touch-action: none;
    }
    .pad button:active, .spin:active { background: #2a8; color: #111; }
    .spins { display: flex; gap: 12px; justify-content: center; user-select: none; -webkit-user-select: none; }
    .spin { padding: 14px 22px; min-width: 72px; }
    .mix { font-size: 0.95rem; padding: 12px 14px; margin: 8px; background: #333; color: #eee; max-width: 340px; }
    .mix.on { background: #2a8; color: #111; }
    .hint { font-size: 0.85rem; color: #aaa; max-width: 340px; margin: 8px auto; }
    .joyrow { display: flex; justify-content: space-around; align-items: center; gap: 12px; margin: 28px 8px; touch-action: none; }
    .stickwrap { touch-action: none; }
    .stickwrap p { margin: 8px 0 0; font-size: 0.85rem; color: #aaa; }
    .stick {
      width: 42vw; height: 42vw; max-width: 190px; max-height: 190px;
      min-width: 140px; min-height: 140px;
      border-radius: 50%; background: #1a1a1a; border: 3px solid #444;
      position: relative; touch-action: none; margin: 0 auto;
    }
    .knob {
      width: 40%; height: 40%; border-radius: 50%; background: #2a8;
      position: absolute; left: 50%; top: 50%;
      transform: translate(-50%, -50%);
      pointer-events: none;
    }
  </style>
</head>
<body>
  <div class="tabs">
    <button type="button" class="tab on" id="tabTest" onclick="showPage('test')">Test</button>
    <button type="button" class="tab" id="tabCtrl" onclick="showPage('ctrl')">Controller</button>
    <button type="button" class="tab" id="tabJoy" onclick="showPage('joy')">Joystick</button>
    <button type="button" class="tab" id="tabTune" onclick="showPage('tune')">Testing</button>
  </div>
  <script>
    function showPage(w) {
      document.getElementById('pageTest').style.display = (w === 'test') ? 'block' : 'none';
      document.getElementById('pageCtrl').style.display = (w === 'ctrl') ? 'block' : 'none';
      document.getElementById('pageJoy').style.display = (w === 'joy') ? 'block' : 'none';
      document.getElementById('pageTune').style.display = (w === 'tune') ? 'block' : 'none';
      document.getElementById('tabTest').className = (w === 'test') ? 'tab on' : 'tab';
      document.getElementById('tabCtrl').className = (w === 'ctrl') ? 'tab on' : 'tab';
      document.getElementById('tabJoy').className = (w === 'joy') ? 'tab on' : 'tab';
      document.getElementById('tabTune').className = (w === 'tune') ? 'tab on' : 'tab';
    }
  </script>
  <div id="pageTest">
  <h1>Motor test</h1>
  <p id="state">Stopped</p>
  <div class="row">
    <button id="go">Start all</button>
    <button id="kiwi">Omni F/B</button>
  </div>
  <p style="font-size:0.85rem;color:#aaa">Omni F/B: L298N is front (should stay still). M1 left-rear, M2 right-rear. Speed from M3 slider.</p>
  <label>M1 TB6612 A <span id="n1">180</span> (−80 PWM)
    <input id="s1" type="range" min="0" max="255" value="180">
    <button type="button" class="dir on" id="d1">Direction</button>
  </label>
  <label>M2 TB6612 B <span id="n2">180</span> (−80 PWM)
    <input id="s2" type="range" min="0" max="255" value="180">
    <button type="button" class="dir on" id="d2">Direction</button>
  </label>
  <label>M3 L298N <span id="n3">180</span>
    <input id="s3" type="range" min="0" max="255" value="180">
    <button type="button" class="dir" id="d3">Direction</button>
  </label>
  </div>
  <div id="pageCtrl" style="display:none">
    <h1>Controller</h1>
    <p style="font-size:0.85rem;color:#aaa">Hold a button. Speed from M3 slider. Release to stop.</p>
    <div class="pad">
      <button type="button" id="cFL" aria-label="forward left"><svg class="ico" viewBox="0 0 24 24"><path d="M6 6 L18 18 M6 6 L6 14 M6 6 L14 6"/></svg></button>
      <button type="button" id="cF" aria-label="forward"><svg class="ico" viewBox="0 0 24 24"><path d="M12 4 L12 20 M12 4 L6 10 M12 4 L18 10"/></svg></button>
      <button type="button" id="cFR" aria-label="forward right"><svg class="ico" viewBox="0 0 24 24"><path d="M18 6 L6 18 M18 6 L18 14 M18 6 L10 6"/></svg></button>
      <button type="button" id="cL" aria-label="left"><svg class="ico" viewBox="0 0 24 24"><path d="M4 12 L20 12 M4 12 L10 6 M4 12 L10 18"/></svg></button>
      <span></span>
      <button type="button" id="cR" aria-label="right"><svg class="ico" viewBox="0 0 24 24"><path d="M20 12 L4 12 M20 12 L14 6 M20 12 L14 18"/></svg></button>
      <button type="button" id="cBL" aria-label="back left"><svg class="ico" viewBox="0 0 24 24"><path d="M6 18 L18 6 M6 18 L14 18 M6 18 L6 10"/></svg></button>
      <button type="button" id="cB" aria-label="back"><svg class="ico" viewBox="0 0 24 24"><path d="M12 20 L12 4 M12 20 L6 14 M12 20 L18 14"/></svg></button>
      <button type="button" id="cBR" aria-label="back right"><svg class="ico" viewBox="0 0 24 24"><path d="M18 18 L6 6 M18 18 L10 18 M18 18 L18 10"/></svg></button>
    </div>
    <div class="spins">
      <button type="button" class="spin" id="cCCW" aria-label="ccw"><svg class="ico" viewBox="0 0 24 24"><path d="M7 7 A7 7 0 1 0 12 5"/><path d="M7 7 L3 7 L7 11"/></svg></button>
      <button type="button" class="spin" id="cCW" aria-label="cw"><svg class="ico" viewBox="0 0 24 24"><path d="M17 7 A7 7 0 1 1 12 5"/><path d="M17 7 L21 7 L17 11"/></svg></button>
    </div>
  </div>
  <div id="pageJoy" style="display:none">
    <h1>Joystick</h1>
    <p class="hint">Left = drive. Right = spin. Use both at once. Speed from M3 slider.</p>
    <div class="joyrow">
      <div class="stickwrap">
        <div class="stick" id="joyL"><div class="knob" id="knobL"></div></div>
        <p>Move</p>
      </div>
      <div class="stickwrap">
        <div class="stick" id="joyR"><div class="knob" id="knobR"></div></div>
        <p>Turn</p>
      </div>
    </div>
  </div>
  <div id="pageTune" style="display:none">
    <h1>Testing</h1>
    <p class="hint">Pick a left/right mix, then hold Left/Right here or on Controller. Default is A Classic.</p>
    <button type="button" class="mix on" id="mix0">A Classic — rear 50%, front 100%</button>
    <button type="button" class="mix" id="mix1">B Strong rear — rear 87%, front 100%</button>
    <button type="button" class="mix" id="mix2">C Slow rear — rear 32%, front 100%</button>
    <p class="hint" id="mixLabel">Using A</p>
    <div class="pad">
      <span></span><span></span><span></span>
      <button type="button" id="tL" aria-label="left"><svg class="ico" viewBox="0 0 24 24"><path d="M4 12 L20 12 M4 12 L10 6 M4 12 L10 18"/></svg></button>
      <span></span>
      <button type="button" id="tR" aria-label="right"><svg class="ico" viewBox="0 0 24 24"><path d="M20 12 L4 12 M20 12 L14 6 M20 12 L14 18"/></svg></button>
    </div>
  </div>
  <script src="/app.js"></script>
</body>
</html>
)HTML";

static const char APP_JS[] PROGMEM = R"JS(
    const btn = document.getElementById('go');
    const kiwi = document.getElementById('kiwi');
    const state = document.getElementById('state');
    function setUi(mode) {
      const runAll = mode === '1';
      const runKiwi = mode === '2';
      btn.textContent = runAll ? 'Stop' : 'Start all';
      btn.className = runAll ? 'stop' : '';
      kiwi.textContent = runKiwi ? 'Stop' : 'Omni F/B';
      kiwi.className = runKiwi ? 'stop' : '';
      state.textContent = runAll ? 'All wheels' : (runKiwi ? 'Omni F/B' : 'Stopped');
    }
    btn.onclick = function() {
      fetch('/toggle').then(function(r){ return r.text(); }).then(setUi);
    };
    kiwi.onclick = function() {
      fetch('/kiwi').then(function(r){ return r.text(); }).then(setUi);
    };
    fetch('/state').then(function(r){ return r.text(); }).then(setUi);
    function bind(id, nId, motor) {
      const el = document.getElementById(id);
      const n = document.getElementById(nId);
      el.oninput = function() {
        n.textContent = el.value;
        fetch('/speed?m=' + motor + '&v=' + el.value);
      };
    }
    function bindDir(id, motor) {
      const el = document.getElementById(id);
      el.onclick = function() {
        fetch('/dir?m=' + motor).then(function(r){ return r.text(); }).then(function(t){
          el.className = (t === '1') ? 'dir on' : 'dir';
        });
      };
    }
    bind('s1', 'n1', 1);
    bind('s2', 'n2', 2);
    bind('s3', 'n3', 3);
    bindDir('d1', 1);
    bindDir('d2', 2);
    bindDir('d3', 3);
    function setMixUi(n) {
      for (let i = 0; i < 3; i++) {
        document.getElementById('mix'+i).className = (String(n) === String(i)) ? 'mix on' : 'mix';
      }
      document.getElementById('mixLabel').textContent = 'Using ' + ['A','B','C'][n];
    }
    function pickMix(n) {
      fetch('/strafemix?n=' + n).then(function(){ setMixUi(n); });
    }
    document.getElementById('mix0').onclick = function(){ pickMix(0); };
    document.getElementById('mix1').onclick = function(){ pickMix(1); };
    document.getElementById('mix2').onclick = function(){ pickMix(2); };
    const cmds = {
      cF:[0,1,0], cB:[0,-1,0], cL:[-1,0,0], cR:[1,0,0],
      cFL:[-0.71,0.71,0], cFR:[0.71,0.71,0], cBL:[-0.71,-0.71,0], cBR:[0.71,-0.71,0],
      cCCW:[0,0,1], cCW:[0,0,-1], tL:[-1,0,0], tR:[1,0,0]
    };
    let holdT = null, curKey = null, ptrId = null;
    function sendKey(k) {
      const c = cmds[k];
      if (!c) return;
      fetch('/drive?vx='+c[0]+'&vy='+c[1]+'&w='+c[2]);
    }
    function startHold(k) {
      if (curKey === k) return;
      curKey = k;
      sendKey(k);
      if (holdT) clearInterval(holdT);
      holdT = setInterval(function(){ if (curKey) sendKey(curKey); }, 150);
    }
    function stopHold() {
      curKey = null;
      ptrId = null;
      if (holdT) { clearInterval(holdT); holdT = null; }
      fetch('/drive?vx=0&vy=0&w=0');
    }
    function keyAt(x, y) {
      const el = document.elementFromPoint(x, y);
      const b = el && el.closest ? el.closest('button') : null;
      if (b && cmds[b.id]) return b.id;
      return null;
    }
    function onDown(e) {
      const k = keyAt(e.clientX, e.clientY);
      if (!k) return;
      e.preventDefault();
      try { e.currentTarget.setPointerCapture(e.pointerId); } catch (err) {}
      ptrId = e.pointerId;
      startHold(k);
    }
    function onMove(e) {
      if (ptrId !== e.pointerId || curKey === null) return;
      const k = keyAt(e.clientX, e.clientY);
      if (k) startHold(k);
    }
    function onUp(e) {
      if (ptrId === e.pointerId) stopHold();
    }
    function bindRoot(id) {
      const root = document.getElementById(id);
      root.addEventListener('pointerdown', onDown);
      root.addEventListener('pointermove', onMove);
      root.addEventListener('pointerup', onUp);
      root.addEventListener('pointercancel', onUp);
      root.addEventListener('touchstart', function(e){
        const t = e.changedTouches[0];
        const k = keyAt(t.clientX, t.clientY);
        if (!k) return;
        e.preventDefault();
        ptrId = 1;
        startHold(k);
      }, {passive:false});
      root.addEventListener('touchmove', function(e){
        if (curKey === null) return;
        const t = e.changedTouches[0];
        const k = keyAt(t.clientX, t.clientY);
        if (k) startHold(k);
      }, {passive:false});
      root.addEventListener('touchend', function(){ if (curKey) stopHold(); });
    }
    bindRoot('pageCtrl');
    bindRoot('pageTune');
    var joy = { vx:0, vy:0, w:0 };
    var joyTimer = null;
    function joySend() {
      fetch('/drive?vx='+joy.vx.toFixed(3)+'&vy='+joy.vy.toFixed(3)+'&w='+joy.w.toFixed(3));
    }
    function joyLoopOn() {
      if (!joyTimer) joyTimer = setInterval(joySend, 80);
      joySend();
    }
    function joyLoopOff() {
      if (joy.vx === 0 && joy.vy === 0 && joy.w === 0) {
        if (joyTimer) { clearInterval(joyTimer); joyTimer = null; }
        fetch('/drive?vx=0&vy=0&w=0');
      }
    }
    function bindStick(baseId, knobId, kind) {
      const base = document.getElementById(baseId);
      const knob = document.getElementById(knobId);
      let pid = null;
      function setKnob(px, py) {
        knob.style.transform = 'translate(calc(-50% + '+px+'px), calc(-50% + '+py+'px))';
      }
      function apply(cx, cy) {
        const r = base.getBoundingClientRect();
        const mx = r.left + r.width / 2;
        const my = r.top + r.height / 2;
        const max = r.width * 0.38;
        let dx = cx - mx;
        let dy = cy - my;
        const mag = Math.sqrt(dx * dx + dy * dy);
        if (mag > max && mag > 0) {
          dx = dx * max / mag;
          dy = dy * max / mag;
        }
        setKnob(dx, dy);
        let nx = dx / max;
        let ny = -dy / max;
        if (nx > -0.12 && nx < 0.12) nx = 0;
        if (ny > -0.12 && ny < 0.12) ny = 0;
        if (kind === 'move') { joy.vx = nx; joy.vy = ny; }
        else { joy.w = -nx; }
        joyLoopOn();
      }
      function end() {
        pid = null;
        setKnob(0, 0);
        if (kind === 'move') { joy.vx = 0; joy.vy = 0; }
        else { joy.w = 0; }
        joyLoopOff();
      }
      base.addEventListener('pointerdown', function(e) {
        e.preventDefault();
        pid = e.pointerId;
        try { base.setPointerCapture(e.pointerId); } catch (err) {}
        apply(e.clientX, e.clientY);
      });
      base.addEventListener('pointermove', function(e) {
        if (pid !== e.pointerId) return;
        apply(e.clientX, e.clientY);
      });
      base.addEventListener('pointerup', function(e) { if (pid === e.pointerId) end(); });
      base.addEventListener('pointercancel', function(e) { if (pid === e.pointerId) end(); });
      base.addEventListener('touchstart', function(e) {
        const t = e.changedTouches[0];
        e.preventDefault();
        pid = t.identifier;
        apply(t.clientX, t.clientY);
      }, {passive:false});
      base.addEventListener('touchmove', function(e) {
        let i;
        for (i = 0; i < e.changedTouches.length; i++) {
          const t = e.changedTouches[i];
          if (pid === t.identifier) apply(t.clientX, t.clientY);
        }
        e.preventDefault();
      }, {passive:false});
      base.addEventListener('touchend', function(e) {
        let i;
        for (i = 0; i < e.changedTouches.length; i++) {
          if (pid === e.changedTouches[i].identifier) end();
        }
      });
    }
    bindStick('joyL', 'knobL', 'move');
    bindStick('joyR', 'knobR', 'turn');
)JS";

void sendProgmem(const char* type, const char* data) {
  size_t n = strlen_P(data);
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(n);
  server.send(200, type, "");
  const size_t CHUNK = 512;
  char buf[513];
  size_t off = 0;
  while (off < n) {
    size_t m = n - off;
    if (m > CHUNK) m = CHUNK;
    memcpy_P(buf, data + off, m);
    buf[m] = 0;
    server.sendContent(buf);
    off += m;
    yield();
  }
}

void handleIndex() {
  sendProgmem("text/html", INDEX_HTML);
}

void handleAppJs() {
  sendProgmem("text/javascript", APP_JS);
}

void handleState() {
  if (!testRunning) {
    server.send(200, "text/plain", "0");
  } else if (kiwiFb) {
    server.send(200, "text/plain", "2");
  } else {
    server.send(200, "text/plain", "1");
  }
}

static void sendRunState() {
  if (!testRunning) {
    server.send(200, "text/plain", "0");
  } else if (kiwiFb) {
    server.send(200, "text/plain", "2");
  } else {
    server.send(200, "text/plain", "1");
  }
}

void handleToggle() {
  if (testRunning && !kiwiFb) {
    startTbCoastRamp();
    testRunning = false;
    sendRunState();
    return;
  }
  kiwiFb = false;
  testRunning = true;
  testPhase = PHASE_FWD;
  phaseStartMs = millis();
  applyPhase();
  sendRunState();
}

void handleKiwi() {
  if (testRunning && kiwiFb) {
    startTbCoastRamp();
    testRunning = false;
    sendRunState();
    return;
  }
  kiwiFb = true;
  testRunning = true;
  testPhase = PHASE_FWD;
  phaseStartMs = millis();
  applyPhase();
  sendRunState();
}

void handleSpeed() {
  int m = server.arg("m").toInt();
  int v = constrain(server.arg("v").toInt(), 0, 255);
  if (m == 1) speed1 = v;
  else if (m == 2) speed2 = v;
  else if (m == 3) speed3 = v;
  if (testRunning && (testPhase == PHASE_FWD || testPhase == PHASE_REV)) {
    applyPhase();
  }
  server.send(200, "text/plain", "ok");
}

void handleDir() {
  int m = server.arg("m").toInt();
  bool inv = false;
  if (m == 1) { invert1 = !invert1; inv = invert1; }
  else if (m == 2) { invert2 = !invert2; inv = invert2; }
  else if (m == 3) { invert3 = !invert3; inv = invert3; }
  if (testRunning && (testPhase == PHASE_FWD || testPhase == PHASE_REV)) {
    applyPhase();
  }
  server.send(200, "text/plain", inv ? "1" : "0");
}

void handleHalt() {
  testRunning = false;
  kiwiFb = false;
  ctrlActive = false;
  startTbCoastRamp();
  server.send(200, "text/plain", "ok");
}

void handleStrafeMix() {
  int n = constrain(server.arg("n").toInt(), 0, 2);
  strafeMix = n;
  server.send(200, "text/plain", String(n));
}

void handleDrive() {
  float vxN = server.arg("vx").toFloat();
  float vyN = server.arg("vy").toFloat();
  float wN = server.arg("w").toFloat();
  testRunning = false;
  kiwiFb = false;
  if (vxN == 0 && vyN == 0 && wN == 0) {
    ctrlActive = false;
    startTbCoastRamp();
    server.send(200, "text/plain", "ok");
    return;
  }
  tbRamping = false;
  float V = (float)speed3;
  ctrlActive = true;
  lastDriveMs = millis();
  applyBody(vxN * V, vyN * V, wN * V);
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);

  pinMode(PIN_ENC1_A, INPUT_PULLUP);
  pinMode(PIN_ENC1_B, INPUT_PULLUP);
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);
  pinMode(PIN_ENC3_A, INPUT_PULLUP);
  pinMode(PIN_ENC3_B, INPUT_PULLUP);

  digitalWrite(PIN_STBY, HIGH);
  stopAllMotors();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC3_A), readEncoder3, RISING);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("http://");
    Serial.println(WiFi.localIP());
    if (MDNS.begin("omnirover")) {
      Serial.println("http://omnirover.local");
    }
  } else {
    Serial.println("WiFi failed");
  }

  server.on("/", handleIndex);
  server.on("/app.js", handleAppJs);
  server.on("/toggle", handleToggle);
  server.on("/state", handleState);
  server.on("/speed", handleSpeed);
  server.on("/dir", handleDir);
  server.on("/kiwi", handleKiwi);
  server.on("/drive", handleDrive);
  server.on("/halt", handleHalt);
  server.on("/strafemix", handleStrafeMix);
  server.begin();
}

void loop() {
  server.handleClient();
  updateTbCoastRamp();

  if (ctrlActive && (millis() - lastDriveMs > 800)) {
    ctrlActive = false;
    startTbCoastRamp();
  }

  if (!testRunning) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - phaseStartMs;
  unsigned long hold = (testPhase == PHASE_FWD || testPhase == PHASE_REV) ? 2000 : 1000;

  if (elapsed >= hold) {
    testPhase = (TestPhase)((testPhase + 1) % 4);
    phaseStartMs = now;
    applyPhase();
  }

  if (now - lastTickPrintMs >= 100) {
    lastTickPrintMs = now;
    Serial.print("M1: ");
    Serial.print(encoderTicks1);
    Serial.print("  M2: ");
    Serial.print(encoderTicks2);
    Serial.print("  M3: ");
    Serial.println(encoderTicks3);
  }
}

void applyPhase() {
  if (testPhase == PHASE_FWD) {
    tbRamping = false;
    Serial.println(kiwiFb ? "Kiwi FORWARD..." : "Moving FORWARD...");
    if (kiwiFb) {
      driveKiwiFb(true);
    } else {
      bool d1 = true != invert1;
      bool d2 = true != invert2;
      tbRampDir1 = d1;
      tbRampDir2 = d2;
      tbRampDir3 = true != invert3;
      setMotor1(tbPwm(speed1), d1);
      setMotor2(tbPwm(speed2), d2);
      setMotor3(speed3, tbRampDir3);
    }
  } else if (testPhase == PHASE_REV) {
    tbRamping = false;
    Serial.println(kiwiFb ? "Kiwi BACKWARD..." : "Moving REVERSE...");
    if (kiwiFb) {
      driveKiwiFb(false);
    } else {
      bool d1 = false != invert1;
      bool d2 = false != invert2;
      tbRampDir1 = d1;
      tbRampDir2 = d2;
      tbRampDir3 = false != invert3;
      setMotor1(tbPwm(speed1), d1);
      setMotor2(tbPwm(speed2), d2);
      setMotor3(speed3, tbRampDir3);
    }
  } else {
    Serial.println("STOPPING...");
    startTbCoastRamp();
  }
}

void driveKiwiFb(bool bodyForward) {
  float V = (float)speed3;
  if (!bodyForward) {
    V = -V;
  }
  applyBody(0, V, 0);
}

void applyBody(float vx, float vy, float wz) {
  // F/B (vy): rear TB6612 opposite, front still.
  // L/R (vx): rear scale k from Testing tab. A=0.5 classic, B=0.866, C=0.32 slow TB.
  float k = 0.32f;
  if (strafeMix == 0) k = 0.5f;
  else if (strafeMix == 1) k = 0.8660254f;
  float s1 = k * vx - 0.8660254f * vy + wz;
  float s2 = k * vx + 0.8660254f * vy + wz;
  float s3 = -vx + wz;
  float m = fabsf(s1);
  if (fabsf(s2) > m) m = fabsf(s2);
  if (fabsf(s3) > m) m = fabsf(s3);
  if (m > 255.0f) {
    s1 *= 255.0f / m;
    s2 *= 255.0f / m;
    s3 *= 255.0f / m;
  }
  driveTbSigned(1, s1);
  driveTbSigned(2, s2);
  driveM3Signed(s3);
}

void driveM3Signed(float signedCmd) {
  bool fwd = signedCmd >= 0;
  int mag = constrain((int)fabsf(signedCmd), 0, 255);
  if (mag > 0) {
    mag = constrain(mag + TB_OFFSET, 0, 255);
  }
  bool d = fwd != invert3;
  tbRampDir3 = d;
  setMotor3(mag, d);
}

void driveTbSigned(int motor, float signedCmd) {
  bool fwd = signedCmd >= 0;
  int mag = constrain((int)fabsf(signedCmd), 0, 255);
  if (motor == 1) {
    bool d = fwd != invert1;
    tbRampDir1 = d;
    setMotor1(mag, d);
  } else {
    bool d = fwd != invert2;
    tbRampDir2 = d;
    setMotor2(mag, d);
  }
}

int tbPwm(int slider) {
  return constrain(slider - TB_OFFSET, 0, 255);
}

void startTbCoastRamp() {
  tbRampFrom1 = lastTbOut1;
  tbRampFrom2 = lastTbOut2;
  tbRampFrom3 = lastTbOut3;
  tbRampStartMs = millis();
  tbRamping = true;
}

void updateTbCoastRamp() {
  if (!tbRamping) {
    return;
  }
  unsigned long e = millis() - tbRampStartMs;
  if (e >= TB_RAMP_MS) {
    setMotor1(0, tbRampDir1);
    setMotor2(0, tbRampDir2);
    setMotor3(0, tbRampDir3);
    tbRamping = false;
    return;
  }
  int p1 = (int)((long)tbRampFrom1 * (TB_RAMP_MS - e) / TB_RAMP_MS);
  int p2 = (int)((long)tbRampFrom2 * (TB_RAMP_MS - e) / TB_RAMP_MS);
  int p3 = (int)((long)tbRampFrom3 * (TB_RAMP_MS - e) / TB_RAMP_MS);
  setMotor1(p1, tbRampDir1);
  setMotor2(p2, tbRampDir2);
  setMotor3(p3, tbRampDir3);
}

void stopAllMotors() {
  setMotor1(0, true);
  setMotor2(0, true);
  setMotor3(0, true);
}

void setMotor1(int speed, bool forward) {
  lastTbOut1 = speed;
  ledcWrite(PIN_PWMA, speed);
  if (forward) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }
}

void setMotor2(int speed, bool forward) {
  lastTbOut2 = speed;
  ledcWrite(PIN_PWMB, speed);
  if (forward) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  }
}

void setMotor3(int speed, bool forward) {
  lastTbOut3 = speed;
  ledcWrite(PIN_ENA, speed);
  if (forward) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
}

void IRAM_ATTR readEncoder1() {
  if (digitalRead(PIN_ENC1_B) == HIGH) {
    encoderTicks1++;
  } else {
    encoderTicks1--;
  }
}

void IRAM_ATTR readEncoder2() {
  if (digitalRead(PIN_ENC2_B) == HIGH) {
    encoderTicks2++;
  } else {
    encoderTicks2--;
  }
}

void IRAM_ATTR readEncoder3() {
  if (digitalRead(PIN_ENC3_B) == HIGH) {
    encoderTicks3++;
  } else {
    encoderTicks3--;
  }
}
