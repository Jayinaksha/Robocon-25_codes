/* Auto-alignment simulator.
   Ports PIDController + the error terms from realsense_basket_detector.py and the
   pid() mixer from Drive_with_auto.ino, so the committed axis mapping can be compared
   against a corrected one side by side. */

(function () {
  const cv = document.getElementById('pidCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');

  const HFOV_HALF_TAN = Math.tan(34.5 * Math.PI / 180); // D455 colour ≈ 69° horizontal
  const AUTO_SPEED = 60;      // PWM, translate
  const AUTO_ROT_SPEED = 40;  // PWM, the axes[3] branch
  const V_MAX = 1.2;          // m/s at PWM 255 — assumed, for the animation only
  const CLAMP = 0.5;

  class PID {
    constructor(kp, ki, kd, lo, hi) { Object.assign(this, { kp, ki, kd, lo, hi }); this.reset(); }
    reset() { this.prev = 0; this.integral = 0; }
    compute(err, dt) {
      if (dt <= 0) dt = 0.01;
      const d = (err - this.prev) / dt;
      this.integral += err * dt;
      let o = this.kp * err + this.ki * this.integral + this.kd * d;
      o = Math.max(this.lo, Math.min(this.hi, o));
      this.prev = err;
      return o;
    }
  }

  const S = {
    px: 1.6, py: 5.2,          // robot position, metres, hoop at origin
    trail: [],
    running: true,
    corrected: false,
    targetDistance: 5.726,
    shootingAngle: 55,
    applyCos: true,            // as committed: setpoint gets cos(), measurement does not
    kpx: 0.5, kpy: 0.5, ki: 0.0, kd: 0.0,
    hist: [],
    t: 0
  };

  let xPid = new PID(S.kpx, S.ki, S.kd, -CLAMP, CLAMP);
  let yPid = new PID(S.kpy, S.ki, S.kd, -CLAMP, CLAMP);

  function rebuildPids() {
    xPid = new PID(S.kpx, S.ki, S.kd, -CLAMP, CLAMP);
    yPid = new PID(S.kpy, S.ki, S.kd, -CLAMP, CLAMP);
  }

  function setpoint() {
    return S.applyCos
      ? S.targetDistance * Math.cos(S.shootingAngle * Math.PI / 180)
      : S.targetDistance;
  }

  /* Body-frame convention, taken from the firmware comment: +x is LEFT, +y is FRONT.
     Wheel pairs per controlMotors(): M1/M3 carry the x axis, M2/M4 carry the y axis.
     Both mixers return body-frame speeds; the integrator converts to world coordinates. */

  /* pid() exactly as written in Drive_with_auto.ino:310–344. */
  function mixCommitted(vx, vy) {
    const pwm = { M1: 0, M2: 0, M3: 0, M4: 0 };
    let front = 0, left = 0;

    if (vx < 0) {
      pwm.M2 = pwm.M4 = Math.abs(vx * AUTO_SPEED);
      front = +(pwm.M2 / 255) * V_MAX;                    // m2,m4 = 0 → +y = front
    } else if (vx > 0) {
      pwm.M2 = pwm.M4 = Math.abs(vx * AUTO_SPEED);
      front = -(pwm.M2 / 255) * V_MAX;                    // m2,m4 = 1 → −y = back
    } else if (vy < 0) {
      pwm.M2 = pwm.M4 = Math.abs(vx * AUTO_ROT_SPEED);    // reads axes[2], not axes[3]
      pwm.M1 = pwm.M3 = Math.abs(vy * AUTO_ROT_SPEED);
      left = +(pwm.M1 / 255) * V_MAX;                     // m1 = 0 → +x = left
      front = +(pwm.M2 / 255) * V_MAX;
    } else if (vy > 0) {
      pwm.M2 = pwm.M4 = Math.abs(vx * AUTO_ROT_SPEED);
      pwm.M1 = pwm.M3 = Math.abs(vy * AUTO_ROT_SPEED);
      left = +(pwm.M1 / 255) * V_MAX;                     // m1,m3 = 0 → +x = left
      front = -(pwm.M2 / 255) * V_MAX;                    // m2,m4 = 1 → back
    }
    return { pwm, front, left };
  }

  /* What the mapping was evidently meant to be: lateral error → x pair, range error → y pair. */
  function mixCorrected(vx, vy) {
    const pwm = { M1: 0, M2: 0, M3: 0, M4: 0 };
    pwm.M1 = pwm.M3 = Math.abs(vx * AUTO_SPEED);
    pwm.M2 = pwm.M4 = Math.abs(vy * AUTO_SPEED);
    const left = Math.sign(vx) * (pwm.M1 / 255) * V_MAX;   // +vx = hoop is left → strafe left
    const front = Math.sign(vy) * (pwm.M2 / 255) * V_MAX;  // +vy = too far → drive forward
    return { pwm, front, left };
  }

  let last = performance.now();
  let frame = null;

  function step(now) {
    const dtReal = Math.min(0.05, (now - last) / 1000);
    last = now;

    if (S.running) {
      const dt = dtReal;
      S.t += dt;

      const range = Math.hypot(S.px, S.py);
      const horiz = S.py > 0.15 ? (S.px / S.py) / HFOV_HALF_TAN : Math.sign(S.px) * 1.5;
      const horizClamped = Math.max(-1, Math.min(1, horiz));
      const distErr = range - setpoint();

      const vx = xPid.compute(horizClamped, dt);
      const vy = yPid.compute(distErr, dt);

      const m = S.corrected ? mixCorrected(vx, vy) : mixCommitted(vx, vy);

      /* body → world: the hoop sits at the origin and the robot faces it, so driving
         "front" closes py and strafing "left" closes px from the positive side. */
      S.px -= m.left * dt;
      S.py -= m.front * dt;
      S.py = Math.max(0.2, S.py);

      S.trail.push([S.px, S.py]);
      if (S.trail.length > 900) S.trail.shift();

      S.hist.push({ t: S.t, h: horizClamped, d: distErr });
      if (S.hist.length > 480) S.hist.shift();

      S._last = { range, horiz: horizClamped, distErr, vx, vy, m };
    }

    draw();
    frame = requestAnimationFrame(step);
  }

  const css = getComputedStyle(document.body);
  const c = n => css.getPropertyValue(n).trim() || '#888';

  /* world → canvas. Hoop at top centre, downrange increases downward. */
  const VIEW = { x0: 20, y0: 16, w: 400, h: 300, mPerPx: 0 };
  function w2c(px, py) {
    const scale = VIEW.h / 8.5;            // 8.5 m of court shown
    return [VIEW.x0 + VIEW.w / 2 + px * scale, VIEW.y0 + 26 + py * scale];
  }

  function draw() {
    const W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    const L = S._last;
    if (!L) return;

    /* court grid */
    ctx.strokeStyle = c('--line-soft'); ctx.lineWidth = 1;
    for (let m = 0; m <= 8; m++) {
      const [, y] = w2c(0, m);
      ctx.beginPath(); ctx.moveTo(VIEW.x0, y); ctx.lineTo(VIEW.x0 + VIEW.w, y); ctx.stroke();
    }
    for (let m = -3; m <= 3; m++) {
      const [x] = w2c(m, 0);
      ctx.beginPath(); ctx.moveTo(x, VIEW.y0 + 20); ctx.lineTo(x, VIEW.y0 + VIEW.h); ctx.stroke();
    }

    /* setpoint arc */
    const [hx, hy] = w2c(0, 0);
    const scale = VIEW.h / 8.5;
    ctx.strokeStyle = c('--green'); ctx.lineWidth = 1.6; ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.arc(hx, hy, setpoint() * scale, 0.12 * Math.PI, 0.88 * Math.PI); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = c('--green'); ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'left';
    ctx.fillText(`setpoint ${setpoint().toFixed(2)} m`, hx + setpoint() * scale * 0.72, hy + setpoint() * scale * 0.72);

    /* hoop */
    ctx.fillStyle = c('--accent');
    ctx.beginPath(); ctx.arc(hx, hy, 9, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = c('--accent'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(hx - 22, hy - 10); ctx.lineTo(hx + 22, hy - 10); ctx.stroke();
    ctx.fillStyle = c('--ink-faint'); ctx.font = '11px ui-monospace, monospace'; ctx.textAlign = 'center';
    ctx.fillText('hoop', hx, hy - 18);

    /* trail */
    if (S.trail.length > 1) {
      ctx.strokeStyle = c('--accent-line'); ctx.lineWidth = 1.5;
      ctx.beginPath();
      S.trail.forEach((p, i) => {
        const [x, y] = w2c(p[0], p[1]);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.stroke();
    }

    /* fov cone */
    const [rx, ry] = w2c(S.px, S.py);
    const coneLen = 3.4 * scale;
    ctx.fillStyle = c('--cyan'); ctx.globalAlpha = 0.10;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx - coneLen * HFOV_HALF_TAN, ry - coneLen);
    ctx.lineTo(rx + coneLen * HFOV_HALF_TAN, ry - coneLen);
    ctx.closePath(); ctx.fill();
    ctx.globalAlpha = 1;

    /* robot */
    ctx.fillStyle = c('--panel'); ctx.strokeStyle = c('--cyan'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.rect(rx - 13, ry - 13, 26, 26); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = c('--cyan');
    ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx, ry - 20); ctx.stroke();

    /* velocity vector, converted back to world for drawing */
    if (L.m) {
      ctx.strokeStyle = c('--accent'); ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(rx - L.m.left * scale * 1.6, ry - L.m.front * scale * 1.6);
      ctx.stroke();
    }

    /* --- error plot --- */
    const P = { x: 450, y: 40, w: 200, h: 118 };
    ctx.strokeStyle = c('--line'); ctx.lineWidth = 1;
    ctx.strokeRect(P.x, P.y, P.w, P.h);
    ctx.fillStyle = c('--ink-faint'); ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'left';
    ctx.fillText('error over time', P.x, P.y - 8);

    ctx.strokeStyle = c('--line-soft');
    ctx.beginPath(); ctx.moveTo(P.x, P.y + P.h / 2); ctx.lineTo(P.x + P.w, P.y + P.h / 2); ctx.stroke();

    if (S.hist.length > 1) {
      const n = S.hist.length;
      const sx = P.w / Math.max(1, n - 1);
      const plot = (key, colour, sc) => {
        ctx.strokeStyle = colour; ctx.lineWidth = 1.6;
        ctx.beginPath();
        S.hist.forEach((p, i) => {
          const v = Math.max(-1, Math.min(1, p[key] / sc));
          const x = P.x + i * sx, y = P.y + P.h / 2 - v * (P.h / 2 - 4);
          i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        });
        ctx.stroke();
      };
      plot('h', c('--cyan'), 1);
      plot('d', c('--accent'), 4);
    }
    ctx.fillStyle = c('--cyan'); ctx.fillText('horizontal', P.x + 4, P.y + 14);
    ctx.fillStyle = c('--accent'); ctx.fillText('range (±4 m)', P.x + 4, P.y + 28);

    /* --- wheel bars --- */
    const B = { x: 450, y: 200, w: 200 };
    ctx.fillStyle = c('--ink-faint'); ctx.font = '10px ui-monospace, monospace';
    ctx.fillText('wheel PWM', B.x, B.y - 8);
    ['M1', 'M2', 'M3', 'M4'].forEach((k, i) => {
      const v = L.m ? L.m.pwm[k] : 0;
      const y = B.y + i * 22;
      ctx.fillStyle = c('--bg-sunken'); ctx.fillRect(B.x + 26, y, B.w - 26, 13);
      ctx.fillStyle = v > 0 ? c('--accent') : c('--line');
      ctx.fillRect(B.x + 26, y, Math.max(1, (B.w - 26) * Math.min(1, v / 120)), 13);
      ctx.fillStyle = c('--ink-dim'); ctx.fillText(k, B.x, y + 11);
      ctx.fillStyle = c('--ink'); ctx.textAlign = 'right';
      ctx.fillText(v.toFixed(0), B.x + B.w - 4, y + 11);
      ctx.textAlign = 'left';
    });

    /* readouts */
    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set('pdRange', L.range.toFixed(2) + ' m');
    set('pdSet', setpoint().toFixed(2) + ' m');
    set('pdHoriz', L.horiz.toFixed(3));
    set('pdErr', L.distErr.toFixed(3) + ' m');
    set('pdVx', L.vx.toFixed(3));
    set('pdVy', L.vy.toFixed(3));

    const status = document.getElementById('pdStatus');
    if (status) {
      const ok = Math.abs(L.distErr) < 0.1 && Math.abs(L.horiz) < 0.15;
      status.textContent = ok ? 'POSITIONED' : 'ADJUSTING';
      status.style.color = ok ? c('--green') : c('--amber');
    }

    const diag = document.getElementById('pdDiag');
    if (diag && !S.corrected) {
      const stuck = Math.abs(L.vx) > 1e-6;
      diag.innerHTML = stuck
        ? '<b>The range branch is unreachable.</b> <code>pid()</code> only tests <code>axes[3]</code> after <code>axes[2] == 0</code> exactly — and a float PID output is essentially never exactly zero, so range correction never runs.'
        : '<b>Horizontal error is zero</b>, so the <code>axes[3]</code> branch is finally reachable — and it drives M2/M4 from <code>axes[2]</code>, which is now zero.';
    } else if (diag) {
      diag.innerHTML = '<b>Corrected mapping.</b> Lateral error drives the x pair, range error drives the y pair, and both act every tick.';
    }
  }

  /* ---------- controls ---------- */
  const bind = (id, fn) => { const e = document.getElementById(id); if (e) e.addEventListener('input', fn); };
  const setLbl = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };

  bind('pdKpx', e => { S.kpx = +e.target.value; setLbl('pdKpxVal', S.kpx.toFixed(2)); rebuildPids(); });
  bind('pdKpy', e => { S.kpy = +e.target.value; setLbl('pdKpyVal', S.kpy.toFixed(2)); rebuildPids(); });
  bind('pdKi', e => { S.ki = +e.target.value; setLbl('pdKiVal', S.ki.toFixed(3)); rebuildPids(); });
  bind('pdKd', e => { S.kd = +e.target.value; setLbl('pdKdVal', S.kd.toFixed(3)); rebuildPids(); });
  bind('pdTarget', e => { S.targetDistance = +e.target.value; setLbl('pdTargetVal', S.targetDistance.toFixed(2) + ' m'); });

  const btn = (id, fn) => { const e = document.getElementById(id); if (e) e.addEventListener('click', () => fn(e)); };

  btn('pdRun', e => {
    S.running = !S.running;
    e.classList.toggle('on', S.running);
    e.textContent = S.running ? 'Pause' : 'Run';
  });
  btn('pdReset', () => {
    S.px = 1.6; S.py = 5.2; S.trail = []; S.hist = []; S.t = 0; rebuildPids();
  });
  btn('pdMode', e => {
    S.corrected = !S.corrected;
    e.classList.toggle('on', S.corrected);
    e.textContent = S.corrected ? 'Corrected mapping' : 'As committed';
    S.trail = []; S.hist = []; S.t = 0; rebuildPids();
  });
  btn('pdCos', e => {
    S.applyCos = !S.applyCos;
    e.classList.toggle('on', !S.applyCos);
    e.textContent = S.applyCos ? 'Setpoint × cos(55°)' : 'Setpoint as written';
    rebuildPids();
  });

  /* drag to reposition the robot */
  cv.addEventListener('pointerdown', ev => {
    const rect = cv.getBoundingClientRect();
    const cx = (ev.clientX - rect.left) * (cv.width / rect.width);
    const cy = (ev.clientY - rect.top) * (cv.height / rect.height);
    if (cx > VIEW.x0 + VIEW.w) return;
    const scale = VIEW.h / 8.5;
    S.px = (cx - (VIEW.x0 + VIEW.w / 2)) / scale;
    S.py = Math.max(0.4, (cy - (VIEW.y0 + 26)) / scale);
    S.trail = []; S.hist = []; S.t = 0; rebuildPids();
  });

  frame = requestAnimationFrame(step);
})();
