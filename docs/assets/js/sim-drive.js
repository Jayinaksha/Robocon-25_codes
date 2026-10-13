/* Drive mixer simulator.
   Faithful port of controlMotors() from arduino_codes/Drive_1_for_new_controllers.ino
   including Arduino's integer map(), which is where the trigger quantisation comes from. */

(function () {
  const cv = document.getElementById('driveCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');

  const MOVEMENT_SPEED = 180;
  const ROT_SPEED = 100;

  const state = {
    x: 0, y: 0,           // axes[0], axes[1]
    ccw: 1, cw: 1,        // axes[4], axes[5] — triggers rest at +1.0
    override: false,
    dragging: false
  };

  /* Arduino map() takes longs: the float argument is truncated toward zero first. */
  function arduinoMap(v, inMin, inMax, outMin, outMax) {
    const t = Math.trunc(v);
    return Math.trunc((t - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
  }

  /* Returns { branch, pwm:[M1..M4], dir:[m1..m4] } — dir null means "not written this pass". */
  function controlMotors(xAxis, yAxis, rAnticlock, rClock, override) {
    const x = xAxis * MOVEMENT_SPEED;
    const y = yAxis * MOVEMENT_SPEED;
    const rotAnticlock = arduinoMap(-rAnticlock, -1, 1, 0, ROT_SPEED);
    const rotclock = arduinoMap(-rClock, -1, 1, 0, ROT_SPEED);

    const out = (branch, pwm, dir) => ({ branch, pwm, dir, rotAnticlock, rotclock });

    if (override) return out('Override — all PWM 0', [0, 0, 0, 0], [null, null, null, null]);

    if (x > 0 && y > 0) return out('Second quadrant', [x, y, x, y], [0, 0, 0, 0]);
    if (x > 0 && y < 0) return out('Third quadrant', [x, -y, x, -y], [0, 1, 0, 1]);
    if (x < 0 && y > 0) return out('First quadrant', [-x, y, -x, y], [1, 0, 1, 0]);
    if (x < 0 && y < 0) return out('Fourth quadrant', [-x, -y, -x, -y], [1, 1, 1, 1]);

    if (x > 0 && y === 0) return out('Straight, x pair only', [x, 0, x, 0], [0, null, 0, null]);
    if (x < 0 && y === 0) return out('Straight, x pair only', [-x, 0, -x, 0], [1, null, 1, null]);
    if (x === 0 && y > 0) return out('Straight, y pair only', [0, y, 0, y], [null, 0, null, 0]);
    if (x === 0 && y < 0) return out('Straight, y pair only', [0, -y, 0, -y], [null, 1, null, 1]);

    if (rotAnticlock > 0) {
      return out('Rotate anticlockwise', [rotAnticlock, rotAnticlock, rotAnticlock, rotAnticlock], [0, 0, 1, 1]);
    }
    if (rotclock > 0) {
      return out('Rotate clockwise', [rotclock, rotclock, rotclock, rotclock], [1, 1, 0, 0]);
    }
    return out('Stop', [0, 0, 0, 0], [null, null, null, null]);
  }

  /* ---------- drawing ---------- */
  const css = getComputedStyle(document.body);
  const c = n => css.getPropertyValue(n).trim() || '#888';

  const STICK = { cx: 118, cy: 180, r: 88 };

  function draw() {
    const r = controlMotors(state.x, state.y, state.ccw, state.cw, state.override);
    const W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);

    /* --- joystick pad --- */
    ctx.strokeStyle = c('--line'); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(STICK.cx, STICK.cy, STICK.r, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(STICK.cx - STICK.r, STICK.cy); ctx.lineTo(STICK.cx + STICK.r, STICK.cy);
    ctx.moveTo(STICK.cx, STICK.cy - STICK.r); ctx.lineTo(STICK.cx, STICK.cy + STICK.r);
    ctx.setLineDash([3, 4]); ctx.stroke(); ctx.setLineDash([]);

    const px = STICK.cx + state.x * STICK.r;
    const py = STICK.cy - state.y * STICK.r;
    ctx.strokeStyle = c('--accent'); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(STICK.cx, STICK.cy); ctx.lineTo(px, py); ctx.stroke();
    ctx.fillStyle = c('--accent');
    ctx.beginPath(); ctx.arc(px, py, 9, 0, Math.PI * 2); ctx.fill();

    ctx.fillStyle = c('--ink-faint');
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('drag — axes[0], axes[1]', STICK.cx, STICK.cy + STICK.r + 24);
    ctx.fillText('+x left · +y front', STICK.cx, STICK.cy + STICK.r + 40);

    /* --- robot --- */
    const R = { cx: 430, cy: 180 };
    const arm = 76, wheelW = 46, wheelH = 24;

    ctx.strokeStyle = c('--line'); ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.arc(R.cx, R.cy, arm, 0, Math.PI * 2); ctx.stroke();
    ctx.setLineDash([]);

    const wheels = [
      { i: 0, name: 'M1', lx: 0, ly: -arm, horiz: true },   // front
      { i: 1, name: 'M2', lx: -arm, ly: 0, horiz: false },  // left
      { i: 2, name: 'M3', lx: 0, ly: arm, horiz: true },    // back
      { i: 3, name: 'M4', lx: arm, ly: 0, horiz: false }    // right
    ];

    wheels.forEach(w => {
      const pwm = Math.min(255, Math.abs(Math.round(r.pwm[w.i])));
      const frac = pwm / 255;
      const wx = R.cx + w.lx, wy = R.cy + w.ly;
      const ww = w.horiz ? wheelW : wheelH;
      const hh = w.horiz ? wheelH : wheelW;

      ctx.fillStyle = c('--bg-raised');
      ctx.strokeStyle = pwm > 0 ? c('--accent') : c('--line');
      ctx.lineWidth = 1.5;
      roundRect(wx - ww / 2, wy - hh / 2, ww, hh, 5);
      ctx.fill(); ctx.stroke();

      if (pwm > 0) {
        ctx.fillStyle = c('--accent');
        ctx.globalAlpha = 0.18 + 0.62 * frac;
        roundRect(wx - ww / 2, wy - hh / 2, ww, hh, 5);
        ctx.fill();
        ctx.globalAlpha = 1;
      }

      ctx.fillStyle = c('--ink');
      ctx.font = '600 11px ui-monospace, monospace';
      ctx.fillText(w.name, wx, wy + 4);

      const lbl = pwm > 0 ? `${pwm}` : '0';
      const dir = r.dir[w.i];
      ctx.fillStyle = pwm > 0 ? c('--accent') : c('--ink-faint');
      ctx.font = '10px ui-monospace, monospace';
      const ox = w.horiz ? 0 : (w.lx > 0 ? 46 : -46);
      const oy = w.horiz ? (w.ly > 0 ? 30 : -28) : 0;
      ctx.fillText(dir === null ? lbl : `${lbl} · dir ${dir}`, wx + ox, wy + oy + 3);

      /* thrust arrow */
      if (pwm > 0 && dir !== null) {
        const sign = dir === 0 ? -1 : 1;
        ctx.strokeStyle = c('--accent'); ctx.lineWidth = 2;
        ctx.beginPath();
        if (w.horiz) {
          const y0 = wy, x0 = wx + sign * 4;
          ctx.moveTo(x0, y0); ctx.lineTo(x0 + sign * 16 * (0.4 + frac), y0);
        } else {
          const x0 = wx, y0 = wy + sign * 4;
          ctx.moveTo(x0, y0); ctx.lineTo(x0, y0 + sign * 16 * (0.4 + frac));
        }
        ctx.stroke();
      }
    });

    /* body */
    ctx.fillStyle = c('--panel'); ctx.strokeStyle = c('--line'); ctx.lineWidth = 1.5;
    roundRect(R.cx - 34, R.cy - 34, 68, 68, 10); ctx.fill(); ctx.stroke();
    ctx.fillStyle = state.override ? c('--red') : c('--green');
    ctx.beginPath(); ctx.arc(R.cx, R.cy, 7, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = c('--ink-faint'); ctx.font = '10px ui-monospace, monospace';
    ctx.fillText(state.override ? 'OVERRIDE' : 'live', R.cx, R.cy + 26);

    /* branch label */
    ctx.textAlign = 'left';
    ctx.fillStyle = c('--ink-faint'); ctx.font = '11px ui-monospace, monospace';
    ctx.fillText('branch taken', 300, 36);
    ctx.fillStyle = state.override ? c('--red') : c('--accent');
    ctx.font = '600 15px ui-monospace, monospace';
    ctx.fillText(r.branch, 300, 58);

    /* readouts */
    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set('dvBranch', r.branch);
    set('dvX', (state.x * MOVEMENT_SPEED).toFixed(0));
    set('dvY', (state.y * MOVEMENT_SPEED).toFixed(0));
    set('dvCcw', r.rotAnticlock);
    set('dvCw', r.rotclock);
    set('dvPwm', r.pwm.map(p => Math.abs(Math.round(p))).join(' · '));

    const warn = document.getElementById('dvWarn');
    if (warn) {
      const partial = (Math.abs(state.ccw) < 1 && state.ccw !== 0) || (Math.abs(state.cw) < 1 && state.cw !== 0);
      if (r.branch.startsWith('Rotate') && state.x === 0 && state.y === 0 && (r.rotAnticlock === 50 || r.rotclock === 50)) {
        warn.style.display = 'block';
        warn.innerHTML = '<b>Quantised to 50.</b> A trigger anywhere strictly between its endpoints truncates to 0 inside <code>map()</code>, which returns exactly half of <code>rot_speed</code> — so the robot rotates at a fixed PWM 50 rather than proportionally.';
      } else if (partial) {
        warn.style.display = 'block';
        warn.innerHTML = '<b>Partial trigger travel is discarded.</b> <code>map()</code> truncates its float argument to a long, so the trigger only ever produces 0, 50 or 100.';
      } else {
        warn.style.display = 'none';
      }
    }
  }

  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  /* ---------- interaction ---------- */
  function pointerPos(e) {
    const rect = cv.getBoundingClientRect();
    return {
      x: (e.clientX - rect.left) * (cv.width / rect.width),
      y: (e.clientY - rect.top) * (cv.height / rect.height)
    };
  }

  function setStick(p) {
    let dx = (p.x - STICK.cx) / STICK.r;
    let dy = -(p.y - STICK.cy) / STICK.r;
    const m = Math.hypot(dx, dy);
    if (m > 1) { dx /= m; dy /= m; }
    // joy_node deadzone: 0.1
    state.x = Math.abs(dx) < 0.1 ? 0 : +dx.toFixed(2);
    state.y = Math.abs(dy) < 0.1 ? 0 : +dy.toFixed(2);
    draw();
  }

  cv.addEventListener('pointerdown', e => {
    const p = pointerPos(e);
    if (Math.hypot(p.x - STICK.cx, p.y - STICK.cy) < STICK.r + 24) {
      state.dragging = true;
      setStick(p);
      try { cv.setPointerCapture(e.pointerId); } catch (err) { /* capture is a nicety, not required */ }
    }
  });
  cv.addEventListener('pointermove', e => { if (state.dragging) setStick(pointerPos(e)); });
  cv.addEventListener('pointerup', e => {
    state.dragging = false;
    state.x = 0; state.y = 0;   // spring back to centre, like the real stick
    draw();
  });

  const bind = (id, fn) => { const e = document.getElementById(id); if (e) e.addEventListener('input', fn); };
  bind('dvCcwIn', e => { state.ccw = parseFloat(e.target.value); document.getElementById('dvCcwVal').textContent = state.ccw.toFixed(2); draw(); });
  bind('dvCwIn', e => { state.cw = parseFloat(e.target.value); document.getElementById('dvCwVal').textContent = state.cw.toFixed(2); draw(); });

  const ov = document.getElementById('dvOverride');
  if (ov) ov.addEventListener('click', () => {
    state.override = !state.override;
    ov.classList.toggle('on', state.override);
    ov.textContent = state.override ? 'Override ON' : 'Override OFF';
    draw();
  });

  draw();
  window.addEventListener('resize', draw);
})();
