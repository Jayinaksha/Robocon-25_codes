/* Shooter trajectory model.
   The firmware's three PWM presets are real (100/120/150 on bot 1, 50/80/120 on bot 2);
   the PWM → exit-velocity constant is NOT — it is a calibration slider, because nothing in
   the repository measures it. Treat the arcs as a model to calibrate against, not a claim. */

(function () {
  const cv = document.getElementById('shootCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const G = 9.81;

  const PRESETS = {
    bot1: { name: 'Bot 1', speeds: [100, 120, 150], rest: 90 },
    bot2: { name: 'Bot 2', speeds: [50, 80, 120], rest: 100 }
  };

  const S = {
    bot: 'bot1',
    preset: 2,
    trim: 0,
    angle: 55,
    kv: 0.057,        // m/s per PWM count — set so bot 1's top preset lands near the rim
    releaseH: 0.85,
    range: 5.726,
    rimH: 2.43
  };

  const css = getComputedStyle(document.body);
  const c = n => css.getPropertyValue(n).trim() || '#888';

  const V = { x0: 56, y0: 20, w: 600, h: 280 };
  const world = { xMax: 8, yMax: 4.2 };
  const w2c = (x, y) => [
    V.x0 + (x / world.xMax) * V.w,
    V.y0 + V.h - (y / world.yMax) * V.h
  ];

  function pwmFor(i) { return PRESETS[S.bot].speeds[i] + S.trim; }
  function speedFor(pwm) { return Math.max(0, pwm) * S.kv; }

  function trajectory(v0, thetaDeg) {
    const th = thetaDeg * Math.PI / 180;
    const vx = v0 * Math.cos(th), vy = v0 * Math.sin(th);
    const pts = [];
    const dt = 0.008;
    for (let t = 0; t < 6; t += dt) {
      const x = vx * t;
      const y = S.releaseH + vy * t - 0.5 * G * t * t;
      if (y < -0.2 || x > world.xMax * 1.4) break;
      pts.push([x, y, t]);
    }
    return pts;
  }

  function heightAt(v0, thetaDeg, D) {
    const th = thetaDeg * Math.PI / 180;
    if (v0 <= 0) return -Infinity;
    const t = D / (v0 * Math.cos(th));
    return { h: S.releaseH + v0 * Math.sin(th) * t - 0.5 * G * t * t, t };
  }

  function requiredV0() {
    const th = S.angle * Math.PI / 180;
    const denom = 2 * Math.cos(th) * Math.cos(th) * (S.releaseH + S.range * Math.tan(th) - S.rimH);
    if (denom <= 0) return null;
    return Math.sqrt(G * S.range * S.range / denom);
  }

  function draw() {
    ctx.clearRect(0, 0, cv.width, cv.height);

    /* ground + grid */
    ctx.strokeStyle = c('--line-soft'); ctx.lineWidth = 1;
    for (let m = 0; m <= world.xMax; m++) {
      const [x] = w2c(m, 0);
      ctx.beginPath(); ctx.moveTo(x, V.y0); ctx.lineTo(x, V.y0 + V.h); ctx.stroke();
    }
    for (let m = 0; m <= 4; m++) {
      const [, y] = w2c(0, m);
      ctx.beginPath(); ctx.moveTo(V.x0, y); ctx.lineTo(V.x0 + V.w, y); ctx.stroke();
      ctx.fillStyle = c('--ink-faint'); ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'right'; ctx.fillText(m + ' m', V.x0 - 8, y + 4);
    }
    ctx.strokeStyle = c('--line'); ctx.lineWidth = 2;
    const [, gy] = w2c(0, 0);
    ctx.beginPath(); ctx.moveTo(V.x0, gy); ctx.lineTo(V.x0 + V.w, gy); ctx.stroke();

    /* hoop */
    const [hx, hy] = w2c(S.range, S.rimH);
    ctx.strokeStyle = c('--ink-faint'); ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(hx + 26, hy - 26); ctx.lineTo(hx + 26, gy); ctx.stroke();
    ctx.strokeStyle = c('--accent'); ctx.lineWidth = 4;
    ctx.beginPath(); ctx.moveTo(hx - 14, hy); ctx.lineTo(hx + 26, hy); ctx.stroke();
    ctx.fillStyle = c('--accent'); ctx.font = '11px ui-monospace, monospace'; ctx.textAlign = 'left';
    ctx.fillText(`${S.rimH.toFixed(2)} m @ ${S.range.toFixed(2)} m`, hx - 6, hy - 14);

    /* robot */
    const [bx, by] = w2c(0, S.releaseH);
    ctx.fillStyle = c('--panel'); ctx.strokeStyle = c('--line'); ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.rect(bx - 22, by, 40, gy - by); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = c('--cyan'); ctx.lineWidth = 3;
    const th = S.angle * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + 40 * Math.cos(th), by - 40 * Math.sin(th)); ctx.stroke();

    /* ghost arcs for the other two presets */
    PRESETS[S.bot].speeds.forEach((sp, i) => {
      if (i === S.preset) return;
      const pts = trajectory(speedFor(sp + S.trim), S.angle);
      ctx.strokeStyle = c('--line'); ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]);
      ctx.beginPath();
      pts.forEach((p, k) => { const [x, y] = w2c(p[0], p[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke(); ctx.setLineDash([]);
      if (pts.length) {
        const apex = pts.reduce((a, b) => (b[1] > a[1] ? b : a));
        const [ax, ay] = w2c(apex[0], apex[1]);
        ctx.fillStyle = c('--ink-faint'); ctx.font = '9px ui-monospace, monospace'; ctx.textAlign = 'center';
        ctx.fillText(`PWM ${sp + S.trim}`, ax, ay - 6);
      }
    });

    /* active arc */
    const pwm = pwmFor(S.preset);
    const v0 = speedFor(pwm);
    const pts = trajectory(v0, S.angle);
    const at = heightAt(v0, S.angle, S.range);
    const made = at && Math.abs(at.h - S.rimH) < 0.18 && v0 > 0;

    ctx.strokeStyle = made ? c('--green') : c('--accent');
    ctx.lineWidth = 2.6;
    ctx.beginPath();
    pts.forEach((p, k) => { const [x, y] = w2c(p[0], p[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();

    /* ball at the rim plane */
    if (at && isFinite(at.h) && at.h > -1) {
      const [px, py] = w2c(S.range, at.h);
      ctx.fillStyle = made ? c('--green') : c('--red');
      ctx.beginPath(); ctx.arc(px, py, 6, 0, Math.PI * 2); ctx.fill();
      if (!made) {
        ctx.strokeStyle = c('--red'); ctx.lineWidth = 1.4; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, hy); ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = c('--red'); ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'left';
        ctx.fillText(`${(at.h - S.rimH >= 0 ? '+' : '')}${(at.h - S.rimH).toFixed(2)} m`, px + 9, (py + hy) / 2);
      }
    }

    /* apex marker */
    if (pts.length) {
      const apex = pts.reduce((a, b) => (b[1] > a[1] ? b : a));
      const [ax, ay] = w2c(apex[0], apex[1]);
      ctx.strokeStyle = c('--ink-faint'); ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax, gy); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = c('--ink-dim'); ctx.font = '10px ui-monospace, monospace'; ctx.textAlign = 'center';
      ctx.fillText(`apex ${apex[1].toFixed(2)} m`, ax, ay - 8);
    }

    /* readouts */
    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set('shPwm', String(pwm));
    set('shV0', v0.toFixed(2) + ' m/s');
    set('shTof', at && isFinite(at.t) ? at.t.toFixed(2) + ' s' : '—');
    set('shAtRim', at && isFinite(at.h) ? at.h.toFixed(2) + ' m' : '—');

    const req = requiredV0();
    set('shReq', req ? `${req.toFixed(2)} m/s (PWM ${Math.round(req / S.kv)})` : 'unreachable at this angle');

    const verdict = document.getElementById('shVerdict');
    if (verdict) {
      verdict.textContent = made ? 'THROUGH THE RIM' : (at && at.h > S.rimH ? 'LONG / HIGH' : 'SHORT');
      verdict.style.color = made ? c('--green') : c('--red');
    }
  }

  const bind = (id, fn) => { const e = document.getElementById(id); if (e) e.addEventListener('input', fn); };
  const lbl = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };

  bind('shAngle', e => { S.angle = +e.target.value; lbl('shAngleVal', S.angle + '°'); draw(); });
  bind('shTrim', e => { S.trim = +e.target.value; lbl('shTrimVal', (S.trim >= 0 ? '+' : '') + S.trim); draw(); });
  bind('shKv', e => { S.kv = +e.target.value; lbl('shKvVal', S.kv.toFixed(3)); draw(); });
  bind('shRange', e => { S.range = +e.target.value; lbl('shRangeVal', S.range.toFixed(2) + ' m'); draw(); });
  bind('shRim', e => { S.rimH = +e.target.value; lbl('shRimVal', S.rimH.toFixed(2) + ' m'); draw(); });

  document.querySelectorAll('[data-preset]').forEach(b => {
    b.addEventListener('click', () => {
      S.preset = +b.dataset.preset;
      document.querySelectorAll('[data-preset]').forEach(o => o.classList.toggle('on', o === b));
      draw();
    });
  });
  document.querySelectorAll('[data-bot]').forEach(b => {
    b.addEventListener('click', () => {
      S.bot = b.dataset.bot;
      document.querySelectorAll('[data-bot]').forEach(o => o.classList.toggle('on', o === b));
      document.querySelectorAll('[data-preset]').forEach((o, i) => {
        o.textContent = 'PWM ' + PRESETS[S.bot].speeds[i];
      });
      draw();
    });
  });

  draw();
})();
