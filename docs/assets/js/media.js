/* Two things live here:
   1. HUD — a reconstruction of the overlay realsense_basket_detector.py draws, built from its
      own cv2 calls (box, centre dot, four labels, translucent status bar). Not camera footage.
   2. Annotation engine — timed overlay boxes and captions for the match clips. Reads a JSON
      track from a <script type="application/json"> inside each figure, and falls back to a
      placeholder when the video file is absent, so the page degrades cleanly. */

/* ---------------- 1. detector HUD ---------------- */
(function () {
  const cv = document.getElementById('hudCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');

  const S = { dist: 4.20, horiz: -0.31, conf: 0.87, fps: 24.3 };
  const TARGET = 5.726 * Math.cos(55 * Math.PI / 180); // 3.284, as the node computes it

  function draw() {
    const W = 640, H = 480;
    const err = S.dist - TARGET;
    const centred = Math.abs(S.horiz) < 0.15;
    const ranged = Math.abs(err) < 0.1;

    /* backdrop — deliberately schematic, this is not a photo */
    const g = ctx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, '#1b2130'); g.addColorStop(1, '#0c0f16');
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);

    ctx.strokeStyle = 'rgba(255,255,255,.05)'; ctx.lineWidth = 1;
    for (let i = 0; i < W; i += 40) { ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, H); ctx.stroke(); }
    for (let i = 0; i < H; i += 40) { ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(W, i); ctx.stroke(); }
    ctx.fillStyle = '#171d29'; ctx.fillRect(0, 380, W, 100);

    /* hoop, placed from the current horizontal offset and range */
    const cx = 320 - S.horiz * 320;
    const scale = Math.max(0.35, Math.min(2.2, 3.2 / S.dist));
    const boxW = 150 * scale, boxH = 96 * scale;
    const cy = 210 + (S.dist - 3) * 12;

    ctx.strokeStyle = '#c8622a'; ctx.lineWidth = 5 * scale;
    ctx.beginPath(); ctx.ellipse(cx, cy, boxW * 0.42, boxH * 0.2, 0, 0, Math.PI * 2); ctx.stroke();
    ctx.strokeStyle = '#8c8f98'; ctx.lineWidth = 4 * scale;
    ctx.beginPath(); ctx.moveTo(cx + boxW * 0.42, cy); ctx.lineTo(cx + boxW * 0.7, cy - 26 * scale); ctx.stroke();
    ctx.fillStyle = 'rgba(240,240,245,.16)';
    ctx.fillRect(cx + boxW * 0.66, cy - 78 * scale, boxW * 0.5, 60 * scale);

    /* --- overlay, in the node's own drawing order --- */
    const x1 = Math.round(cx - boxW / 2), y1 = Math.round(cy - boxH / 2);
    const x2 = Math.round(cx + boxW / 2), y2 = Math.round(cy + boxH / 2);
    const colour = ranged ? '#00ff00' : '#ffa500';   // (0,255,0) : (0,165,255) in BGR

    ctx.strokeStyle = colour; ctx.lineWidth = 2;
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);

    ctx.fillStyle = '#ff0000';
    ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill();

    const labels = [
      `Basket ${S.conf.toFixed(2)}`,
      `D: ${S.dist.toFixed(2)}m G: ${S.dist.toFixed(2)}m`,
      `Error: ${err.toFixed(3)}m`,
      `Horiz: ${S.horiz.toFixed(2)}`
    ];
    ctx.font = '600 15px ui-monospace, monospace';
    let yOff = y1 - 10;
    labels.forEach(t => {
      const w = ctx.measureText(t).width;
      ctx.fillStyle = '#000';
      ctx.fillRect(x1, yOff - 13, w + 4, 17);
      ctx.fillStyle = colour;
      ctx.fillText(t, x1, yOff);
      yOff -= 20;
    });

    /* status bar */
    ctx.fillStyle = 'rgba(0,0,0,.7)';
    ctx.fillRect(0, 0, W, 80);
    ctx.fillStyle = '#fff'; ctx.font = '600 18px ui-monospace, monospace';
    ctx.fillText(`Detected: 1 baskets | FPS: ${S.fps.toFixed(1)}`, 10, 25);
    ctx.font = '15px ui-monospace, monospace';
    const vx = Math.max(-0.5, Math.min(0.5, 0.5 * S.horiz));
    const vy = Math.max(-0.5, Math.min(0.5, 0.5 * err));
    ctx.fillText(`PID Control: X=${vx.toFixed(3)}, Y=${vy.toFixed(3)}`, 10, 50);

    ctx.font = '600 17px ui-monospace, monospace';
    if (ranged && centred) {
      ctx.fillStyle = '#00ff00';
      ctx.fillText('POSITIONED CORRECTLY', 10, 75);
    } else {
      ctx.fillStyle = '#ffa500';
      ctx.fillText('ADJUSTING POSITION', 10, 75);
    }
    ctx.fillStyle = '#fff'; ctx.font = '12px ui-monospace, monospace';
    ctx.fillText(`Target: ${TARGET.toFixed(2)}m`, 500, 75);

    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set('hudErr', err.toFixed(3) + ' m');
    set('hudVx', vx.toFixed(3));
    set('hudVy', vy.toFixed(3));
    set('hudState', (ranged && centred) ? 'POSITIONED CORRECTLY' : 'ADJUSTING POSITION');
  }

  const bind = (id, fn) => { const e = document.getElementById(id); if (e) e.addEventListener('input', fn); };
  bind('hudDist', e => {
    S.dist = +e.target.value;
    document.getElementById('hudDistVal').textContent = S.dist.toFixed(2) + ' m';
    draw();
  });
  bind('hudHoriz', e => {
    S.horiz = +e.target.value;
    document.getElementById('hudHorizVal').textContent = S.horiz.toFixed(2);
    draw();
  });

  draw();
})();

/* ---------------- 2. timed annotation engine ---------------- */
(function () {
  document.querySelectorAll('[data-annotated]').forEach(setup);

  function setup(fig) {
    const video = fig.querySelector('video');
    const layer = fig.querySelector('.anno-layer');
    const trackEl = fig.querySelector('script[type="application/json"]');
    /* All three clips share one .wrap parent, so querySelector on the parent would
       return clip 1's nodes every time. Walk forward instead, stopping at the next figure. */
    const caption = nextSibling(fig, '.anno-caption');
    const timeline = nextSibling(fig, '.anno-timeline');
    if (!video || !layer || !trackEl) return;

    let track = [];
    try { track = JSON.parse(trackEl.textContent).annotations || []; } catch (e) { return; }

    /* If a source ever goes missing, hide the broken player rather than showing a
       dead rectangle; a placeholder is shown in its place when one is present. */
    video.addEventListener('error', () => {
      const slot = nextSibling(fig, '.slot');
      fig.style.display = 'none';
      if (slot) slot.style.display = 'block';
    });

    /* timeline markers */
    if (timeline && video.duration !== undefined) {
      const render = () => {
        const dur = video.duration || track.reduce((m, a) => Math.max(m, a.end), 1);
        timeline.innerHTML = '';
        track.forEach((a, i) => {
          if (a.chapter === false) return;   // short overlay-only cues stay out of the chapter list
          const b = document.createElement('button');
          b.className = 'btn-sm';
          b.textContent = `${fmt(a.start)} · ${a.label}`;
          b.addEventListener('click', () => { video.currentTime = a.start; video.play(); });
          timeline.appendChild(b);
        });
      };
      render();
    }

    video.addEventListener('timeupdate', () => {
      const t = video.currentTime;
      layer.innerHTML = '';
      let active = null;
      track.forEach(a => {
        if (t >= a.start && t < a.end) {
          active = a;
          (a.boxes || []).forEach(bx => {
            const d = document.createElement('div');
            d.className = 'anno-box';
            if (bx.colour) d.dataset.c = bx.colour;
            d.style.left = bx.x + '%';
            d.style.top = bx.y + '%';
            d.style.width = bx.w + '%';
            d.style.height = bx.h + '%';
            if (bx.label) {
              const tag = document.createElement('span');
              tag.className = 'anno-tag';
              tag.textContent = bx.label;
              d.appendChild(tag);
            }
            layer.appendChild(d);
          });
        }
      });
      if (caption) {
        caption.textContent = active ? active.note : '';
        caption.style.opacity = active ? '1' : '.35';
      }
    });
  }

  function nextSibling(el, sel) {
    let n = el.nextElementSibling;
    while (n) {
      if (n.matches(sel)) return n;
      if (n.tagName === 'FIGURE') return null;
      n = n.nextElementSibling;
    }
    return null;
  }

  function fmt(s) {
    const m = Math.floor(s / 60), r = Math.floor(s % 60);
    return `${m}:${String(r).padStart(2, '0')}`;
  }
})();
