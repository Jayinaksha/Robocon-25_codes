/* Hero reel: autoplay only when it is on screen and the viewer has not asked for
   reduced motion. The poster carries the frame either way, so nothing is lost. */
(function () {
  const v = document.querySelector('.hero-reel video');
  if (!v) return;

  const reduce = window.matchMedia('(prefers-reduced-motion: reduce)');
  const play = () => { const p = v.play(); if (p) p.catch(() => {}); };

  function apply() {
    if (reduce.matches) { v.pause(); v.currentTime = 0; }
    else play();
  }

  if (reduce.addEventListener) reduce.addEventListener('change', apply);
  apply();

  /* Stop decoding once it scrolls away — it is decoration, not content. */
  if ('IntersectionObserver' in window) {
    new IntersectionObserver(entries => {
      entries.forEach(e => {
        if (reduce.matches) return;
        if (e.isIntersecting) play(); else v.pause();
      });
    }, { threshold: 0.15 }).observe(v);
  }
})();
