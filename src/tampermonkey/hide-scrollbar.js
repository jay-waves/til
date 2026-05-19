// ==UserScript==
// @name         Hide Scrollbar + Top Progress Bar
// @namespace    https://tampermonkey.net/
// @version      1.1.0
// @description  Hide page scrollbars and show a thin adaptive scroll progress bar.
// @match        *://*/*
// @grant        none
// @run-at       document-start
// ==/UserScript==

(() => {
  'use strict';

  const BAR_ID = '__tm_scroll_progress_bar__';
  const STYLE_ID = '__tm_scroll_progress_style__';

  let scrollTarget = null;
  let ticking = false;

  function injectStyle() {
    if (document.getElementById(STYLE_ID)) return;

    const style = document.createElement('style');
    style.id = STYLE_ID;
    style.textContent = `
      :root {
        color-scheme: light dark;
      }

      html, body {
        scrollbar-width: none !important;
        -ms-overflow-style: none !important;
      }

      html::-webkit-scrollbar,
      body::-webkit-scrollbar,
      *::-webkit-scrollbar {
        width: 0 !important;
        height: 0 !important;
        display: none !important;
      }

      #${BAR_ID} {
        position: fixed !important;
        top: 0 !important;
        left: 0 !important;
        width: 0%;
        height: 1.5px !important;
        z-index: 2147483647 !important;
        background: light-dark(
          rgba(0, 0, 0, 0.24),
          rgba(255, 255, 255, 0.32)
        ) !important;
        pointer-events: none !important;
        transition: width 80ms linear !important;
      }
    `;

    document.documentElement.appendChild(style);
  }

  function createBar() {
    if (document.getElementById(BAR_ID)) return;

    const bar = document.createElement('div');
    bar.id = BAR_ID;

    const mount = () => {
      if (document.body) {
        document.body.appendChild(bar);
        updateProgress();
      } else {
        requestAnimationFrame(mount);
      }
    };

    mount();
  }

  function isScrollable(el) {
    if (!el || el === document.body || el === document.documentElement) {
      return false;
    }

    const style = getComputedStyle(el);
    const overflowY = style.overflowY;

    const canScroll =
      overflowY === 'auto' ||
      overflowY === 'scroll' ||
      overflowY === 'overlay';

    return canScroll && el.scrollHeight > el.clientHeight + 1;
  }

  function findScrollTarget() {
    const doc = document.scrollingElement || document.documentElement;

    if (doc.scrollHeight > doc.clientHeight + 1) {
      return doc;
    }

    const elements = document.querySelectorAll('body *');
    let best = null;
    let bestScrollableDistance = 0;

    for (const el of elements) {
      if (!isScrollable(el)) continue;

      const distance = el.scrollHeight - el.clientHeight;

      if (distance > bestScrollableDistance) {
        best = el;
        bestScrollableDistance = distance;
      }
    }

    return best || doc;
  }

  function bindScrollTarget() {
    const nextTarget = findScrollTarget();

    if (nextTarget === scrollTarget) return;

    if (scrollTarget) {
      scrollTarget.removeEventListener('scroll', updateProgress);
    }

    scrollTarget = nextTarget;

    if (scrollTarget) {
      scrollTarget.addEventListener('scroll', updateProgress, { passive: true });
    }

    updateProgress();
  }

  function getProgress() {
    const el = scrollTarget || findScrollTarget();

    const scrollTop =
      el === document.documentElement || el === document.body
        ? window.scrollY || el.scrollTop
        : el.scrollTop;

    const maxScroll = el.scrollHeight - el.clientHeight;

    if (maxScroll <= 1) return 100;

    return Math.min(100, Math.max(0, (scrollTop / maxScroll) * 100));
  }

  function updateProgress() {
    if (ticking) return;

    ticking = true;

    requestAnimationFrame(() => {
      const bar = document.getElementById(BAR_ID);
      if (bar) {
        bar.style.width = `${getProgress()}%`;
      }
      ticking = false;
    });
  }

  function init() {
    injectStyle();
    createBar();
    bindScrollTarget();

    window.addEventListener('resize', () => {
      bindScrollTarget();
      updateProgress();
    });

    window.addEventListener('load', () => {
      bindScrollTarget();
      updateProgress();
    });

    setTimeout(bindScrollTarget, 500);
    setTimeout(bindScrollTarget, 1500);
    setTimeout(bindScrollTarget, 3000);
  }

  init();
})();
