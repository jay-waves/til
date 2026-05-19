// ==UserScript==
// @name         Text Highlighter with Markdown Export
// @namespace    local.text-highlighter-export
// @version      1.3.4
// @description  Temporarily highlight selected text and export current-session highlights as Markdown.
// @match        *://*/*
// @grant        GM_registerMenuCommand
// ==/UserScript==

(function () {
  "use strict";

  const HIGHLIGHT_ATTR = "data-tm-highlight-id";

  let lastRange = null;
  let lastText = "";
  let highlights = [];

  function getPageKey() {
    return location.origin + location.pathname;
  }

  function rememberSelection() {
    const selection = window.getSelection();

    if (!selection || selection.rangeCount === 0) return;

    const text = selection.toString().trim();

    if (!text) return;

    lastRange = selection.getRangeAt(0).cloneRange();
    lastText = text;
  }

  function getUsableSelection() {
    const selection = window.getSelection();

    if (selection && selection.rangeCount > 0) {
      const text = selection.toString().trim();

      if (text) {
        return {
          range: selection.getRangeAt(0).cloneRange(),
          text
        };
      }
    }

    if (lastRange && lastText) {
      return {
        range: lastRange.cloneRange(),
        text: lastText
      };
    }

    return null;
  }

  function highlightRangeTextNodes(range, id) {
    const commonAncestor = range.commonAncestorContainer;
    const root =
      commonAncestor.nodeType === Node.TEXT_NODE
        ? commonAncestor.parentNode
        : commonAncestor;

    const walker = document.createTreeWalker(
      root,
      NodeFilter.SHOW_TEXT,
      {
        acceptNode(node) {
          if (!node.nodeValue.trim()) {
            return NodeFilter.FILTER_REJECT;
          }

          if (!range.intersectsNode(node)) {
            return NodeFilter.FILTER_REJECT;
          }

          if (shouldSkipHighlightNode(node)) {
            return NodeFilter.FILTER_REJECT;
          }

          return NodeFilter.FILTER_ACCEPT;
        }
      }
    );

    const textNodes = [];

    while (walker.nextNode()) {
      textNodes.push(walker.currentNode);
    }

    for (const node of textNodes) {
      wrapTextNodePart(node, range, id);
    }
  }

  function wrapTextNodePart(textNode, range, id) {
    let start = 0;
    let end = textNode.nodeValue.length;

    if (textNode === range.startContainer) {
      start = range.startOffset;
    }

    if (textNode === range.endContainer) {
      end = range.endOffset;
    }

    if (start >= end) return;

    const highlightRange = document.createRange();
    highlightRange.setStart(textNode, start);
    highlightRange.setEnd(textNode, end);

    const mark = document.createElement("mark");

    mark.setAttribute(HIGHLIGHT_ATTR, id);
    mark.style.backgroundColor = "#fff59d";
    mark.style.color = "inherit";
    mark.style.padding = "0 2px";
    mark.style.borderRadius = "2px";
    mark.style.boxDecorationBreak = "clone";
    mark.style.webkitBoxDecorationBreak = "clone";
    mark.title = "Tampermonkey highlight";

    try {
      highlightRange.surroundContents(mark);
    } catch {
      // Skip fragile DOM fragments instead of wrecking the page.
    }
  }

  function shouldSkipHighlightNode(node) {
    const parent = node.parentElement;

    if (!parent) return true;

    return Boolean(
      parent.closest(
        [
          "script",
          "style",
          "textarea",
          "input",
          "select",
          "option",
          "button",
          "svg",
          "canvas",
          "[contenteditable='true']",
          `[${HIGHLIGHT_ATTR}]`
        ].join(",")
      )
    );
  }

  function highlightSelection() {
    const selected = getUsableSelection();

    if (!selected) {
      return;
    }

    const { range, text } = selected;
    const id = crypto.randomUUID();

    highlightRangeTextNodes(range, id);

    highlights.push({
      id,
      text,
      title: document.title || "Untitled",
      url: location.href,
      pageKey: getPageKey(),
      createdAt: new Date().toISOString()
    });

    const selection = window.getSelection();
    if (selection) selection.removeAllRanges();

    lastRange = null;
    lastText = "";
  }

  function exportMarkdown() {
    if (!highlights.length) {
      return;
    }

    const grouped = new Map();

    for (const item of highlights) {
      const key = item.url || "Unknown URL";
      if (!grouped.has(key)) grouped.set(key, []);
      grouped.get(key).push(item);
    }

    const lines = [];

    lines.push("# Exported Highlights");
    lines.push("");
    lines.push(`Exported at: ${new Date().toISOString()}`);
    lines.push("");

    for (const [url, items] of grouped.entries()) {
      const title = items[0]?.title || "Untitled";

      lines.push(`## ${title}`);
      lines.push("");
      lines.push(`URL: ${url}`);
      lines.push("");

      items.forEach((item, index) => {
        lines.push(`### Highlight ${index + 1}`);
        lines.push("");
        lines.push(`> ${item.text.replace(/\n+/g, "\n> ")}`);
        lines.push("");
        lines.push(`Created: ${item.createdAt}`);
        lines.push("");
      });
    }

    const markdown = lines.join("\n");

    const blob = new Blob([markdown], {
      type: "text/markdown;charset=utf-8"
    });

    const downloadUrl = URL.createObjectURL(blob);
    const a = document.createElement("a");

    a.href = downloadUrl;
    a.download = "highlights.md";
    document.body.appendChild(a);
    a.click();
    a.remove();

    URL.revokeObjectURL(downloadUrl);
  }

  document.addEventListener("selectionchange", rememberSelection);
  document.addEventListener("mouseup", rememberSelection);
  document.addEventListener("keyup", rememberSelection);
  document.addEventListener("contextmenu", rememberSelection, true);

  document.addEventListener("keydown", event => {
    if (event.altKey && event.key.toLowerCase() === "h") {
      event.preventDefault();
      rememberSelection();
      highlightSelection();
    }
  });

  GM_registerMenuCommand("Highlight selected text", highlightSelection);
  GM_registerMenuCommand("Export highlights as Markdown", exportMarkdown);
})();
