// Keep the reader's color preference without changing the Read the Docs layout.
(() => {
  const stored = () => {
    try { return localStorage.getItem("gnc-theme"); } catch (_) { return null; }
  };
  const preferred = stored() || (matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
  document.documentElement.dataset.theme = preferred;
  document.addEventListener("DOMContentLoaded", () => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "theme-toggle";
    button.setAttribute("role", "switch");
    button.setAttribute("aria-label", "Dark theme");
    button.innerHTML = `
      <span class="theme-toggle__thumb" aria-hidden="true"></span>
      <svg class="theme-toggle__sun" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true" focusable="false">
        <circle cx="12" cy="12" r="4"/>
        <path d="M12 2v2m0 16v2M2 12h2m16 0h2M4.93 4.93l1.42 1.42m11.3 11.3 1.42 1.42M4.93 19.07l1.42-1.42m11.3-11.3 1.42-1.42"/>
      </svg>
      <svg class="theme-toggle__moon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">
        <path d="M20.9 13.2A9 9 0 0 1 10.8 3.1 9 9 0 1 0 20.9 13.2Z"/>
      </svg>`;
    const update = () => {
      const dark = document.documentElement.dataset.theme === "dark";
      button.setAttribute("aria-checked", String(dark));
      button.title = "Switch to " + (dark ? "light" : "dark") + " theme";
    };
    button.addEventListener("click", () => {
      const next = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
      document.documentElement.dataset.theme = next;
      try { localStorage.setItem("gnc-theme", next); } catch (_) { /* Storage may be disabled. */ }
      update();
    });
    update();
    (document.querySelector(".wy-side-nav-search") || document.body).append(button);
  });
})();
