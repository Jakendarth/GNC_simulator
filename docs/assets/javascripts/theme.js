// Keep the reader's color preference without changing the Read the Docs layout.
(() => {
  const stored = () => {
    try { return localStorage.getItem("gnc-theme"); } catch (_) { return null; }
  };
  const preferred = stored() || (matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
  document.documentElement.dataset.theme = preferred;
  document.addEventListener("DOMContentLoaded", () => {
    const button = document.createElement("button");
    button.className = "theme-toggle";
    const update = () => {
      const dark = document.documentElement.dataset.theme === "dark";
      button.textContent = dark ? "Light theme" : "Dark theme";
      button.setAttribute("aria-label", "Switch to " + (dark ? "light" : "dark") + " theme");
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
