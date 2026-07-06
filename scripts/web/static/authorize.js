// Authorize Person block — shared by the landing page and the run page.
//
// Self-contained by design: touches only its own block's DOM (defensive
// lookups) and only the /authorize, /authorize/upload and /authorized
// endpoints. Whether DART is running, starting, or stopped is decided
// server-side (409 responses) — no dependency on the run page's /state
// polling or any page-specific element.
(() => {
  const $ = (id) => document.getElementById(id);

  const authName    = $("authName");
  const authCapture = $("authCapture");
  const authFiles   = $("authFiles");
  if (!authName || !authCapture) return;   // page has no authorize block

  function authMsg(text, cls) {
    const el = $("authMsg");
    if (el) { el.textContent = text || ""; el.className = "auth-msg" + (cls ? " " + cls : ""); }
  }

  // Inline two-button prompt in the message area (no window.confirm).
  function confirmOverwrite(name) {
    return new Promise((resolve) => {
      const el = $("authMsg");
      if (!el) { resolve(false); return; }
      el.className = "auth-msg";
      el.textContent = "";
      const wrap  = document.createElement("div");
      wrap.className = "auth-confirm";
      const label = document.createElement("span");
      label.textContent = `"${name}" is already authorized —`;
      const yes = document.createElement("button");
      yes.type = "button"; yes.className = "yes"; yes.textContent = "Overwrite";
      const no  = document.createElement("button");
      no.type = "button"; no.className = "no"; no.textContent = "Cancel";
      const done = (v) => { el.textContent = ""; resolve(v); };
      yes.addEventListener("click", () => done(true));
      no.addEventListener("click", () => done(false));
      wrap.append(label, yes, no);
      el.appendChild(wrap);
      yes.focus();
    });
  }

  async function refreshAuthorized() {
    try {
      const j = await (await fetch("/authorized", { cache: "no-store" })).json();
      const names = j.identities || [];
      const el = $("authList");
      if (!el) return;
      el.textContent = "";
      if (!names.length) {
        const none = document.createElement("span");
        none.className = "none";
        none.textContent = "no one authorized yet";
        el.appendChild(none);
      } else {
        names.forEach((n) => {
          const chip = document.createElement("span");
          chip.className = "chip";
          chip.textContent = n;
          el.appendChild(chip);
        });
      }
    } catch (e) { /* keep whatever is there */ }
  }

  // Landing page only: its camera picker, if present, selects the device for a
  // standalone capture (DART stopped). Absent on the run page — the server
  // uses the live runner's frames there anyway.
  function selectedCamera() {
    const sel = $("cameraSelect");
    return sel && sel.value !== "" ? sel.value : null;
  }

  // POST an enrollment request; on "name exists" ask to overwrite and retry once.
  async function submitAuth(url, buildBody) {
    const name = (authName.value || "").trim();
    if (!name) {
      authName.classList.add("err");
      authName.focus();
      authMsg("⚠ enter a name first", "err");
      return;
    }

    const captureLabel = authCapture.innerHTML;
    authCapture.disabled = true;
    authCapture.innerHTML = '<span class="spin"></span>'
      + (url === "/authorize" ? "capturing…" : "processing…");
    authMsg(url === "/authorize" ? "look at the camera, hold still"
                                 : "processing photos…");
    try {
      let res = await fetch(url, { method: "POST", body: buildBody(name, false) });
      let j = await res.json();
      if (res.status === 409 && j.error === "exists") {
        authCapture.disabled = false;
        authCapture.innerHTML = captureLabel;
        if (!(await confirmOverwrite(name))) { authMsg(""); return; }
        authCapture.disabled = true;
        authCapture.innerHTML = '<span class="spin"></span>'
          + (url === "/authorize" ? "capturing…" : "processing…");
        res = await fetch(url, { method: "POST", body: buildBody(name, true) });
        j = await res.json();
      }
      if (j.ok) {
        authMsg(`✔ ${j.name} authorized (${j.accepted}/${j.captured} frames)`, "ok");
        authName.value = "";
        refreshAuthorized();
      } else {
        authMsg("⚠ " + (j.error || "enrollment failed"), "err");
      }
    } catch (e) {
      authMsg("⚠ request failed", "err");
    } finally {
      authCapture.disabled = false;
      authCapture.innerHTML = captureLabel;
      if (authFiles) authFiles.value = "";
    }
  }

  authName.addEventListener("input", () => authName.classList.remove("err"));

  authCapture.addEventListener("click", () => {
    submitAuth("/authorize", (name, overwrite) => {
      const b = new URLSearchParams();
      b.set("name", name);
      if (overwrite) b.set("overwrite", "1");
      const cam = selectedCamera();
      if (cam != null) b.set("camera", cam);
      return b;
    });
  });

  if (authFiles) {
    authFiles.addEventListener("change", () => {
      if (!authFiles.files.length) return;
      const files = [...authFiles.files];   // keep a copy; the input is cleared after
      submitAuth("/authorize/upload", (name, overwrite) => {
        const fd = new FormData();
        fd.set("name", name);
        if (overwrite) fd.set("overwrite", "1");
        files.forEach((f) => fd.append("photos", f));
        return fd;
      });
    });
  }

  refreshAuthorized();
})();
