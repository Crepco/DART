// Polls /state ~5x/sec and paints the status strip.
// The video itself is a plain MJPEG <img>, so JS only handles telemetry + the
// "Authorize Person" flow.

const $ = (id) => document.getElementById(id);
const set = (id, v) => { const el = $(id); if (el) el.textContent = v; };

function paint(s) {
  const banner = $("banner");
  if (s.status === "ERROR") {
    banner.textContent = "⚠ " + (s.error || "Runner error");
    banner.classList.remove("hidden"); banner.classList.add("err");
  } else if (s.status === "STOPPED") {
    banner.textContent = "Runner stopped.";
    banner.classList.remove("hidden", "err");
  } else if (!s.running || ["STARTING", "LOADING MODELS"].includes(s.status)
             || (s.status === "RUNNING" && s.fps === undefined)) {
    const t = s.elapsed != null ? ` (${Math.round(s.elapsed)}s)` : "";
    banner.textContent = "Starting camera + loading models…" + t;
    banner.classList.remove("hidden", "err");
  } else {
    banner.classList.add("hidden");
  }

  set("status", s.status || "—");
  set("serial", s.serial === "connected" ? "● connected"
              : s.serial === "preview"   ? "○ preview (no R3)" : "—");
  set("fps", s.fps != null ? s.fps.toFixed(1) : "—");

  // Face-auth backend + per-verdict latency. "CPU (degraded)" means a CUDA GPU
  // is present but onnxruntime-gpu is shadowed — 5-10x slower, styled as warning.
  const authEl = $("auth");
  if (authEl) {
    if (s.auth_provider) {
      authEl.textContent = s.auth_provider
        + (s.auth_ms != null ? ` · ${s.auth_ms}ms` : "");
      authEl.classList.toggle("warn", s.auth_provider.includes("degraded"));
    } else {
      authEl.textContent = "—";
      authEl.classList.remove("warn");
    }
  }
}

async function tick() {
  try {
    const r = await fetch("/state", { cache: "no-store" });
    const s = await r.json();
    if (!s.running && s.status === "IDLE") { window.location.href = "/"; return; }
    paint(s);
  } catch (e) { /* transient; keep polling */ }
}

setInterval(tick, 200);
tick();

// ── Authorize Person ─────────────────────────────────────────────────────
const authName    = $("authName");
const authCapture = $("authCapture");
const authFiles   = $("authFiles");

function authMsg(text, cls) {
  const el = $("authMsg");
  if (el) { el.textContent = text || ""; el.className = "auth-msg" + (cls ? " " + cls : ""); }
}

async function refreshAuthorized() {
  try {
    const j = await (await fetch("/authorized", { cache: "no-store" })).json();
    const names = j.identities || [];
    set("authList", names.length ? "authorized: " + names.join(", ")
                                 : "no one authorized yet");
  } catch (e) { /* keep whatever is there */ }
}

// POST an enrollment request; on "name exists" ask to overwrite and retry once.
async function submitAuth(url, buildBody) {
  const name = (authName.value || "").trim();
  if (!name) { authMsg("⚠ enter a name first", "err"); return; }

  authCapture.disabled = true;
  authMsg(url === "/authorize" ? "capturing… look at the camera, hold still"
                               : "processing photos…");
  try {
    let res = await fetch(url, { method: "POST", body: buildBody(name, false) });
    let j = await res.json();
    if (res.status === 409 && j.error === "exists") {
      if (!window.confirm(`"${name}" is already authorized. Overwrite?`)) {
        authMsg(""); return;
      }
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
    authFiles.value = "";
  }
}

if (authCapture) {
  authCapture.addEventListener("click", () => {
    submitAuth("/authorize", (name, overwrite) => {
      const b = new URLSearchParams();
      b.set("name", name);
      if (overwrite) b.set("overwrite", "1");
      return b;
    });
  });
}

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
