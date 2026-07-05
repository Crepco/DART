// Polls /state ~5x/sec and paints the status panel.
// The video itself is a plain MJPEG <img>, so JS only handles telemetry/labels.

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
  set("persons", s.n_persons != null ? s.n_persons : "—");
  set("trackid", s.track_id != null ? "ID " + s.track_id : "none");
  set("pantilt", (s.pan != null) ? `${s.pan} / ${s.tilt}` : "—");

  if (camSel && !camBusy && s.camera != null) {
    const want = String(s.camera);
    const has = [...camSel.options].some((o) => o.value === want);
    if (has && camSel.value !== want) camSel.value = want;
  }

  // Fire state chip
  const chip = $("fireState");
  if (chip) {
    set("fireLabel", s.fire_label || "SAFE");
    let st = "0";
    if (s.fire) st = "1";
    else if (s.fire_label === "ARMED") st = "armed";
    else if (s.fire_label === "SAFE/AUTH") st = "focused";
    chip.dataset.fire = st;
  }
}

// ── live camera switching ──────────────────────────────────────────────
const camSel = document.getElementById("camSelect");
let camBusy = false;   // don't let /state overwrite the dropdown mid-switch

async function loadCams() {
  if (!camSel) return;
  try {
    const j = await (await fetch("/cameras", { cache: "no-store" })).json();
    camSel.innerHTML = "";
    (j.cameras || []).forEach((i) => {
      const o = document.createElement("option");
      o.value = i; o.textContent = "Camera " + i;
      camSel.appendChild(o);
    });
    if (j.current != null) camSel.value = j.current;
  } catch (e) { /* keep whatever is there */ }
}

if (camSel) {
  camSel.addEventListener("change", async () => {
    const idx = camSel.value;
    const msg = $("camMsg");
    camBusy = true;
    if (msg) { msg.textContent = "switching to camera " + idx + "…"; msg.className = "cam-msg"; }
    try {
      const body = new URLSearchParams(); body.set("camera", idx);
      const j = await (await fetch("/set_camera", { method: "POST", body })).json();
      if (msg) {
        msg.textContent = j.ok ? "" : ("⚠ " + (j.error || "switch failed"));
        msg.className = j.ok ? "cam-msg" : "cam-msg err";
      }
      if (!j.ok && j.current != null) camSel.value = j.current;  // revert dropdown
    } catch (e) {
      if (msg) { msg.textContent = "⚠ switch error"; msg.className = "cam-msg err"; }
    } finally {
      camBusy = false;
    }
  });
  loadCams();
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
