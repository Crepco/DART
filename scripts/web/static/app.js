// Run page telemetry: polls /state ~5x/sec and paints the status strip.
// The video is a plain MJPEG <img>; the Authorize Person block lives in the
// shared authorize.js.

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
