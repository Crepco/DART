// DART dashboard — state/log polling, telemetry paint, FPS sparkline, feed
// management, enrolled DB, folder-upload enrollment, and the turret toggle.
// Every displayed value comes from the backend (/state, /logs, /authorized,
// /cameras, /ports); nothing here fabricates telemetry.
(() => {
  const $ = (id) => document.getElementById(id);
  const CONST = window.DART_CONST ||
    { stopPan: 90, stopTilt: 90, baud: 115200, defaultPort: "COM3" };

  const RUNNING_SET = ["RUNNING", "TRACKING", "NO BODY"];
  const FPS_SAMPLES = 300;               // 60s of history at the 200ms poll

  let cur = { running: false, status: "IDLE" };
  let lastStateOk = 0;                   // Date.now() of last successful poll
  let feedOn = false;
  let toggleBusy = false;
  const fpsHist = [];

  // ── /state polling ─────────────────────────────────────────────────────────
  async function tick() {
    try {
      const res = await fetch("/state", { cache: "no-store" });
      cur = await res.json();
      lastStateOk = Date.now();
    } catch (e) { /* stale `cur`; the TELEMETRY STABLE chip goes dark */ }
    fpsHist.push(typeof cur.fps === "number" && cur.running ? cur.fps : null);
    if (fpsHist.length > FPS_SAMPLES) fpsHist.shift();
    paint();
  }

  // ── paint ──────────────────────────────────────────────────────────────────
  const sign = (n) => (n > 0 ? "+" : "") + n;

  // feed corner readout: loop rate + face-auth classify latency (auth_ms is
  // null until the classifier has run at least once)
  const fmtFeedStats = (s) => s.fps.toFixed(1) + " FPS · " +
    (s.auth_ms != null ? Math.round(s.auth_ms) + "ms AUTH" : "--ms AUTH");

  function setVal(id, text, state) {
    const el = $(id);
    el.textContent = text;
    if (state) el.dataset.state = state; else delete el.dataset.state;
  }

  function paintFeed(s) {
    const wantFeed = s.running === true;
    if (wantFeed && !feedOn) {
      $("video").src = "/video_feed?" + Date.now();   // fresh MJPEG socket
      feedOn = true;
    } else if (!wantFeed && feedOn) {
      $("video").removeAttribute("src");              // close the stream
      feedOn = false;
    }

    const cover = $("feedCover");
    const banner = $("feedBanner");
    const streaming = s.running && s.fps != null;
    cover.classList.toggle("hidden", streaming);
    cover.classList.toggle("err", s.status === "ERROR");
    if (s.status === "ERROR") {
      $("coverTitle").textContent = "SYSTEM ERROR";
      $("coverSub").textContent = s.error || "runner failed";
    } else if (s.running && s.fps == null) {
      $("coverTitle").textContent = s.status;
      $("coverSub").textContent = "starting camera + loading models… " +
        (s.elapsed != null ? s.elapsed.toFixed(0) + "s" : "");
    } else {
      $("coverTitle").textContent = "SYSTEM OFFLINE";
      $("coverSub").textContent = "START TURRET TO ENGAGE FEED";
    }

    // status banner on the live feed: real runner state only
    if (streaming) {
      const hot = s.fire_label === "FIRING" || s.fire_label === "ARMED";
      banner.textContent = "SYSTEM STATE: " + s.status +
        (hot ? "  ·  " + s.fire_label : "");
      banner.className = "feed-banner" +
        (s.fire_label === "FIRING" ? " err" : hot ? " warn" : "");
    } else {
      banner.className = "feed-banner hidden";
    }

    const stats = $("feedStats");
    stats.classList.toggle("hidden", !streaming);
    if (streaming) stats.textContent = fmtFeedStats(s);

    paintFeedBox();
    $("liveDot").classList.toggle("on", streaming);
  }

  // keep --feed-ar (stream aspect ratio) and --feed-head (panel-head height)
  // in sync so CSS can shrink-wrap the feed box to the video — survives
  // camera hot-swaps mid-run, since each poll re-reads the frame size
  let feedAr = 0, feedHead = 0;

  function paintFeedBox() {
    const stage = $("feedStage");
    const img = $("video");
    if (feedOn && img.naturalWidth > 0 && img.naturalHeight > 0) {
      const ar = img.naturalWidth / img.naturalHeight;
      if (Math.abs(ar - feedAr) > 0.01) {
        feedAr = ar;
        stage.style.setProperty("--feed-ar", ar.toFixed(4));
      }
    }
    const hh = document.querySelector(".feed-panel .panel-head").offsetHeight;
    if (hh && hh !== feedHead) {
      feedHead = hh;
      stage.style.setProperty("--feed-head", hh + "px");
    }
  }

  function paintTelemetry(s) {
    if (s.running && s.fps != null) {
      setVal("tFps", s.fps.toFixed(1) + " FPS");
      setVal("tTid", s.track_id != null ? "#" + s.track_id : "—",
             s.track_id != null ? null : "dim");
      setVal("tErr", "(" + sign(s.error_x ?? 0) + ", " + sign(s.error_y ?? 0) + ") px");
      const panDir = s.pan > CONST.stopPan ? "CW" : s.pan < CONST.stopPan ? "CCW" : "HOLD";
      const tiltDir = s.tilt > CONST.stopTilt ? "DOWN" : s.tilt < CONST.stopTilt ? "UP" : "HOLD";
      setVal("tPan", s.pan + " (" + panDir + ")");
      setVal("tTilt", s.tilt + " (" + tiltDir + ")");
      setVal("gFps", s.fps.toFixed(1));
    } else {
      ["tFps", "tTid", "tErr", "tPan", "tTilt", "gFps"]
        .forEach((id) => setVal(id, "—", "dim"));
    }
    drawGraph();
  }

  function paintChips(s) {
    const streaming = s.running && s.fps != null;
    $("chipTrack").classList.toggle("on", s.status === "TRACKING");
    $("chipCam").classList.toggle("on",
      streaming && RUNNING_SET.includes(s.status));
    $("chipTele").classList.toggle("on", Date.now() - lastStateOk < 1000);
    $("chipEng").classList.toggle("on", s.running === true && s.serial === "connected");
  }

  function paintFooter(s) {
    const btn = $("toggleTurret");
    const wizardOpen = window.DartWizard && window.DartWizard.isOpen();
    btn.disabled = toggleBusy || wizardOpen;
    if (s.running) {
      btn.textContent = "[■] STOP TURRET";
      btn.classList.add("stop");
    } else {
      btn.textContent = "[▶] START TURRET";
      btn.classList.remove("stop");
    }
    $("portSelect").disabled = s.running === true;

    const portInfo = s.running && s.port
      ? s.port + " (" + (s.baud || CONST.baud) + ")"
      : ($("portSelect").value || CONST.defaultPort) + " (" + CONST.baud + ")";
    $("portInfo").textContent = portInfo;
  }

  function paint() {
    const s = cur;
    paintFeed(s);
    paintTelemetry(s);
    paintChips(s);
    paintFooter(s);
  }

  // ── FPS sparkline (plain canvas, no libs) ──────────────────────────────────
  function drawGraph() {
    const cv = $("fpsGraph");
    const ctx = cv.getContext("2d");
    const w = cv.width, h = cv.height;
    ctx.clearRect(0, 0, w, h);

    const vals = fpsHist.filter((v) => v != null);
    const max = Math.max(30, ...vals) * 1.1;

    ctx.strokeStyle = "rgba(63,224,129,.18)";       // midline grid
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
    ctx.stroke();
    if (!vals.length) return;

    const x = (i) => (i / (FPS_SAMPLES - 1)) * w;
    const y = (v) => h - (v / max) * (h - 4) - 2;

    ctx.beginPath();
    let started = false;
    fpsHist.forEach((v, i) => {
      if (v == null) { started = false; return; }
      const px = x(i + (FPS_SAMPLES - fpsHist.length)), py = y(v);
      if (!started) { ctx.moveTo(px, py); started = true; }
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = "#3fe081";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  // ── system log window ──────────────────────────────────────────────────────
  let logCursor = 0;

  async function pollLogs() {
    try {
      const res = await fetch("/logs?since=" + logCursor, { cache: "no-store" });
      const j = await res.json();
      logCursor = j.latest;
      if (j.events.length) appendLog(j.events);
    } catch (e) { /* retry next poll */ }
  }

  function appendLog(events) {
    const el = $("logScroll");
    const stick = el.scrollHeight - el.scrollTop - el.clientHeight < 8;
    for (const e of events) {
      const line = document.createElement("div");
      line.className = "log-line " + e.level;
      const ts = document.createElement("span");
      ts.className = "log-ts";
      ts.textContent = new Date(e.ts * 1000).toTimeString().slice(0, 8);
      const tag = document.createElement("span");
      tag.className = "log-tag";
      tag.textContent = "[" + e.tag + "]";
      const msg = document.createElement("span");
      msg.className = "log-msg";
      msg.textContent = e.msg;
      line.append(ts, tag, msg);
      el.appendChild(line);
    }
    while (el.children.length > 300) el.removeChild(el.firstChild);
    if (stick) el.scrollTop = el.scrollHeight;
  }

  // ── enrolled database ──────────────────────────────────────────────────────
  async function refreshAuthorized() {
    try {
      const j = await (await fetch("/authorized", { cache: "no-store" })).json();
      const names = j.identities || [];
      const ol = $("dbList");
      ol.textContent = "";
      if (!names.length) {
        const li = document.createElement("li");
        li.className = "none";
        li.textContent = "no one enrolled yet";
        ol.appendChild(li);
      } else {
        names.forEach((n) => {
          const li = document.createElement("li");
          li.textContent = n;
          ol.appendChild(li);
        });
      }
      $("dbTotal").textContent = names.length;
    } catch (e) { /* keep whatever is there */ }
  }

  // ── authorization manager ──────────────────────────────────────────────────
  function authMsg(text, cls) {
    const el = $("authMsg");
    el.textContent = text || "";
    el.className = "auth-msg" + (cls ? " " + cls : "");
  }

  // Inline two-button prompt in the message area (no window.confirm).
  function confirmOverwrite(name) {
    return new Promise((resolve) => {
      const el = $("authMsg");
      el.className = "auth-msg";
      el.textContent = "";
      const wrap = document.createElement("div");
      wrap.className = "auth-confirm";
      const label = document.createElement("span");
      label.textContent = '"' + name + '" is already enrolled —';
      const yes = document.createElement("button");
      yes.type = "button"; yes.className = "yes"; yes.textContent = "Overwrite";
      const no = document.createElement("button");
      no.type = "button"; no.className = "no"; no.textContent = "Cancel";
      const done = (v) => { el.textContent = ""; resolve(v); };
      yes.addEventListener("click", () => done(true));
      no.addEventListener("click", () => done(false));
      wrap.append(label, yes, no);
      el.appendChild(wrap);
      yes.focus();
    });
  }

  function requireName() {
    const name = ($("authName").value || "").trim();
    if (!name) {
      $("authName").classList.add("err");
      $("authName").focus();
      authMsg("⚠ enter a name first", "err");
      return null;
    }
    return name;
  }

  function selectedCamera() {
    const v = $("cameraSelect").value;
    return v !== "" ? v : null;
  }

  $("authName").addEventListener("input",
    () => $("authName").classList.remove("err"));

  $("startCapture").addEventListener("click", () => {
    const name = requireName();
    if (!name) return;
    const source = document.querySelector('input[name="authSource"]:checked').value;
    if (source === "folder") {
      $("authFiles").click();          // multi-select -> /authorize/upload
    } else {
      authMsg("");
      window.DartWizard.open(name, selectedCamera());
    }
  });

  // Folder source: existing upload endpoint, one usable face is enough.
  $("authFiles").addEventListener("change", async () => {
    const input = $("authFiles");
    if (!input.files.length) return;
    const files = [...input.files];
    input.value = "";
    const name = requireName();
    if (!name) return;

    const btn = $("startCapture");
    const label = btn.textContent;
    const send = (overwrite) => {
      const fd = new FormData();
      fd.set("name", name);
      if (overwrite) fd.set("overwrite", "1");
      files.forEach((f) => fd.append("photos", f));
      return fetch("/authorize/upload", { method: "POST", body: fd });
    };

    btn.disabled = true;
    btn.textContent = "PROCESSING…";
    authMsg("processing photos…");
    try {
      let res = await send(false);
      let j = await res.json();
      if (res.status === 409 && j.error === "exists") {
        btn.disabled = false; btn.textContent = label;
        if (!(await confirmOverwrite(name))) { authMsg(""); return; }
        btn.disabled = true; btn.textContent = "PROCESSING…";
        res = await send(true);
        j = await res.json();
      }
      if (j.ok) {
        authMsg("✔ " + j.name + " enrolled (" + j.accepted + "/" +
                j.captured + " photos usable)", "ok");
        $("authName").value = "";
        refreshAuthorized();
      } else {
        authMsg("⚠ " + (j.error || "enrollment failed"), "err");
      }
    } catch (e) {
      authMsg("⚠ request failed", "err");
    } finally {
      btn.disabled = false;
      btn.textContent = label;
    }
  });

  // ── turret toggle + device pickers ─────────────────────────────────────────
  $("toggleTurret").addEventListener("click", async () => {
    toggleBusy = true;
    paint();
    try {
      if (cur.running) {
        await fetch("/stop", { method: "POST" });
      } else {
        const fd = new FormData();
        const cam = selectedCamera();
        if (cam != null) fd.set("camera", cam);
        if ($("portSelect").value !== "") fd.set("port", $("portSelect").value);
        await fetch("/start", { method: "POST", body: fd });
      }
      await tick();
    } catch (e) { /* next poll reflects reality */ }
    toggleBusy = false;
    paint();
  });

  async function loadDevices() {
    try {
      const j = await (await fetch("/cameras", { cache: "no-store" })).json();
      const sel = $("cameraSelect");
      const prev = sel.value;
      sel.textContent = "";
      (j.cameras || []).forEach((i) => {
        const o = document.createElement("option");
        o.value = i; o.textContent = i;
        sel.appendChild(o);
      });
      if (!sel.options.length) {
        const o = document.createElement("option");
        o.value = ""; o.textContent = "default";
        sel.appendChild(o);
      }
      const want = j.current != null ? String(j.current) : prev;
      if ([...sel.options].some((o) => o.value === want)) sel.value = want;
    } catch (e) { /* keep current list */ }
    try {
      const j = await (await fetch("/ports", { cache: "no-store" })).json();
      const sel = $("portSelect");
      const prev = sel.value;
      sel.textContent = "";
      const def = document.createElement("option");
      def.value = ""; def.textContent = "default (" + j.default + ")";
      sel.appendChild(def);
      (j.ports || []).forEach((p) => {
        const o = document.createElement("option");
        o.value = p.device; o.textContent = p.desc;
        sel.appendChild(o);
      });
      if ([...sel.options].some((o) => o.value === prev)) sel.value = prev;
    } catch (e) { /* keep current list */ }
    paint();
  }

  $("rescan").addEventListener("click", loadDevices);

  // camera stays hot-swappable while the turret runs
  $("cameraSelect").addEventListener("change", async () => {
    if (!cur.running) return;
    const fd = new FormData();
    fd.set("camera", $("cameraSelect").value);
    try {
      const j = await (await fetch("/set_camera", { method: "POST", body: fd })).json();
      if (!j.ok) authMsg("⚠ " + (j.error || "camera switch failed"), "err");
    } catch (e) { /* state poll will show the truth */ }
  });

  // ── collapsible sidebar panels (persisted) ─────────────────────────────────
  const FOLD_KEY = "dart.folds";

  // preserve log autoscroll across layout changes that resize .log-scroll
  function withLogStick(mutate) {
    const el = $("logScroll");
    const stick = el.scrollHeight - el.scrollTop - el.clientHeight < 8;
    mutate();
    if (stick) el.scrollTop = el.scrollHeight;
  }

  function initFolds() {
    let saved = {};
    try { saved = JSON.parse(localStorage.getItem(FOLD_KEY)) || {}; } catch (e) {}
    document.querySelectorAll(".fold-btn").forEach((btn) => {
      const panel = $(btn.dataset.fold);
      const apply = (folded) => {
        panel.classList.toggle("folded", folded);
        btn.setAttribute("aria-expanded", String(!folded));
      };
      // first visit: honor the panel's declared default; after that the
      // user's saved choice wins
      const sv = saved[btn.dataset.fold];
      apply(sv === undefined ? btn.dataset.foldDefault === "folded" : sv === true);
      btn.addEventListener("click", () => {
        const folded = !panel.classList.contains("folded");
        withLogStick(() => apply(folded));
        saved[btn.dataset.fold] = folded;
        try { localStorage.setItem(FOLD_KEY, JSON.stringify(saved)); } catch (e) {}
      });
    });
  }

  // wizard callbacks
  window.DartDashboard = {
    refreshAuthorized,
    clearName: () => { $("authName").value = ""; },
    repaint: paint,
  };

  // ── boot ──────────────────────────────────────────────────────────────────
  initFolds();
  tick();
  pollLogs();
  refreshAuthorized();
  loadDevices();
  setInterval(tick, 200);
  setInterval(pollLogs, 1000);
})();
