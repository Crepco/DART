// Guided-capture wizard — 2 sets of 5 poses (10 shots), glasses interstitial,
// per-shot quality-gate feedback, and the /enroll/* session lifecycle.
// Every shot is accepted or rejected by the SERVER's quality gate; this file
// only drives the prompts and relays the gate's verdicts.
(() => {
  const POSE_LABEL = { front: "LOOK FRONT", down: "LOOK DOWN", left: "LOOK LEFT",
                       right: "LOOK RIGHT", up: "LOOK UP" };
  const POSE_HINT = {
    front: "face the camera straight on",
    down:  "tilt your chin down, eyes toward the floor",
    left:  "turn your head to YOUR left",
    right: "turn your head to YOUR right",
    up:    "tilt your chin up, eyes toward the ceiling",
  };
  const MAX_RETRIES = 3;     // after this, down/up may be skipped; front/left/
                             // right stay retry-only (server's directional floor)
  const PING_MS = 45000;     // keep-alive vs the server's 300s idle timeout

  let overlay = null, body = null, slots = [];
  let open = false, aborted = false;
  let session = null, cfg = null, glasses = null;
  let personName = "", lastCamera = null;
  let pingTimer = null;
  let pendingResolve = null;   // lets the × button unblock an awaited screen

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

  async function post(url, data) {
    const b = new URLSearchParams();
    for (const [k, v] of Object.entries(data)) if (v != null) b.set(k, String(v));
    try {
      const res = await fetch(url, { method: "POST", body: b });
      let j = {};
      if (res.status !== 204) { try { j = await res.json(); } catch (e) {} }
      j.__status = res.status;
      return j;
    } catch (e) {
      return { __status: 0, error: "request failed — is the server running?" };
    }
  }

  // ── modal shell ─────────────────────────────────────────────────────────────
  function build() {
    overlay = document.createElement("div");
    overlay.className = "wiz-overlay hidden";
    overlay.innerHTML =
      '<div class="wiz" role="dialog" aria-modal="true">' +
      '  <div class="wiz-head">' +
      '    <span class="wiz-title" id="wizTitle">GUIDED CAPTURE</span>' +
      '    <button class="wiz-close" id="wizClose" type="button"' +
      '            title="cancel capture">×</button>' +
      "  </div>" +
      '  <div class="wiz-body" id="wizBody"></div>' +
      '  <div class="wiz-progress" id="wizProgress"></div>' +
      "</div>";
    document.body.appendChild(overlay);
    body = overlay.querySelector("#wizBody");
    overlay.querySelector("#wizClose").addEventListener("click", () => abort());
  }

  function buildProgress(poses) {
    const grid = overlay.querySelector("#wizProgress");
    grid.textContent = "";
    slots = [];
    for (let set = 1; set <= 2; set++) {
      for (const pose of poses) {
        const d = document.createElement("div");
        d.className = "wiz-slot";
        d.textContent = pose.toUpperCase();
        grid.appendChild(d);
        slots.push({ el: d, set, pose });
      }
    }
  }

  function markSlot(set, pose, state, thumb) {
    const s = slots.find((x) => x.set === set && x.pose === pose);
    if (!s) return;
    s.el.className = "wiz-slot " + state;
    if (thumb) {
      s.el.textContent = "";
      const im = document.createElement("img");
      im.src = thumb;
      im.alt = "";
      s.el.appendChild(im);
    }
  }

  // One screen of the wizard. With buttons it resolves to the clicked value
  // (or "__abort" when the × unblocks it); without buttons it renders and
  // returns null immediately.
  function screen(spec) {
    body.textContent = "";
    const add = (tag, cls, text) => {
      const d = document.createElement(tag);
      d.className = cls;
      d.textContent = text;
      body.appendChild(d);
      return d;
    };
    if (spec.step) add("div", "wiz-step", spec.step);
    if (spec.big) add("div", "wiz-big", spec.big);
    if (spec.hint) add("div", "wiz-hint", spec.hint);
    if (spec.count != null) add("div", "wiz-count", spec.count);
    if (spec.msg) add("div", "wiz-msg" + (spec.msgCls ? " " + spec.msgCls : ""),
                      spec.msg);
    if (!spec.buttons || !spec.buttons.length) return null;
    const row = document.createElement("div");
    row.className = "wiz-btns";
    body.appendChild(row);
    return new Promise((resolve) => {
      pendingResolve = resolve;
      for (const [label, value, cls] of spec.buttons) {
        const b = document.createElement("button");
        b.type = "button";
        b.className = cls || "btn-line";
        b.textContent = label;
        b.addEventListener("click", () => { pendingResolve = null; resolve(value); });
        row.appendChild(b);
      }
    });
  }

  // ── lifecycle plumbing ──────────────────────────────────────────────────────
  function hide() {
    open = false;
    overlay.classList.add("hidden");
    if (window.DartDashboard) window.DartDashboard.repaint();  // re-enable Start Turret
  }

  function abort() {
    if (!open) return;
    aborted = true;
    if (pendingResolve) {
      const r = pendingResolve;
      pendingResolve = null;
      r("__abort");
    }
    stopPing();
    const sid = session;
    session = null;
    hide();
    if (sid) post("/enroll/cancel", { session: sid });
  }

  function startPing() {
    stopPing();
    pingTimer = setInterval(() => {
      if (session) post("/enroll/ping", { session });
    }, PING_MS);
  }

  function stopPing() {
    if (pingTimer) { clearInterval(pingTimer); pingTimer = null; }
  }

  function toast(text) {
    const t = document.createElement("div");
    t.className = "wiz-toast";
    t.textContent = text;
    document.body.appendChild(t);
    setTimeout(() => t.remove(), 4200);
  }

  // best-effort server-side cleanup if the tab closes mid-wizard
  window.addEventListener("beforeunload", () => {
    if (open && session) {
      const b = new URLSearchParams({ session });
      navigator.sendBeacon("/enroll/cancel",
        new Blob([b.toString()], { type: "application/x-www-form-urlencoded" }));
    }
  });

  // ── main flow ───────────────────────────────────────────────────────────────
  async function start(name, camera) {
    if (open) return;
    if (!overlay) build();
    open = true;
    aborted = false;
    session = null;
    cfg = null;
    personName = name;
    lastCamera = camera;
    overlay.querySelector("#wizTitle").textContent =
      "GUIDED CAPTURE — " + name.toUpperCase();
    overlay.querySelector("#wizProgress").textContent = "";
    overlay.classList.remove("hidden");
    if (window.DartDashboard) window.DartDashboard.repaint();  // freeze Start Turret

    const g = await screen({
      step: "STEP 1 OF 3 — SETUP",
      big: "GLASSES?",
      hint: "Does this person wear glasses? 10 photos are taken either way " +
            "(2 sets of 5 poses) — with glasses, they come off for set 2.",
      buttons: [["YES — WEARS GLASSES", "yes"], ["NO", "no", "btn-ghost"]],
    });
    if (g === "__abort" || aborted) return;
    glasses = g;

    if (!(await beginSession(name, camera, false))) return;
    buildProgress(cfg.poses);

    for (let set = 1; set <= 2 && !aborted; set++) {
      if (set === 2) {
        const c = await screen({
          step: "SET 1 COMPLETE — 5 OF 10 SHOTS DONE",
          big: glasses === "yes" ? "REMOVE GLASSES NOW" : "HALFWAY THERE",
          hint: glasses === "yes"
            ? "Take the glasses off — the next 5 photos are captured without " +
              "them so both looks match."
            : "Same 5 poses once more — a second pass makes the match more robust.",
          buttons: [["CONTINUE", "go"]],
        });
        if (c === "__abort" || aborted) return;
      }
      for (const pose of cfg.poses) {
        if (!(await capturePose(set, pose)) || aborted) return;
      }
    }
    if (!aborted) await doFinish();
  }

  async function beginSession(name, camera, overwrite) {
    screen({ step: "CONNECTING", big: "···",
             hint: "starting capture session" });
    const j = await post("/enroll/begin",
                         { name, camera, overwrite: overwrite ? "1" : null });
    if (aborted) {
      if (j.ok) post("/enroll/cancel", { session: j.session });
      return false;
    }
    if (j.__status === 409 && j.error === "exists") {
      const c = await screen({
        big: "ALREADY ENROLLED",
        hint: '"' + name + '" already exists in the database. Overwrite the ' +
              "stored identity with these new captures?",
        buttons: [["OVERWRITE", "ow"], ["CANCEL", "no", "btn-ghost"]],
      });
      if (c !== "ow" || aborted) { if (!aborted) hide(); return false; }
      return beginSession(name, camera, true);
    }
    if (!j.ok) {
      await screen({ big: "CANNOT START", msg: j.error || "capture failed",
                     msgCls: "err", buttons: [["CLOSE", "x", "btn-ghost"]] });
      hide();
      return false;
    }
    session = j.session;
    cfg = j;
    if (j.cancelled_previous) {
      toast('previous capture for "' + j.cancelled_previous.name +
            '" discarded (' + j.cancelled_previous.accepted + " shots)");
    }
    startPing();
    return true;
  }

  async function capturePose(set, pose) {
    const required = cfg.required_poses.includes(pose);
    const stepLabel = "SET " + set + " OF 2" +
      (glasses === "yes" ? (set === 1 ? " — GLASSES ON" : " — GLASSES OFF") : "") +
      " — SHOT " + (cfg.poses.indexOf(pose) + 1) + " OF " + cfg.per_set;
    let fails = 0;

    while (!aborted) {
      markSlot(set, pose, "cur");
      screen({ step: stepLabel, big: POSE_LABEL[pose],
               hint: POSE_HINT[pose] + " — hold still", count: "3" });
      const countEl = body.querySelector(".wiz-count");
      for (const c of ["3", "2", "1"]) {
        countEl.textContent = c;
        await sleep(650);
        if (aborted) return false;
      }
      countEl.textContent = "◉";

      const j = await post("/enroll/capture", { session, pose, set, glasses });
      if (aborted) return false;
      if (j.__status === 404) return expiredScreen();

      if (j.ok) {
        markSlot(set, pose, "ok", j.thumb);
        countEl.textContent = "✓";
        await sleep(450);
        return true;
      }

      fails++;
      const reason = j.reason || j.error || "capture failed";
      const buttons = [["RETRY", "retry"]];
      if (fails >= MAX_RETRIES && !required) {
        buttons.push(["SKIP POSE", "skip", "btn-ghost"]);
      }
      buttons.push(["CANCEL", "cancel", "btn-ghost"]);
      const hint = fails >= MAX_RETRIES && required
        ? "this pose is required for enrollment — adjust lighting or distance " +
          "and retry, or cancel the whole capture"
        : POSE_HINT[pose];
      const c = await screen({
        step: stepLabel, big: POSE_LABEL[pose], hint,
        msg: "✗ " + reason + " (attempt " + fails + ")", msgCls: "err",
        buttons,
      });
      if (c === "retry") continue;
      if (c === "skip") { markSlot(set, pose, "skip"); return true; }
      abort();   // explicit CANCEL; "__abort" already ran abort() — it no-ops
      return false;
    }
    return false;
  }

  async function expiredScreen() {
    stopPing();
    session = null;
    const c = await screen({
      big: "SESSION EXPIRED",
      msg: "the capture session timed out — nothing was saved",
      msgCls: "err",
      buttons: [["RESTART CAPTURE", "again"], ["CLOSE", "x", "btn-ghost"]],
    });
    hide();
    if (c === "again") start(personName, lastCamera);
    return false;
  }

  async function doFinish() {
    screen({ step: "STEP 3 OF 3", big: "PROCESSING",
             hint: "saving identity + reloading the classifier database",
             count: "···" });
    const j = await post("/enroll/finish", { session });
    if (aborted) return;
    if (j.__status === 404) { await expiredScreen(); return; }

    if (j.ok) {
      stopPing();
      session = null;
      if (window.DartDashboard) {
        window.DartDashboard.refreshAuthorized();
        window.DartDashboard.clearName();
      }
      await screen({
        big: "✓ AUTHORIZED",
        msg: '"' + j.name + '" enrolled with ' + j.accepted +
             " accepted captures",
        msgCls: "ok",
        buttons: [["DONE", "x"]],
      });
      hide();
      return;
    }

    // Safety net: the wizard never skips required poses, so a directional-floor
    // 422 shouldn't happen here — but the session survives it, so offer retakes.
    const m = /missing required poses: (.+)/.exec(j.error || "");
    if (j.__status === 422 && m) {
      const c = await screen({
        big: "POSES MISSING",
        msg: j.error, msgCls: "err",
        hint: "these poses are required — retake them now?",
        buttons: [["RETAKE", "re"], ["CANCEL", "cancel", "btn-ghost"]],
      });
      if (c !== "re" || aborted) { abort(); return; }
      for (const pose of m[1].split(", ")) {
        if (!(await capturePose(2, pose)) || aborted) return;
      }
      return doFinish();
    }

    await screen({ big: "ENROLLMENT FAILED", msg: j.error || "unknown error",
                   msgCls: "err", buttons: [["CLOSE", "x", "btn-ghost"]] });
    abort();
  }

  window.DartWizard = {
    open: (name, camera) => start(name, camera),
    isOpen: () => open,
  };
})();
