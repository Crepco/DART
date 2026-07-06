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

  async function refreshAuthorized() {
    try {
      const j = await (await fetch("/authorized", { cache: "no-store" })).json();
      const names = j.identities || [];
      const el = $("authList");
      if (el) el.textContent = names.length ? "authorized: " + names.join(", ")
                                            : "no one authorized yet";
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
      if (authFiles) authFiles.value = "";
    }
  }

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
