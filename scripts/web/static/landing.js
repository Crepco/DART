// Landing page: scan for connected cameras + Arduino serial ports and feed the
// choices into the DART launch form on submit.

const sel = document.getElementById("cameraSelect");
const fields = () => document.querySelectorAll(".cameraField");

function sync() {
  const v = sel.value;
  fields().forEach((f) => { f.value = v; });
}

async function loadCams() {
  sel.innerHTML = '<option value="">scanning…</option>';
  try {
    const j = await (await fetch("/cameras", { cache: "no-store" })).json();
    sel.innerHTML = "";
    if (!j.cameras || !j.cameras.length) {
      sel.innerHTML = '<option value="">default (0)</option>';
    } else {
      j.cameras.forEach((i) => {
        const o = document.createElement("option");
        o.value = i; o.textContent = "Camera " + i;
        sel.appendChild(o);
      });
    }
  } catch (e) {
    sel.innerHTML = '<option value="">default (0)</option>';
  }
  sync();
}

sel.addEventListener("change", sync);
document.getElementById("refreshCams").addEventListener("click", loadCams);
loadCams();

// ── Arduino serial port ───────────────────────────────────────────────────
const portSel = document.getElementById("portSelect");
const portFields = () => document.querySelectorAll(".portField");

function syncPort() {
  const v = portSel.value;
  portFields().forEach((f) => { f.value = v; });
}

async function loadPorts() {
  portSel.innerHTML = '<option value="">scanning…</option>';
  let def = "";
  try {
    const j = await (await fetch("/ports", { cache: "no-store" })).json();
    def = j.default || "";
    portSel.innerHTML = `<option value="">default (${def || "config"})</option>`;
    (j.ports || []).forEach((p) => {
      const o = document.createElement("option");
      o.value = p.device;
      o.textContent = p.desc && p.desc !== p.device ? `${p.device} — ${p.desc}` : p.device;
      portSel.appendChild(o);
    });
  } catch (e) {
    portSel.innerHTML = '<option value="">default</option>';
  }
  syncPort();
}

portSel.addEventListener("change", syncPort);
document.getElementById("refreshPorts").addEventListener("click", loadPorts);
loadPorts();
