// Landing page: scan for connected cameras and feed the choice into whichever
// launch form (FlowState+DART or Just DART) gets submitted.

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
