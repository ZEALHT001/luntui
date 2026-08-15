import "./styles.css";

const stateNames = ["theta", "dtheta", "x", "dx", "phi", "dphi"];
const inputNames = ["T", "Tp"];
const qDefaults = [1, 100, 200, 500, 20000, 1];
const rDefaults = [35, 1];

const $ = (id) => document.getElementById(id);

function renderWeights(container, names, values, prefix) {
  container.innerHTML = "";
  names.forEach((name, index) => {
    const label = document.createElement("label");
    label.innerHTML = `<span>${name}</span><input id="${prefix}${index}" type="number" step="0.001" value="${values[index]}">`;
    container.appendChild(label);
  });
}

function readWeights(prefix, count) {
  return Array.from({ length: count }, (_, index) => Number($(prefix + index).value));
}

function requestBody(sendRtt = false) {
  return {
    preset: $("preset").value,
    leg_length: Number($("legLength").value),
    q: readWeights("q", 6),
    r: readWeights("r", 2),
    send_rtt: sendRtt,
    dry_run: $("dryRun").checked,
    rtt_client_path: $("rttClient").value
  };
}

function setStatus(text, ok = true) {
  $("status").textContent = text;
  $("status").className = ok ? "status ok" : "status bad";
}

function appendLog(value) {
  const text = typeof value === "string" ? value : JSON.stringify(value, null, 2);
  $("log").textContent = `${new Date().toLocaleTimeString()} ${text}\n${$("log").textContent}`;
}

function renderK(matrix) {
  const table = $("kTable");
  const head = `<thead><tr><th></th>${stateNames.map((name) => `<th>${name}</th>`).join("")}</tr></thead>`;
  const body = matrix
    .map((row, rowIndex) => {
      const cells = row.map((value) => `<td>${Number(value).toFixed(6)}</td>`).join("");
      return `<tr><th>${inputNames[rowIndex]}</th>${cells}</tr>`;
    })
    .join("");
  table.innerHTML = `${head}<tbody>${body}</tbody>`;
}

async function post(path, body) {
  const res = await fetch(`${$("apiBase").value}${path}`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body)
  });
  const data = await res.json();
  if (!res.ok || !data.ok) {
    throw new Error(data.error || `HTTP ${res.status}`);
  }
  return data;
}

async function getDefaults() {
  const res = await fetch(`${$("apiBase").value}/api/defaults`);
  const data = await res.json();
  if (!res.ok || !data.presets) {
    throw new Error(data.error || `HTTP ${res.status}`);
  }
  return data;
}

async function calculate(sendRtt = false) {
  setStatus(sendRtt ? "计算并下发中..." : "计算中...");
  try {
    const data = await post("/api/calc", requestBody(sendRtt));
    renderK(data.k_matrix);
    $("payload").value = data.rtt_payload;
    appendLog(sendRtt ? data.rtt : "K calculated");
    setStatus(sendRtt ? "已完成" : "计算完成");
  } catch (error) {
    appendLog(error.message);
    setStatus("失败", false);
  }
}

async function init() {
  try {
    const defaults = await getDefaults();
    $("legLength").value = defaults.leg_length;
    $("rttClient").value = defaults.rtt_client_path;
    renderWeights($("qWeights"), defaults.state_order, defaults.q, "q");
    renderWeights($("rWeights"), defaults.input_order, defaults.r, "r");
    setStatus("默认值已加载");
  } catch (error) {
    renderWeights($("qWeights"), stateNames, qDefaults, "q");
    renderWeights($("rWeights"), inputNames, rDefaults, "r");
    appendLog(`defaults fallback: ${error.message}`);
  }
  calculate(false);
}

$("calcBtn").addEventListener("click", () => calculate(false));
$("sendBtn").addEventListener("click", () => calculate(true));

init();
