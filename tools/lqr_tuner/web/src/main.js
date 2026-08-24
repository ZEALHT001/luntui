import "./styles.css";

const stateNames = ["theta", "dtheta", "x", "dx", "phi", "dphi"];
const inputNames = ["T", "Tp"];
const qDefaults = [1, 100, 200, 500, 20000, 1];
const rDefaults = [35, 1];
const balanceParamIds = [
  ["thetaTarget", "theta_target"],
  ["pitchTarget", "pitch_target"],
  ["pitchDeadband", "pitch_deadband"],
  ["tpOutputEnable", "tp_output_enable"],
  ["wheelOutputUserEnable", "wheel_output_user_enable"],
  ["thetaDeadband", "theta_deadband"],
  ["lqrDthetaGain", "lqr_dtheta_gain"],
  ["lqrDthetaDeadband", "lqr_dtheta_deadband"],
  ["lqrDthetaLimit", "lqr_dtheta_limit"],
  ["tpPhiGain", "tp_phi_gain"],
  ["tpXGain", "tp_x_gain"],
  ["dpitchLpfAlpha", "dpitch_lpf_alpha"],
  ["xGain", "x_feedback_gain"],
  ["dxGain", "dx_feedback_gain"],
  ["xLimit", "x_feedback_limit"],
  ["dxLimit", "dx_feedback_limit"],
  ["speedControlEnable", "speed_control_enable"],
  ["speedTargetMax", "speed_target_max"],
  ["speedRampRate", "speed_ramp_rate"],
  ["speedDeadband", "speed_deadband"],
  ["stopSpeedThreshold", "stop_speed_threshold"],
  ["xHoldMaxStretch", "x_hold_max_stretch"],
  ["speedPitchGain", "speed_pitch_gain"],
  ["speedPitchLimit", "speed_pitch_limit"],
  ["speedPitchRampRate", "speed_pitch_ramp_rate"],
  ["kpLength", "kp_length"],
  ["kdLength", "kd_length"],
  ["maxForce", "max_force"],
  ["supportForce", "support_force"],
  ["lengthRampRate", "length_ramp_rate"],
  ["phi0RampRate", "phi0_ramp_rate"],
  ["kpPhi0", "kp_phi0"],
  ["kdPhi0", "kd_phi0"],
  ["maxPhi0Torque", "max_phi0_torque"],
  ["yawHoldEnable", "yaw_hold_enable"],
  ["yawHoldKp", "yaw_hold_kp"],
  ["yawHoldKd", "yaw_hold_kd"],
  ["yawHoldDeadband", "yaw_hold_deadband"],
  ["yawHoldCurrentLimit", "yaw_hold_current_limit"],
  ["turnCurrentMax", "turn_current_max"]
];
const savedParamsKey = "vmc-lqr-tuner-saved-params-v1";
let liveParamsTimer = 0;

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

function rttTarget() {
  const host = $("rttHost").value.trim() || "127.0.0.1";
  const port = Number($("rttPort").value) || 19021;
  return { rtt_host: host, rtt_port: port };
}

function requestBody(sendRtt = false) {
  return {
    preset: $("preset").value,
    leg_length: Number($("legLength").value),
    q: readWeights("q", 6),
    r: readWeights("r", 2),
    send_rtt: sendRtt,
    dry_run: $("dryRun").checked,
    ...rttTarget()
  };
}

function readBalanceParams() {
  return Object.fromEntries(balanceParamIds.map(([id, key]) => [key, Number($(id).value)]));
}

function setBalanceParams(params) {
  balanceParamIds.forEach(([id, key]) => {
    if (params[key] !== undefined) {
      $(id).value = params[key];
    }
  });
}

function paramsRequestBody() {
  return {
    params: readBalanceParams(),
    dry_run: $("dryRun").checked,
    ...rttTarget()
  };
}

function readUiState() {
  return {
    apiBase: $("apiBase").value,
    preset: $("preset").value,
    legLength: Number($("legLength").value),
    rttHost: $("rttHost").value,
    rttPort: Number($("rttPort").value),
    dryRun: $("dryRun").checked,
    liveParams: $("liveParams").checked,
    q: readWeights("q", 6),
    r: readWeights("r", 2),
    balanceParams: readBalanceParams()
  };
}

function applyUiState(state) {
  if (!state || typeof state !== "object") {
    return false;
  }
  if (state.apiBase !== undefined) $("apiBase").value = state.apiBase;
  if (state.preset !== undefined) $("preset").value = state.preset;
  if (state.legLength !== undefined) $("legLength").value = state.legLength;
  if (state.rttHost !== undefined) $("rttHost").value = state.rttHost;
  if (state.rttPort !== undefined) $("rttPort").value = state.rttPort;
  if (state.dryRun !== undefined) $("dryRun").checked = Boolean(state.dryRun);
  if (state.liveParams !== undefined) $("liveParams").checked = Boolean(state.liveParams);
  if (Array.isArray(state.q)) {
    state.q.slice(0, 6).forEach((value, index) => {
      $("q" + index).value = value;
    });
  }
  if (Array.isArray(state.r)) {
    state.r.slice(0, 2).forEach((value, index) => {
      $("r" + index).value = value;
    });
  }
  if (state.balanceParams) {
    setBalanceParams(state.balanceParams);
  }
  return true;
}

function loadSavedUiState() {
  try {
    const raw = localStorage.getItem(savedParamsKey);
    return raw ? JSON.parse(raw) : null;
  } catch (error) {
    appendLog(`saved params ignored: ${error.message}`);
    return null;
  }
}

function saveUiState() {
  try {
    localStorage.setItem(savedParamsKey, JSON.stringify(readUiState()));
    appendLog("参数已保存到浏览器本地");
    setStatus("参数已保存");
  } catch (error) {
    appendLog(error.message);
    setStatus("保存失败", false);
  }
}

function setStatus(text, ok = true) {
  $("status").textContent = text;
  $("status").className = ok ? "status ok" : "status bad";
}

function appendLog(value) {
  const text = typeof value === "string" ? value : JSON.stringify(value, null, 2);
  $("log").textContent = `${new Date().toLocaleTimeString()} ${text}\n${$("log").textContent}`;
}

function assertRttOk(rtt) {
  if (!rtt || rtt.ok !== false) {
    return;
  }
  throw new Error(rtt.error || `RTT failed: ${rtt.host}:${rtt.port}`);
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
    if (sendRtt) {
      assertRttOk(data.rtt);
    }
    appendLog(sendRtt ? data.rtt : "K calculated");
    setStatus(sendRtt ? "已完成" : "计算完成");
  } catch (error) {
    appendLog(error.message);
    setStatus("失败", false);
  }
}

async function sendBalanceParams(silent = false) {
  if (!silent) {
    setStatus("参数下发中...");
  }
  try {
    const data = await post("/api/send-params", paramsRequestBody());
    $("payload").value = data.payload;
    assertRttOk(data.rtt);
    appendLog(data.rtt);
    setStatus("参数已下发");
  } catch (error) {
    appendLog(error.message);
    setStatus("参数下发失败", false);
  }
}

async function clearKOverride() {
  setStatus("恢复固件 K 调度中...");
  try {
    const payload = JSON.stringify({ type: "vmc_lqr_clear" });
    const data = await post("/api/send-rtt", {
      payload,
      dry_run: $("dryRun").checked,
      ...rttTarget()
    });
    $("payload").value = payload;
    assertRttOk(data.rtt);
    appendLog(data.rtt);
    setStatus("已恢复固件 K 调度");
  } catch (error) {
    appendLog(error.message);
    setStatus("恢复 K 调度失败", false);
  }
}

async function sendRecoveryCommand(command, label) {
  setStatus(`${label} 下发中...`);
  try {
    const payload = JSON.stringify({ type: "vmc_recovery_cmd", cmd: command });
    const data = await post("/api/send-rtt", {
      payload,
      dry_run: $("dryRun").checked,
      ...rttTarget()
    });
    $("payload").value = payload;
    assertRttOk(data.rtt);
    appendLog(data.rtt);
    setStatus(`${label} 已下发`);
  } catch (error) {
    appendLog(error.message);
    setStatus(`${label} 失败`, false);
  }
}

function scheduleLiveParams() {
  if (!$("liveParams").checked) {
    return;
  }
  clearTimeout(liveParamsTimer);
  liveParamsTimer = setTimeout(() => sendBalanceParams(true), 250);
}

async function init() {
  let restored = false;
  try {
    const defaults = await getDefaults();
    $("legLength").value = defaults.leg_length;
    $("rttHost").value = defaults.rtt_host;
    $("rttPort").value = defaults.rtt_port;
    setBalanceParams(defaults.balance_params);
    renderWeights($("qWeights"), defaults.state_order, defaults.q, "q");
    renderWeights($("rWeights"), defaults.input_order, defaults.r, "r");
    setStatus("默认值已加载");
  } catch (error) {
    renderWeights($("qWeights"), stateNames, qDefaults, "q");
    renderWeights($("rWeights"), inputNames, rDefaults, "r");
    appendLog(`defaults fallback: ${error.message}`);
  }
  restored = applyUiState(loadSavedUiState());
  if (restored) {
    appendLog("已加载本地保存参数");
    setStatus("已加载本地参数");
  }
  calculate(false);
}

$("calcBtn").addEventListener("click", () => calculate(false));
$("sendBtn").addEventListener("click", () => calculate(true));
$("clearKBtn").addEventListener("click", clearKOverride);
$("sendParamsBtn").addEventListener("click", () => sendBalanceParams(false));
$("saveParamsBtn").addEventListener("click", saveUiState);
$("recoveryIdleBtn").addEventListener("click", () => sendRecoveryCommand("idle", "倒地待命"));
$("recoveryShortLegBtn").addEventListener("click", () => sendRecoveryCommand("short_leg", "收腿"));
$("recoveryThetaBtn").addEventListener("click", () => sendRecoveryCommand("theta_pd", "锁定 Theta"));
$("recoveryLqrJointBtn").addEventListener("click", () => sendRecoveryCommand("lqr_joint", "打开 LQR 关节"));
$("recoveryLqrWheelBtn").addEventListener("click", () => sendRecoveryCommand("lqr_wheel", "打开 LQR 轮子"));
balanceParamIds.map(([id]) => id).forEach((id) => {
  $(id).addEventListener("input", scheduleLiveParams);
  $(id).addEventListener("change", scheduleLiveParams);
});

init();
