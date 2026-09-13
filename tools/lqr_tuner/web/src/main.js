import "./styles.css";

const stateNames = ["theta", "dtheta", "x", "dx", "phi", "dphi"];
const inputNames = ["T", "Tp"];
const qDefaults = [1, 100, 150, 20, 10000, 1];
const rDefaults = [15, 0.3];
const kalmanDefaults = { q_v: 0.1, q_a: 0.1, r_wheel: 100, r_imu: 1e12 };
const balanceParamIds = [
  ["thetaTarget", "theta_target"],
  ["pitchTarget", "pitch_target"],
  ["pitchDeadband", "phi_deadband"],
  ["tpOutputEnable", "tp_output_enable"],
  ["wheelOutputUserEnable", "wheel_output_user_enable"],
  ["thetaDeadband", "theta_deadband"],
  ["lqrDthetaGain", "lqr_dtheta_gain"],
  ["lqrDthetaDeadband", "lqr_dtheta_deadband"],
  ["lqrDthetaLimit", "lqr_dtheta_limit"],
  ["wheelTorqueLimit", "wheel_torque_limit"],
  ["wheelCurrentLimit", "wheel_current_limit"],
  ["airborneDetectEnable", "airborne_detect_enable"],
  ["airborneWheelSpeedThreshold", "airborne_wheel_speed_threshold"],
  ["airborneJointTorqueThreshold", "airborne_joint_torque_threshold"],
  ["airborneConfirmMs", "airborne_confirm_ms"],
  ["airborneReleaseMs", "airborne_release_ms"],
  ["tpLimit", "tp_limit"],
  ["tpPhiGain", "tp_phi_gain"],
  ["tpXGain", "tp_x_gain"],
  ["dpitchLpfAlpha", "dpitch_lpf_alpha"],
  ["disturbanceObserverEnable", "disturbance_observer_enable"],
  ["disturbanceObserverGain", "disturbance_observer_gain"],
  ["disturbanceObserverLeak", "disturbance_observer_leak"],
  ["disturbanceObserverLimit", "disturbance_observer_limit"],
  ["disturbanceObserverSpeedLimit", "disturbance_observer_speed_limit"],
  ["disturbanceObserverDelay", "disturbance_observer_delay"],
  ["disturbanceObserverPitchGain", "disturbance_observer_pitch_gain"],
  ["disturbanceObserverThetaGain", "disturbance_observer_theta_gain"],
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
  ["jumpEnable", "jump_enable"],
  ["jumpCompressLength", "jump_compress_length"],
  ["jumpExtendLength", "jump_extend_length"],
  ["jumpRetractLength", "jump_retract_length"],
  ["jumpLandLength", "jump_land_length"],
  ["jumpKpLength", "jump_kp_length"],
  ["jumpKdLength", "jump_kd_length"],
  ["jumpMaxForce", "jump_max_force"],
  ["jumpSupportForce", "jump_support_force"],
  ["jumpCompressTimeoutMs", "jump_compress_timeout_ms"],
  ["jumpExtendTimeoutMs", "jump_extend_timeout_ms"],
  ["jumpRetractTimeoutMs", "jump_retract_timeout_ms"],
  ["jumpLandTimeoutMs", "jump_land_timeout_ms"],
  ["jumpCooldownMs", "jump_cooldown_ms"],
  ["jumpTotalTimeoutMs", "jump_total_timeout_ms"],
  ["jumpAttitudeLimit", "jump_attitude_limit"],
  ["jumpLandTorqueThreshold", "jump_land_torque_threshold"],
  ["jumpLandConfirmMs", "jump_land_confirm_ms"],
  ["rollCompEnable", "roll_comp_enable"],
  ["rollTarget", "roll_target"],
  ["rollCompGain", "roll_comp_gain"],
  ["rollCompLimit", "roll_comp_limit"],
  ["rollCompRampRate", "roll_comp_ramp_rate"],
  ["kpPhi0", "kp_phi0"],
  ["kdPhi0", "kd_phi0"],
  ["maxPhi0Torque", "max_phi0_torque"],
  ["yawHoldEnable", "yaw_hold_enable"],
  ["yawHoldKp", "yaw_hold_kp"],
  ["yawHoldKd", "yaw_hold_kd"],
  ["yawHoldDeadband", "yaw_hold_deadband"],
  ["yawTargetRateMax", "yaw_target_rate_max"],
  ["yawHoldCurrentLimit", "yaw_hold_current_limit"],
  ["gimbalFollowEnable", "gimbal_follow_enable"],
  ["gimbalFollowKp", "gimbal_follow_kp"],
  ["gimbalFollowKd", "gimbal_follow_kd"],
  ["gimbalFollowDeadband", "gimbal_follow_deadband"],
  ["gimbalFollowCurrentLimit", "gimbal_follow_current_limit"],
  ["gimbalFollowDirection", "gimbal_follow_direction"],
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

function readKalmanParams() {
  return {
    q_v: Number($("kalmanQv").value),
    q_a: Number($("kalmanQa").value),
    r_wheel: Number($("kalmanRWheel").value),
    r_imu: Number($("kalmanRImu").value)
  };
}

function setKalmanParams(params) {
  const values = { ...kalmanDefaults, ...(params || {}) };
  $("kalmanQv").value = values.q_v;
  $("kalmanQa").value = values.q_a;
  $("kalmanRWheel").value = values.r_wheel;
  $("kalmanRImu").value = values.r_imu;
}

function setBalanceParams(params) {
  balanceParamIds.forEach(([id, key]) => {
    if (params[key] !== undefined) {
      $(id).value = params[key];
    }
  });
  // These two calibration values have dedicated controls outside the main
  // parameter list, but are also firmware defaults and should be imported.
  if (params.gimbal_follow_front_ecd !== undefined) {
    $("gimbalFollowFrontEcd").value = params.gimbal_follow_front_ecd;
  }
  if (params.gimbal_follow_front_valid !== undefined) {
    $("gimbalFollowFrontValid").value = params.gimbal_follow_front_valid;
  }
}

function paramsRequestBody() {
  return {
    params: readBalanceParams(),
    dry_run: $("dryRun").checked,
    ...rttTarget()
  };
}

function kalmanRequestBody() {
  return {
    kalman_params: readKalmanParams(),
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
    balanceParams: readBalanceParams(),
    kalmanParams: readKalmanParams(),
    gimbalFollowFrontEcd: Number($("gimbalFollowFrontEcd").value),
    gimbalFollowFrontValid: Number($("gimbalFollowFrontValid").value)
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
  if (state.kalmanParams) {
    setKalmanParams(state.kalmanParams);
  }
  if (state.gimbalFollowFrontEcd !== undefined) {
    $("gimbalFollowFrontEcd").value = state.gimbalFollowFrontEcd;
  }
  if (state.gimbalFollowFrontValid !== undefined) {
    $("gimbalFollowFrontValid").value = state.gimbalFollowFrontValid;
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
  if (Array.isArray(rtt)) {
    rtt.forEach(assertRttOk);
    return;
  }
  if (!rtt || rtt.ok !== false) {
    if (rtt && rtt.target_applied === false) {
      throw new Error(rtt.target_ack || "目标板未确认参数生效");
    }
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

async function getFirmwareDefaults() {
  const res = await fetch(`${$("apiBase").value}/api/firmware-defaults`);
  const data = await res.json();
  if (!res.ok || !data.ok) {
    throw new Error(data.error || `HTTP ${res.status}`);
  }
  return data;
}

async function readFirmwareDefaults() {
  setStatus("读取固件参数中...");
  try {
    const data = await getFirmwareDefaults();
    const params = data.balance_params || {};
    if (
      data.leg_length !== null &&
      data.leg_length !== undefined &&
      Number.isFinite(Number(data.leg_length))
    ) {
      $("legLength").value = data.leg_length;
    }
    if (Array.isArray(data.q) && data.q.length === 6) {
      data.q.forEach((value, index) => {
        $("q" + index).value = value;
      });
    }
    if (Array.isArray(data.r) && data.r.length === 2) {
      data.r.forEach((value, index) => {
        $("r" + index).value = value;
      });
    }
    setBalanceParams(params);
    setKalmanParams(data.kalman_params);
    const missing = data.missing_macros || [];
    const parseErrors = data.parse_errors || [];
    appendLog({
      loaded: true,
      defaults_file: data.defaults_file,
      lqr_file: data.lqr_file,
      parameter_count: Object.keys(params).length,
      q_r_found: Boolean(data.q_r_found),
      missing_macros: missing,
      parse_errors: parseErrors
    });
    if (missing.length || parseErrors.length || !data.q_r_found) {
      const suffix = data.q_r_found ? "部分缺失" : "未找到Q/R注释";
      setStatus(`固件参数已加载（${Object.keys(params).length}项，${suffix}）`, false);
    } else {
      setStatus(`固件参数已加载（${Object.keys(params).length}项）`);
    }
  } catch (error) {
    appendLog(error.message);
    setStatus("读取固件参数失败", false);
  }
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
    assertRttOk(data.rtt_results || data.rtt);
    appendLog(data.rtt_results || data.rtt);
    setStatus("参数已下发");
  } catch (error) {
    appendLog(error.message);
    setStatus("参数下发失败", false);
  }
}

async function sendKalmanParams(silent = false) {
  if (!silent) setStatus("卡尔曼参数下发中...");
  try {
    const data = await post("/api/send-kalman-params", kalmanRequestBody());
    $("payload").value = data.kalman_payload;
    assertRttOk(data.rtt);
    appendLog(data.rtt);
    setStatus("卡尔曼参数已下发");
  } catch (error) {
    appendLog(error.message);
    setStatus("卡尔曼参数下发失败", false);
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

async function sendFollowCalibration(payloadFields, successText) {
  setStatus("云台正前方标定下发中...");
  try {
    const payload = JSON.stringify({
      type: "vmc_balance_params",
      ...payloadFields
    });
    const data = await post("/api/send-rtt", {
      payload,
      dry_run: $("dryRun").checked,
      ...rttTarget()
    });
    $("payload").value = payload;
    assertRttOk(data.rtt);
    appendLog(data.rtt);
    setStatus(successText);
  } catch (error) {
    appendLog(error.message);
    setStatus("云台正前方标定失败", false);
  }
}

async function captureGimbalFront() {
  await sendFollowCalibration(
    { gimbal_follow_capture_front: 1 },
    "已捕获当前云台正前方"
  );
}

async function sendGimbalFrontEcd() {
  const frontEcd = Number($("gimbalFollowFrontEcd").value);
  if (!Number.isInteger(frontEcd) || frontEcd < 0 || frontEcd > 8191) {
    setStatus("正前方编码值必须为 0～8191", false);
    return;
  }
  await sendFollowCalibration(
    {
      gimbal_follow_front_ecd: frontEcd,
      gimbal_follow_front_valid: Number($("gimbalFollowFrontValid").value) > 0.5 ? 1 : 0
    },
    "正前方编码值已下发"
  );
}

async function writeFirmwareDefaults() {
  setStatus("写入固件默认值中...");
  try {
    const data = await post("/api/save-firmware-defaults", {
      ...requestBody(false),
      params: {
        ...readBalanceParams(),
        gimbal_follow_front_ecd: Number($("gimbalFollowFrontEcd").value),
        gimbal_follow_front_valid: Number($("gimbalFollowFrontValid").value)
      }
    });
    appendLog({
      saved: true,
      lqr_file: data.lqr_file,
      defaults_file: data.defaults_file,
      changed_macros: data.changed_macros.length
    });
    setStatus("固件默认值已写入");
  } catch (error) {
    appendLog(error.message);
    setStatus("写入固件默认值失败", false);
  }
}

function scheduleLiveParams() {
  if (!$("liveParams").checked) {
    return;
  }
  clearTimeout(liveParamsTimer);
  liveParamsTimer = setTimeout(() => sendBalanceParams(true), 250);
}

function scheduleLiveKalmanParams() {
  if (!$("liveParams").checked) return;
  clearTimeout(liveParamsTimer);
  liveParamsTimer = setTimeout(() => sendKalmanParams(true), 250);
}

async function init() {
  let restored = false;
  try {
    const defaults = await getDefaults();
    $("legLength").value = defaults.leg_length;
    $("rttHost").value = defaults.rtt_host;
    $("rttPort").value = defaults.rtt_port;
    setBalanceParams(defaults.balance_params);
    setKalmanParams(defaults.kalman_params);
    renderWeights($("qWeights"), defaults.state_order, defaults.q, "q");
    renderWeights($("rWeights"), defaults.input_order, defaults.r, "r");
    setStatus("默认值已加载");
  } catch (error) {
    renderWeights($("qWeights"), stateNames, qDefaults, "q");
    renderWeights($("rWeights"), inputNames, rDefaults, "r");
    setKalmanParams(kalmanDefaults);
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
$("sendKalmanParamsBtn").addEventListener("click", () => sendKalmanParams(false));
$("readFirmwareDefaultsBtn").addEventListener("click", readFirmwareDefaults);
$("captureGimbalFrontBtn").addEventListener("click", captureGimbalFront);
$("sendGimbalFrontBtn").addEventListener("click", sendGimbalFrontEcd);
$("saveParamsBtn").addEventListener("click", saveUiState);
$("writeDefaultsBtn").addEventListener("click", writeFirmwareDefaults);
balanceParamIds.map(([id]) => id).forEach((id) => {
  $(id).addEventListener("input", scheduleLiveParams);
  $(id).addEventListener("change", scheduleLiveParams);
});
["kalmanQv", "kalmanQa", "kalmanRWheel", "kalmanRImu"].forEach((id) => {
  $(id).addEventListener("input", scheduleLiveKalmanParams);
  $(id).addEventListener("change", scheduleLiveKalmanParams);
});

init();
