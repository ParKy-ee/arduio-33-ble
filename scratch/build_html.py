"""Build the ESP32 viewer/recorder from index2.html and its label config."""

from __future__ import annotations

import gzip
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "index2.html"
CONFIG = ROOT / "config" / "pose_labels.json"
OUTPUT = ROOT / "src" / "webpage.h"


CAPTURE_HTML = r"""
    <section id="datasetCapture" class="record-card">
      <div class="record-head"><div><strong>เก็บข้อมูล IMU สำหรับฝึกโมเดล</strong>
        <small>ตรวจตำแหน่งที่ firmware แจ้งก่อนเริ่ม · label เป็น metadata แยกจากรหัสอุปกรณ์</small></div>
        <span id="datasetRecordStatus" class="record-status">พร้อม</span></div>
      <div class="record-controls">
        <label class="record-field">ตำแหน่งติด IMU<select id="datasetMountSite"></select></label>
        <label class="record-field">ส่วนร่างกาย<select id="datasetBodyPart"></select></label>
        <label class="record-field">ท่า<select id="datasetAction"></select></label>
        <label class="record-field">ข้าง<select id="datasetSide"><option value="unknown">ไม่ระบุ</option><option value="left">ซ้าย</option><option value="right">ขวา</option></select></label>
        <label class="record-field">ผู้เข้าร่วม<input id="datasetSubject" value="P01" maxlength="40"></label>
        <label class="record-field">Session<input id="datasetSession" maxlength="60"></label>
        <label class="record-field">Trial<input id="datasetTrial" value="trial-01" maxlength="60"></label>
      </div>
      <div class="record-controls" style="margin-top:10px">
        <button id="datasetStart" class="tool-button primary" type="button">เริ่มบันทึก</button>
        <button id="datasetStop" class="tool-button" type="button" disabled>หยุดและตรวจ label</button>
        <span id="datasetCount" class="record-count">0 samples</span>
        <span id="datasetDevice" class="record-count">รอ ESP</span>
      </div>
      <div class="record-controls" style="margin-top:10px">
        <span class="record-count">ปิดตัวช่วยจับเวลาเพื่อใช้ปุ่มทำเครื่องหมายด้วยมือ:</span>
        <button data-phase="neutral" class="phase-mark" type="button" disabled>ปกติ</button>
        <button data-phase="approaching" class="phase-mark" type="button" disabled>กำลังเข้าท่า</button>
        <button data-phase="holding" class="phase-mark" type="button" disabled>ค้างในท่า</button>
        <button data-phase="returning" class="phase-mark" type="button" disabled>กำลังกลับ</button>
      </div>
      <div class="guided-panel">
        <label class="guided-toggle"><input id="datasetGuidedEnabled" type="checkbox" checked> ใช้ตัวช่วยเตือนเปลี่ยน phase อัตโนมัติ</label>
        <div class="guided-plan-head"><strong>กำหนดลำดับก่อนอัด</strong><span>เลือก phase และเวลาของแต่ละช่วง แล้วจัดลำดับด้วยปุ่ม ↑ / ↓</span></div>
        <div id="guidedStepList" class="guided-step-list"></div>
        <div class="guided-plan-actions">
          <button id="guidedAddStep" class="tool-button" type="button">+ เพิ่มช่วง</button>
          <label class="guided-lead-field">เตือนก่อนเปลี่ยน (วินาที)<input id="guidedLeadTime" type="number" min="0" max="30" step="1" value="2"></label>
        </div>
        <p id="guidedPlanSummary" class="guided-plan-summary"></p>
        <label class="guided-toggle"><input id="datasetVoiceEnabled" type="checkbox" checked> พูดชื่อ phase (ถ้า browser มีเสียงภาษาไทย)</label>
        <div id="guidedProgressLabel" class="guided-progress-label">ยังไม่เริ่มอัด</div>
        <div id="guidedProgressBar" class="guided-progress-track" role="progressbar" aria-label="ความคืบหน้าของ phase ปัจจุบัน" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0"><div id="guidedProgressFill" class="guided-progress-fill"></div></div>
        <div id="guidedOverallProgressLabel" class="guided-progress-label guided-overall-label">ความคืบหน้าทั้งรอบ</div>
        <div id="guidedOverallProgressBar" class="guided-progress-track guided-overall-track" role="progressbar" aria-label="ความคืบหน้ารวมของทุก phase" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0"><div id="guidedOverallProgressFill" class="guided-progress-fill guided-overall-fill"></div></div>
        <div id="guidedCountdown" class="guided-countdown">เลือก phase และเวลา แล้วกดเริ่มบันทึก</div>
        <p class="record-feedback">ตั้งเวลาของแต่ละช่วงได้ 1–120 วินาที กดเริ่มครั้งเดียวแล้วทำตามเสียง/แถบได้ ระบบหยุดเองเมื่อช่วงสุดท้ายครบ จุดเวลาจากตัวช่วยเป็น cue เบื้องต้น ต้องตรวจและยืนยันในตารางก่อนใช้ฝึกโมเดล</p>
      </div>
      <p id="datasetFeedback" class="record-feedback" role="status">เปิดหน้านี้จาก Wi-Fi ของ ESP32; ปรับเวลาเริ่ม phase ได้หลังหยุดบันทึก</p>
      <p class="record-feedback"><a href="/calibrate" class="recording-link">ตั้งท่ากลาง / calibrate IMU</a></p>
      <div id="phaseEditor" class="phase-editor"></div>
      <div id="datasetLinks" class="recording-list" aria-live="polite"></div>
    </section>
"""


CAPTURE_CSS = r"""
    .record-card { margin-top:14px;padding:15px;background:rgba(17,27,37,.94);border:1px solid var(--line);border-radius:13px }
    section.record-card[aria-labelledby="recordTitle"] { display:none }
    .phase-editor { display:grid;gap:6px;margin-top:10px }
    .phase-edit-row { display:grid;grid-template-columns:minmax(90px,.65fr) minmax(130px,1fr) minmax(120px,.8fr) auto;gap:8px;align-items:center }
    .phase-edit-row input:not([type="checkbox"]),.phase-edit-row select { min-height:34px;color:var(--text);background:#0d151e;border:1px solid var(--line);border-radius:8px;padding:5px 8px }
    .phase-edit-row .phase-review { display:flex;gap:6px;align-items:center;font-size:.88em }
    .guided-panel { margin-top:10px;padding:12px;border:1px solid var(--line);border-radius:10px;background:#101c27 }
    .guided-toggle { display:flex;gap:8px;align-items:center;margin:4px 0 10px }
    .guided-plan-head { display:flex;flex-wrap:wrap;gap:5px 12px;align-items:baseline;margin:8px 0;color:var(--muted);font-size:.9em }
    .guided-plan-head strong { color:var(--text) }
    .guided-step-list { display:grid;gap:7px }
    .guided-step-row { display:grid;grid-template-columns:34px minmax(130px,1fr) minmax(120px,.55fr) auto;gap:8px;align-items:center }
    .guided-step-index { color:var(--muted);text-align:center;font-weight:700 }
    .guided-step-row select,.guided-step-row input { min-height:36px;width:100%;box-sizing:border-box;color:var(--text);background:#0d151e;border:1px solid var(--line);border-radius:8px;padding:5px 8px }
    .guided-duration-label { display:flex;gap:6px;align-items:center;color:var(--muted);font-size:.9em }
    .guided-step-actions { display:flex;gap:4px }
    .guided-step-actions button { min-width:36px;min-height:36px;padding:4px 8px;border:1px solid var(--line);border-radius:8px;color:var(--text);background:#213846 }
    .guided-step-actions button:disabled { opacity:.4 }
    .guided-plan-actions { display:flex;flex-wrap:wrap;justify-content:space-between;gap:8px;align-items:end;margin-top:9px }
    .guided-lead-field { display:grid;gap:4px;min-width:150px;color:var(--muted);font-size:.9em }
    .guided-lead-field input { min-height:34px;color:var(--text);background:#0d151e;border:1px solid var(--line);border-radius:8px;padding:5px 8px }
    .guided-plan-summary { margin:9px 0;color:var(--muted);font-size:.9em }
    .guided-progress-label { margin-top:10px;color:var(--text);font-weight:700 }
    .guided-progress-track { height:12px;margin-top:6px;overflow:hidden;border-radius:99px;background:#273747;border:1px solid var(--line) }
    .guided-progress-fill { width:0;height:100%;border-radius:inherit;background:linear-gradient(90deg,#48d3bb,#84e27b);transition:width .12s linear }
    .guided-overall-label { margin-top:9px;font-size:.9em;color:var(--muted);font-weight:600 }
    .guided-overall-track { height:8px;margin-top:4px }
    .guided-overall-fill { background:linear-gradient(90deg,#5488ff,#48d3bb) }
    .guided-countdown { margin-top:10px;padding:10px;border-radius:8px;background:#172b3a;color:#f0f7ff;font-size:1.15em;font-weight:700 }
    .phase-mark { padding:7px 10px;border:1px solid var(--line);border-radius:8px;color:var(--text);background:#213846 }
    .phase-mark:disabled { opacity:.45 }
    #recordTitle { display:none }
    @media(max-width:600px){.phase-edit-row{grid-template-columns:1fr 1fr}.guided-step-row{grid-template-columns:28px minmax(0,1fr) minmax(92px,.65fr)}.guided-step-actions{grid-column:2/-1;justify-content:flex-end}}
"""


LIVE_AND_CAPTURE_SCRIPT = r"""
    window.POSE_LABELS = __POSE_LABELS__;
    const datasetConfig = window.POSE_LABELS;
    const dataset$ = function (id) { return document.getElementById(id); };
    const originalCaptureSample = window.captureLiveSample;
    const originalStartRecording = startRecording;
    let latestLiveData = null;
    let activeDatasetPhase = "neutral";
    let savedRecordings = [];
    let guidedInterval = 0;
    let datasetAudioContext = null;
    const DATASET_PHASE_NAMES = {neutral:"ปกติ",approaching:"กำลังเข้าท่า",holding:"ค้างท่า",returning:"กำลังกลับท่าปกติ"};

    function datasetPhaseName(phase) { return DATASET_PHASE_NAMES[phase] || phase; }

    const DEFAULT_GUIDED_STEPS = [
      {phase:"neutral",seconds:3}, {phase:"approaching",seconds:3},
      {phase:"holding",seconds:5}, {phase:"returning",seconds:3},
      {phase:"neutral",seconds:2}
    ];

    function updateGuidedStepControls() {
      const list = dataset$("guidedStepList");
      const rows = Array.from(list.children);
      rows.forEach(function (row, index) {
        row.querySelector(".guided-step-index").textContent = String(index + 1);
        row.querySelector(".guided-step-up").disabled = index === 0;
        row.querySelector(".guided-step-down").disabled = index === rows.length - 1;
        row.querySelector(".guided-step-remove").disabled = rows.length <= 1;
      });
      const names = rows.map(function (row) { return datasetPhaseName(row.querySelector(".guided-phase-select").value); });
      const durations = rows.map(function (row) { return Number(row.querySelector(".guided-duration").value); });
      const total = durations.every(Number.isFinite) ? durations.reduce(function (sum, value) { return sum + value; }, 0) : NaN;
      dataset$("guidedPlanSummary").textContent = names.length
        ? "ลำดับที่จะอัด: " + names.join(" → ") + (Number.isFinite(total) ? " · รวม " + total + " วินาที" : "")
        : "เพิ่ม phase อย่างน้อยหนึ่งช่วงก่อนเริ่มอัด";
    }

    function addGuidedStep(phase, seconds) {
      const list = dataset$("guidedStepList");
      const options = datasetConfig.phases || Object.keys(DATASET_PHASE_NAMES);
      const row = document.createElement("div"); row.className = "guided-step-row";
      const number = document.createElement("span"); number.className = "guided-step-index";
      const select = document.createElement("select"); select.className = "guided-phase-select"; select.setAttribute("aria-label", "เลือก phase");
      options.forEach(function (value) { const option = document.createElement("option"); option.value = value; option.textContent = datasetPhaseName(value); select.appendChild(option); });
      select.value = options.indexOf(phase) >= 0 ? phase : (options[0] || "neutral");
      const durationLabel = document.createElement("label"); durationLabel.className = "guided-duration-label";
      const duration = document.createElement("input"); duration.className = "guided-duration"; duration.type = "number"; duration.min = "1"; duration.max = "120"; duration.step = "0.5"; duration.value = String(seconds); duration.setAttribute("aria-label", "ระยะเวลาเป็นวินาที");
      const secondsText = document.createElement("span"); secondsText.textContent = "วินาที"; durationLabel.append(duration, secondsText);
      const actions = document.createElement("div"); actions.className = "guided-step-actions";
      const makeAction = function (label, title, className) { const button = document.createElement("button"); button.type = "button"; button.textContent = label; button.title = title; button.setAttribute("aria-label", title); button.className = className; actions.appendChild(button); return button; };
      const up = makeAction("↑", "เลื่อน phase ขึ้น", "guided-step-up");
      const down = makeAction("↓", "เลื่อน phase ลง", "guided-step-down");
      const remove = makeAction("ลบ", "ลบ phase นี้", "guided-step-remove");
      up.addEventListener("click", function () { const index = Array.from(list.children).indexOf(row); if (index > 0) list.insertBefore(row, list.children[index - 1]); updateGuidedStepControls(); });
      down.addEventListener("click", function () { const index = Array.from(list.children).indexOf(row); if (index >= 0 && index < list.children.length - 1) list.insertBefore(list.children[index + 1], row); updateGuidedStepControls(); });
      remove.addEventListener("click", function () { row.remove(); updateGuidedStepControls(); });
      select.addEventListener("change", updateGuidedStepControls); duration.addEventListener("input", updateGuidedStepControls);
      row.append(number, select, durationLabel, actions); list.appendChild(row); updateGuidedStepControls();
    }

    function renderGuidedSteps(steps) {
      const list = dataset$("guidedStepList"); list.innerHTML = "";
      (steps || DEFAULT_GUIDED_STEPS).forEach(function (step) { addGuidedStep(step.phase, step.seconds); });
      updateGuidedStepControls();
    }

    function appendGuidedStep() {
      const rows = Array.from(dataset$("guidedStepList").children);
      const phases = datasetConfig.phases || Object.keys(DATASET_PHASE_NAMES);
      const last = rows.length ? rows[rows.length - 1].querySelector(".guided-phase-select").value : "neutral";
      const lastIndex = phases.indexOf(last);
      const next = lastIndex >= 0 ? phases[(lastIndex + 1) % phases.length] : (phases[0] || "neutral");
      addGuidedStep(next, 3);
    }

    function fillDatasetLabels() {
      const site = dataset$("datasetMountSite");
      Object.entries(datasetConfig.mount_sites).forEach(function (entry) {
        const option = document.createElement("option"); option.value = entry[0]; option.textContent = entry[1].label + " (" + entry[0] + ")"; site.appendChild(option);
      });
      const body = dataset$("datasetBodyPart");
      Object.entries(datasetConfig.body_parts).forEach(function (entry) {
        const option = document.createElement("option"); option.value = entry[0]; option.textContent = entry[1].label; body.appendChild(option);
      });
      dataset$("datasetSession").value = "session-" + new Date().toISOString().replace(/[-:.TZ]/g, "").slice(0, 14);
      function refreshActions() {
        const item = datasetConfig.body_parts[body.value]; const select = dataset$("datasetAction"); select.innerHTML = "";
        (item.actions || []).forEach(function (action) { const option = document.createElement("option"); option.value = action.id; option.textContent = action.label + " (" + action.id + ")"; select.appendChild(option); });
        select.disabled = !item.actions || !item.actions.length;
        if (select.disabled) { const option = document.createElement("option"); option.value = ""; option.textContent = "เพิ่มท่าใน config/pose_labels.json ก่อน"; select.appendChild(option); }
      }
      body.addEventListener("change", refreshActions); refreshActions();
    }

    function setDatasetStatus(text, state) {
      const item = dataset$("datasetRecordStatus"); item.textContent = text;
      item.className = "record-status" + (state ? " " + state : "");
    }

    function setDatasetPhaseButtons(enabled) {
      document.querySelectorAll(".phase-mark").forEach(function (button) { button.disabled = !enabled; });
    }

    function playDatasetCueTone(kind) {
      try {
        const AudioContextType = window.AudioContext || window.webkitAudioContext;
        if (!AudioContextType) return;
        if (!datasetAudioContext) datasetAudioContext = new AudioContextType();
        if (datasetAudioContext.state === "suspended") datasetAudioContext.resume();
        const frequencies = kind === "change" ? [740, 980] : [620];
        frequencies.forEach(function (frequency, index) {
          const oscillator = datasetAudioContext.createOscillator();
          const gain = datasetAudioContext.createGain();
          const start = datasetAudioContext.currentTime + index * 0.16;
          oscillator.type = "sine"; oscillator.frequency.value = frequency;
          gain.gain.setValueAtTime(0.0001, start); gain.gain.exponentialRampToValueAtTime(0.12, start + 0.015);
          gain.gain.exponentialRampToValueAtTime(0.0001, start + 0.13);
          oscillator.connect(gain); gain.connect(datasetAudioContext.destination);
          oscillator.start(start); oscillator.stop(start + 0.14);
        });
      } catch (error) { /* The visible countdown still works when audio is unavailable. */ }
    }

    function announceDatasetCue(message, kind) {
      const voiceEnabled = dataset$("datasetVoiceEnabled").checked;
      const synth = window.speechSynthesis;
      const Utterance = window.SpeechSynthesisUtterance;
      const thaiVoice = voiceEnabled && synth && Utterance && synth.getVoices().find(function (voice) { return /^th(-|_)/i.test(voice.lang); });
      if (thaiVoice) {
        try {
          synth.cancel();
          const utterance = new Utterance(message); utterance.lang = "th-TH"; utterance.voice = thaiVoice; utterance.rate = 1.05;
          synth.speak(utterance);
          if (kind === "change") playDatasetCueTone("change");
          return;
        } catch (error) { /* Use the short tone fallback. */ }
      }
      playDatasetCueTone(kind);
    }

    function readGuidedSchedule() {
      const rows = Array.from(dataset$("guidedStepList").querySelectorAll(".guided-step-row"));
      if (!rows.length) throw new Error("เพิ่ม phase อย่างน้อยหนึ่งช่วงก่อนเริ่มอัด");
      const validPhases = new Set(datasetConfig.phases || Object.keys(DATASET_PHASE_NAMES));
      const schedule = rows.map(function (row) {
        const phase = row.querySelector(".guided-phase-select").value;
        const value = Number(row.querySelector(".guided-duration").value);
        if (!validPhases.has(phase)) throw new Error("เลือก phase ที่มีใน configuration");
        if (!Number.isFinite(value) || value < 1 || value > 120) throw new Error("เวลาแต่ละ phase ต้องอยู่ระหว่าง 1 ถึง 120 วินาที");
        return {phase:phase,duration_ms:Math.round(value * 1000)};
      });
      const lead = Number(dataset$("guidedLeadTime").value);
      if (!Number.isFinite(lead) || lead < 0 || lead > 30) throw new Error("เวลาเตือนต้องอยู่ระหว่าง 0 ถึง 30 วินาที");
      let cursor = 0;
      schedule.forEach(function (segment) { segment.start_ms = cursor; cursor += segment.duration_ms; segment.end_ms = cursor; });
      return {segments:schedule,lead_ms:Math.round(lead * 1000),total_ms:cursor};
    }

    function setGuidedInputsDisabled(disabled) {
      ["datasetGuidedEnabled","datasetVoiceEnabled","guidedLeadTime","guidedAddStep"].forEach(function (id) { dataset$(id).disabled = disabled; });
      dataset$("guidedStepList").querySelectorAll("select,input,button").forEach(function (control) { control.disabled = disabled; });
      if (!disabled) updateGuidedStepControls();
    }

    function formatGuidedTime(milliseconds, roundUp) {
      const seconds = Math.max(0, (roundUp ? Math.ceil(milliseconds / 1000) : Math.floor(milliseconds / 1000)));
      return String(Math.floor(seconds / 60)).padStart(2, "0") + ":" + String(seconds % 60).padStart(2, "0");
    }

    function syncGuidedPhase(recording, elapsed) {
      const guide = recording && recording.guidedSchedule;
      if (!guide || !activeRecording || activeRecording.id !== recording.id) return -1;
      const segments = Array.isArray(guide.segments) ? guide.segments : [];
      if (!segments.length) {
        const message = "Phase schedule is empty; recording stopped.";
        dataset$("datasetFeedback").textContent = message;
        finishDatasetRecording(message);
        return -1;
      }
      const index = segments.findIndex(function (segment) {
        return segment && Number.isFinite(segment.end_ms) && elapsed < segment.end_ms;
      });
      if (index < 0) return -1;
      const segment = segments[index];
      if (!segment || typeof segment.phase !== "string" || !Number.isFinite(segment.start_ms) || !Number.isFinite(segment.duration_ms) || segment.duration_ms <= 0) {
        const message = "Invalid phase schedule at phase " + (index + 1) + "; recording stopped.";
        dataset$("datasetFeedback").textContent = message;
        finishDatasetRecording(message);
        return -1;
      }
      if (index !== recording.guidedEnteredIndex) {
        for (let crossed = recording.guidedEnteredIndex + 1; crossed <= index; crossed++) {
          const crossedSegment = segments[crossed];
          if (!crossedSegment || typeof crossedSegment.phase !== "string") {
            const message = "Missing phase " + (crossed + 1) + " in the schedule; recording stopped.";
            dataset$("datasetFeedback").textContent = message;
            finishDatasetRecording(message);
            return -1;
          }
          addDatasetPhaseEvent(crossedSegment.phase, "guided_timer_cue", crossedSegment.start_ms);
        }
        recording.guidedEnteredIndex = index;
        activeDatasetPhase = segment.phase;
        announceDatasetCue("เปลี่ยนเป็น" + datasetPhaseName(activeDatasetPhase), "change");
      }
      const next = segments[index + 1];
      const remaining = Math.max(0, segment.end_ms - elapsed);
      if (next && guide.lead_ms > 0 && remaining <= guide.lead_ms && !recording.guidedWarnings.has(index)) {
        recording.guidedWarnings.add(index);
        const seconds = Math.max(1, Math.ceil(remaining / 1000));
        announceDatasetCue("อีก " + seconds + " วินาที เปลี่ยนเป็น" + datasetPhaseName(next.phase), "warning");
      }
      const segmentElapsed = Math.max(0, elapsed - segment.start_ms);
      const phaseProgress = Math.min(100, segmentElapsed / segment.duration_ms * 100);
      const overallProgress = Math.min(100, elapsed / guide.total_ms * 100);
      dataset$("guidedProgressLabel").textContent = "ช่วง " + (index + 1) + "/" + guide.segments.length + " · " + datasetPhaseName(segment.phase);
      dataset$("guidedProgressBar").setAttribute("aria-valuenow", String(Math.round(phaseProgress)));
      dataset$("guidedProgressFill").style.width = phaseProgress.toFixed(1) + "%";
      dataset$("guidedOverallProgressLabel").textContent = "ความคืบหน้าทั้งรอบ: " + formatGuidedTime(elapsed, false) + " / " + formatGuidedTime(guide.total_ms, true) + " · phase " + (index + 1) + "/" + guide.segments.length;
      dataset$("guidedOverallProgressBar").setAttribute("aria-valuenow", String(Math.round(overallProgress)));
      dataset$("guidedOverallProgressFill").style.width = overallProgress.toFixed(1) + "%";
      dataset$("guidedCountdown").textContent = "ผ่านไป " + formatGuidedTime(segmentElapsed, false) + " / " + formatGuidedTime(segment.duration_ms, true) + " · เหลือ " + formatGuidedTime(remaining, true) + (next ? " · ต่อไป: " + datasetPhaseName(next.phase) : " · ช่วงสุดท้าย");
      return index;
    }

    function startGuidedPhaseTimer(recording, guide) {
      if (guidedInterval) clearInterval(guidedInterval);
      guidedInterval = 0;
      let thisGuidedInterval = 0;
      const output = dataset$("guidedCountdown");
      const progressLabel = dataset$("guidedProgressLabel");
      const progressBar = dataset$("guidedProgressBar");
      const progressFill = dataset$("guidedProgressFill");
      const overallLabel = dataset$("guidedOverallProgressLabel");
      const overallBar = dataset$("guidedOverallProgressBar");
      const overallFill = dataset$("guidedOverallProgressFill");
      if (!guide) {
        output.textContent = "โหมดทำเครื่องหมาย phase ด้วยมือ";
        progressLabel.textContent = "ปิดตัวช่วยจับเวลา"; progressFill.style.width = "0%"; progressBar.setAttribute("aria-valuenow", "0");
        overallLabel.textContent = "ความคืบหน้าทั้งรอบ: โหมดทำเครื่องหมายด้วยมือ"; overallFill.style.width = "0%"; overallBar.setAttribute("aria-valuenow", "0");
        return;
      }
      recording.guidedSchedule = guide;
      recording.guidedEnteredIndex = 0;
      recording.guidedWarnings = new Set();
      announceDatasetCue("เริ่มช่วง" + datasetPhaseName(guide.segments[0].phase), "start");
      const tick = function () {
        if (!activeRecording || activeRecording.id !== recording.id) {
          if (thisGuidedInterval) clearInterval(thisGuidedInterval);
          if (guidedInterval === thisGuidedInterval) guidedInterval = 0;
          return;
        }
        const elapsed = Math.max(0, Date.now() - recording.startedEpochMs);
        if (elapsed >= guide.total_ms) {
          syncGuidedPhase(recording, Math.max(0, guide.total_ms - 1));
          output.textContent = "ครบลำดับ phase"; progressLabel.textContent = "จบรอบตามแผน";
          progressFill.style.width = "100%"; progressBar.setAttribute("aria-valuenow", "100");
          overallLabel.textContent = "ความคืบหน้าทั้งรอบ: ครบแล้ว"; overallFill.style.width = "100%"; overallBar.setAttribute("aria-valuenow", "100");
          finishDatasetRecording("ครบลำดับ phase ตามตัวช่วย");
          return;
        }
        const index = syncGuidedPhase(recording, elapsed);
        if (index < 0) return;
      };
      tick();
      if (activeRecording && activeRecording.id === recording.id) {
        thisGuidedInterval = setInterval(tick, 100);
        guidedInterval = thisGuidedInterval;
      }
    }

    function startDatasetRecording() {
      if (!latestLiveData || latestLiveData.valid !== true) { dataset$("datasetFeedback").textContent = "ยังไม่ได้รับข้อมูล IMU ที่ใช้ได้"; return; }
      const configuredSite = dataset$("datasetMountSite").value;
      const deviceSite = latestLiveData.mount_site || latestLiveData.sensor_id;
      if (deviceSite && configuredSite !== deviceSite) {
        dataset$("datasetFeedback").textContent = "ตำแหน่งที่เลือกไม่ตรงกับ firmware (" + deviceSite + "); เลือก build ที่ตรงตำแหน่งแล้ว upload ก่อน"; return;
      }
      if (!dataset$("datasetAction").value) { dataset$("datasetFeedback").textContent = "ตำแหน่งร่างกายนี้ยังไม่มี action ใน config"; return; }
      const guidedEnabled = dataset$("datasetGuidedEnabled").checked;
      let guide = null;
      if (guidedEnabled) {
        try { guide = readGuidedSchedule(); }
        catch (error) { dataset$("datasetFeedback").textContent = error.message; return; }
      }
      recordLocation.value = configuredSite;
      recordLabel.value = dataset$("datasetAction").value;
      originalStartRecording();
      if (!activeRecording) return;
      activeRecording.sensorUid = latestLiveData.sensor_uid || latestLiveData.sensor_id;
      activeRecording.firmwareSensorId = latestLiveData.sensor_id || "unknown";
      activeRecording.mountSite = deviceSite || configuredSite;
      activeRecording.bodyPart = dataset$("datasetBodyPart").value;
      activeRecording.side = dataset$("datasetSide").value;
      activeRecording.subjectId = dataset$("datasetSubject").value.trim() || "unknown";
      activeRecording.sessionId = dataset$("datasetSession").value.trim();
      activeRecording.trialId = dataset$("datasetTrial").value.trim() || activeRecording.id;
      activeRecording.actionLabel = dataset$("datasetAction").value;
      activeRecording.calibrationId = latestLiveData.calibration_id || "";
      activeRecording.streamEpoch = 0;
      activeRecording.lastBootMs = null;
      activeRecording.lastSeq = null;
      activeRecording.gapCount = 0;
      const firstPhase = guide ? guide.segments[0].phase : "neutral";
      activeRecording.phaseEvents = [{at_ms: 0, phase: firstPhase, source: guide ? "guided_timer_cue" : "recording_start_unreviewed", reviewed: false}];
      activeRecording.startedEpochMs = Date.now();
      activeDatasetPhase = firstPhase;
      setGuidedInputsDisabled(true);
      dataset$("datasetMountSite").disabled = dataset$("datasetBodyPart").disabled = dataset$("datasetAction").disabled = true;
      dataset$("datasetSide").disabled = dataset$("datasetSubject").disabled = dataset$("datasetSession").disabled = dataset$("datasetTrial").disabled = true;
      dataset$("datasetStart").disabled = true; dataset$("datasetStop").disabled = false;
      setDatasetPhaseButtons(!guide); setDatasetStatus("กำลังบันทึก", "recording");
      dataset$("datasetFeedback").textContent = guide ? "เริ่มแล้ว ทำตามเสียงเตือนและแถบความคืบหน้า; จุด cue เป็นเวลาโดยประมาณ ตรวจแก้ก่อนใช้ฝึก" : "เริ่มแล้ว ใช้ปุ่ม phase ด้วยมือได้; จุดที่กดเป็น cue โดยประมาณ ตรวจแก้ก่อนใช้ฝึก";
      dataset$("datasetCount").textContent = "0 samples";
      startGuidedPhaseTimer(activeRecording, guide);
    }

    function editAnnotations(recording) {
      const editor = dataset$("phaseEditor"); editor.innerHTML = "";
      if (!recording || !recording.phaseEvents.length) return;
      const title = document.createElement("p"); title.className = "record-feedback"; title.textContent = "ตรวจ/ปรับเวลาเริ่มของแต่ละ phase (ms จากเริ่ม recording); cue จากเสียงและปุ่มมือยังไม่ใช่ ground truth ต้องติ๊กตรวจแล้วก่อนให้ phase นี้เข้า train"; editor.appendChild(title);
      const confirmAll = document.createElement("button"); confirmAll.type = "button"; confirmAll.className = "tool-button"; confirmAll.textContent = "ยืนยันทุกช่วงหลังตรวจ";
      confirmAll.addEventListener("click", function () { recording.phaseEvents.forEach(function (event) { event.reviewed = true; }); editAnnotations(recording); }); editor.appendChild(confirmAll);
      recording.phaseEvents.forEach(function (event, index) {
        const line = document.createElement("div"); line.className = "phase-edit-row";
        const time = document.createElement("input"); time.type = "number"; time.min = "0"; time.max = String(Math.max(0, recording.durationMs)); time.value = String(Math.round(event.at_ms)); time.setAttribute("aria-label", "เวลาเริ่ม phase ms");
        const phase = document.createElement("select");
        (datasetConfig.phases || Object.keys(DATASET_PHASE_NAMES)).forEach(function (value) { const option = document.createElement("option"); option.value = value; option.textContent = datasetPhaseName(value); phase.appendChild(option); });
        phase.value = event.phase;
        const reviewLabel = document.createElement("label"); reviewLabel.className = "phase-review";
        const reviewed = document.createElement("input"); reviewed.type = "checkbox"; reviewed.checked = event.reviewed === true;
        const reviewText = document.createElement("span"); reviewText.textContent = "ตรวจแล้ว/ใช้ฝึก"; reviewLabel.append(reviewed, reviewText);
        const remove = document.createElement("button"); remove.type = "button"; remove.textContent = "ลบช่วง"; remove.disabled = index === 0;
        time.addEventListener("change", function () { event.at_ms = Math.max(0, Math.min(recording.durationMs, Number(time.value) || 0)); event.reviewed = false; reviewed.checked = false; refreshRecordingCsv(recording); });
        phase.addEventListener("change", function () { event.phase = phase.value; event.reviewed = false; reviewed.checked = false; refreshRecordingCsv(recording); });
        reviewed.addEventListener("change", function () { event.reviewed = reviewed.checked; refreshRecordingCsv(recording); });
        remove.addEventListener("click", function () { recording.phaseEvents.splice(index, 1); editAnnotations(recording); refreshRecordingCsv(recording); });
        line.append(time, phase, reviewLabel, remove); editor.appendChild(line);
      });
      refreshRecordingCsv(recording);
    }

    function csvValue(value) { const text = value == null ? "" : String(value); return /[\",\r\n]/.test(text) ? '"' + text.replace(/"/g, '""') + '"' : text; }
    const DATASET_COLUMNS = ["sensor_uid","firmware_sensor_id","mount_site","body_part","side","subject_id","session_id","trial_id","action_label","phase_label","phase_elapsed_ms","phase_annotation_source","phase_annotation_reviewed","calibration_id","calibration_method_version","calibration_time_boot_ms","calibration_boot_id","calibrated","relative_angle_valid","relative_orientation_valid","unit_accel","unit_gyro","host_time_iso","host_time_s","host_elapsed_ms","boot_ms","seq","stream_epoch","gap_before","accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","corrected_gyro_x","corrected_gyro_y","corrected_gyro_z","roll_deg","pitch_deg","relative_roll_deg","relative_pitch_deg","relative_tilt_deg","accel_norm_g","gyro_norm_dps","steady","valid","firmware_status"];

    function applyPhaseLabels(recording) {
      const events = recording.phaseEvents.slice().sort(function (a,b) { return a.at_ms-b.at_ms; });
      recording.rows.forEach(function (row) {
        let current = events[0] || {at_ms:0,phase:"neutral",source:"recording_start_unreviewed",reviewed:false};
        for (const event of events) { if (row.host_elapsed_ms >= event.at_ms) current = event; else break; }
        row.phase_label = current.phase; row.phase_elapsed_ms = row.host_elapsed_ms - current.at_ms;
        row.phase_annotation_source = current.source || "manual_button_cue";
        row.phase_annotation_reviewed = current.reviewed === true;
      });
    }

    function initializeModelTestNavigation() {
      const input = document.getElementById("modelTestServer");
      const button = document.getElementById("openModelTest");
      if (!input || !button) return;
      try {
        const saved = localStorage.getItem("imuModelTestUrlV1");
        if (saved) input.value = saved;
      } catch (error) { /* Storage can be unavailable in a restricted browser. */ }
      button.addEventListener("click", function () {
        let value = input.value.trim();
        if (!value) { alert("ใส่ที่อยู่ Model Test ก่อน"); input.focus(); return; }
        if (!/^https?:\/\//i.test(value)) value = "http://" + value;
        let destination;
        try { destination = new URL(value); }
        catch (error) { alert("ที่อยู่ Model Test ไม่ถูกต้อง"); input.focus(); return; }
        if (destination.protocol !== "http:" && destination.protocol !== "https:") {
          alert("ใช้ที่อยู่ที่ขึ้นต้นด้วย http:// หรือ https://"); input.focus(); return;
        }
        try { localStorage.setItem("imuModelTestUrlV1", destination.href); } catch (error) { /* Optional preference. */ }
        window.open(destination.href, "_blank", "noopener,noreferrer");
      });
    }

    function refreshRecordingCsv(recording) {
      applyPhaseLabels(recording);
      const content = "\uFEFF" + [DATASET_COLUMNS.join(",")].concat(recording.rows.map(function (row) { return DATASET_COLUMNS.map(function (key) { return csvValue(row[key]); }).join(","); })).join("\r\n");
      if (recording.url) URL.revokeObjectURL(recording.url);
      recording.url = URL.createObjectURL(new Blob([content], {type:"text/csv;charset=utf-8"}));
      if (recording.anchor) recording.anchor.href = recording.url;
    }

    function finishDatasetRecording(reason) {
      if (!activeRecording) return;
      if (guidedInterval) clearInterval(guidedInterval);
      guidedInterval = 0;
      const completed = activeRecording; activeRecording = null;
      completed.durationMs = completed.rows.length ? Math.max(...completed.rows.map(function (row) { return row.host_elapsed_ms; })) : 0;
      editAnnotations(completed);
      const anchor = document.createElement("a"); anchor.className = "recording-link";
      const safe = function (value) { return String(value).replace(/[^a-z0-9_-]+/gi,"_"); };
      anchor.download = safe(completed.trialId) + "_" + safe(completed.mountSite) + "_" + safe(completed.actionLabel) + ".csv";
      anchor.textContent = anchor.download + " · " + completed.rows.length.toLocaleString() + " samples · ดาวน์โหลด";
      dataset$("datasetLinks").prepend(anchor); completed.anchor = anchor; savedRecordings.push(completed); refreshRecordingCsv(completed);
      dataset$("datasetStop").disabled = true; dataset$("datasetStart").disabled = false;
      setGuidedInputsDisabled(false);
      const completedAsPlanned = reason && reason.indexOf("ครบลำดับ phase") >= 0;
      dataset$("guidedCountdown").textContent = completedAsPlanned ? "ครบลำดับ phase" : "หยุดการบันทึกแล้ว";
      if (!completedAsPlanned) dataset$("guidedProgressLabel").textContent = "หยุดการบันทึก";
      if (!completedAsPlanned) dataset$("guidedOverallProgressLabel").textContent = "ความคืบหน้าทั้งรอบ: หยุดก่อนครบ";
      ["datasetMountSite","datasetBodyPart","datasetAction","datasetSide","datasetSubject","datasetSession","datasetTrial"].forEach(function (id) { dataset$(id).disabled = false; });
      setDatasetPhaseButtons(false); setDatasetStatus("หยุดแล้ว", "done");
      dataset$("datasetFeedback").textContent = (reason ? reason + " · " : "") + "ตรวจแก้ช่วง phase ได้ก่อนกดดาวน์โหลด CSV";
    }

    function addDatasetPhaseEvent(phase, source, elapsed) {
      if (!activeRecording) return;
      const atMs = Math.max(0, Number.isFinite(elapsed) ? elapsed : Date.now() - activeRecording.startedEpochMs);
      const last = activeRecording.phaseEvents[activeRecording.phaseEvents.length-1];
      if (last && last.phase === phase && source !== "guided_timer_cue") return;
      if (last && last.at_ms === atMs) return;
      activeRecording.phaseEvents.push({at_ms:atMs,phase:phase,source:source,reviewed:false});
      activeDatasetPhase = phase;
    }

    function markDatasetPhase(phase) {
      if (!activeRecording) return;
      const elapsed = Math.max(0, Date.now() - activeRecording.startedEpochMs);
      if (activeRecording.phaseEvents.length && activeRecording.phaseEvents[activeRecording.phaseEvents.length-1].phase === phase) return;
      addDatasetPhaseEvent(phase, "manual_button_cue", elapsed);
      dataset$("datasetFeedback").textContent = "ทำเครื่องหมาย " + phase + " ที่ " + elapsed + " ms; เป็น cue ที่ต้องตรวจหลังหยุด";
    }

    window.captureLiveSample = function (data) {
      latestLiveData = data;
      if (!activeRecording || !data || data.valid !== true) return originalCaptureSample(data);
      const key = String(data.sample_ms) + ":" + String(data.seq);
      if (key === activeRecording.lastSampleKey) return;
      const boot = Number(data.sample_ms), seq = Number(data.seq);
      if (activeRecording.rows.length && String(data.calibration_id || "") !== activeRecording.calibrationId) {
        const old = activeRecording;
        finishDatasetRecording("calibration เปลี่ยน; แบ่ง recording ใหม่เพื่อไม่ให้ข้าม baseline");
        dataset$("datasetTrial").value = old.trialId + "_cal" + (savedRecordings.length + 1);
        startDatasetRecording();
        if (!activeRecording) return;
      }
      if (activeRecording.guidedSchedule) {
        const guidedElapsed = Math.max(0, Date.now() - activeRecording.startedEpochMs);
        const lastPlannedMs = Math.max(0, activeRecording.guidedSchedule.total_ms - 1);
        const syncIndex = syncGuidedPhase(activeRecording, Math.min(guidedElapsed, lastPlannedMs));
        if (syncIndex < 0 || !activeRecording) return;
      }
      const countBefore = activeRecording.rows.length;
      originalCaptureSample(data);
      if (activeRecording.rows.length === countBefore) return;
      const row = activeRecording.rows[activeRecording.rows.length - 1];
      if (activeRecording.lastSeq !== null && (seq <= activeRecording.lastSeq || boot < activeRecording.lastBootMs)) activeRecording.streamEpoch++;
      const gapBefore = activeRecording.lastSeq !== null && seq !== activeRecording.lastSeq + 1;
      if (gapBefore) activeRecording.gapCount++;
      activeRecording.lastSeq = seq; activeRecording.lastBootMs = boot; activeRecording.lastSampleKey = key;
      const hostMs = Date.now();
      Object.assign(row, {
        sensor_uid: data.sensor_uid || data.sensor_id || activeRecording.sensorUid,
        firmware_sensor_id: data.sensor_id || activeRecording.firmwareSensorId,
        mount_site: data.mount_site || activeRecording.mountSite,
        body_part: activeRecording.bodyPart, side: activeRecording.side,
        subject_id: activeRecording.subjectId, session_id: activeRecording.sessionId,
        trial_id: activeRecording.trialId, action_label: activeRecording.actionLabel,
        phase_label: activeDatasetPhase, phase_annotation_source: "pending", phase_annotation_reviewed: false, calibration_id: data.calibration_id || "",
        calibration_method_version: data.calibration_method_version || "",
        calibration_time_boot_ms: data.calibration_time_boot_ms || "",
        calibration_boot_id: data.calibration_boot_id || "",
        calibrated: data.calibrated === true, relative_angle_valid: data.relative_angle_valid === true,
        relative_orientation_valid: data.relative_orientation_valid === true,
        unit_accel: "m/s^2", unit_gyro: "rad/s",
        host_time_iso: new Date(hostMs).toISOString(), host_time_s: (hostMs/1000).toFixed(6),
        host_elapsed_ms: hostMs-activeRecording.startedEpochMs, boot_ms: boot, seq: seq,
        stream_epoch: activeRecording.streamEpoch, gap_before: gapBefore,
        accel_x: Number(data.ax)*9.80665, accel_y: Number(data.ay)*9.80665, accel_z: Number(data.az)*9.80665,
        gyro_x: data.gx, gyro_y: data.gy, gyro_z: data.gz,
        corrected_gyro_x: data.gx_corrected, corrected_gyro_y: data.gy_corrected, corrected_gyro_z: data.gz_corrected,
        roll_deg: data.roll_deg, pitch_deg: data.pitch_deg,
        relative_roll_deg: data.relative_roll_deg, relative_pitch_deg: data.relative_pitch_deg,
        relative_tilt_deg: data.wrist_flex_deg, accel_norm_g: data.accel_norm_g,
        gyro_norm_dps: data.gyro_norm_dps, steady: data.steady, valid: data.valid,
        firmware_status: data.status
      });
      dataset$("datasetCount").textContent = activeRecording.rows.length.toLocaleString() + " samples · gaps " + activeRecording.gapCount;
    };

    document.querySelectorAll(".phase-mark").forEach(function (button) { button.addEventListener("click", function () { markDatasetPhase(button.dataset.phase); }); });
    dataset$("datasetStart").addEventListener("click", startDatasetRecording);
    dataset$("datasetStop").addEventListener("click", function () { finishDatasetRecording(""); });
    dataset$("recordingsList").style.display = "none";
    dataset$("recordTitle").style.display = "none";
    fillDatasetLabels();
    renderGuidedSteps();
    dataset$("guidedAddStep").addEventListener("click", appendGuidedStep);

    let liveInterval = 0, liveBusy = false, liveQ = [1,0,0,0], previousLiveTime = 0;
    function startLive() {
      if (liveInterval) return;
      sourceLabel.textContent = "Live ESP32"; setMessage("กำลังเชื่อมต่อ ESP32…");
      liveInterval = setInterval(async function () {
        if (liveBusy) return; liveBusy = true;
        try {
          const response = await fetch("/data?_t=" + Date.now(), {cache:"no-store"});
          if (!response.ok) throw new Error("ESP /data ตอบ " + response.status);
          const data = await response.json(); latestLiveData = data;
          if (data.mount_site && dataset$("datasetMountSite").value !== data.mount_site) dataset$("datasetMountSite").value = data.mount_site;
          dataset$("datasetMountSite").disabled = true;
          dataset$("datasetDevice").textContent = (data.sensor_uid || data.sensor_id) + " · " + data.mount_site + " · " + (data.calibrated ? "calibrated" : "ยังไม่ calibrate");
          if (typeof window.captureLiveSample === "function") {
            try { window.captureLiveSample(data); }
            catch (captureError) {
              dataset$("datasetFeedback").textContent = "Recording/phase processing error: " + captureError.message;
              console.error("Live sample capture failed", captureError);
            }
          }
          const accel = [data.ax,data.ay,data.az], gyro = [data.gx_corrected,data.gy_corrected,data.gz_corrected], time = data.sample_ms/1000;
          if (!previousLiveTime) { liveQ = getInitialOrientation(accel,null); previousLiveTime=time; }
          else {
            const dt=time-previousLiveTime;previousLiveTime=time;
            if(dt>0&&dt<0.25){let omega=gyro.slice();let correction=[0,0,0];
              if(Math.abs(Math.hypot(...accel)-1)<0.4){const observed=vecNorm(accel),q=liveQ,expected=[2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]];correction=cross(observed,expected).map(function(v){return v*1.8;});}
              omega=omega.map(function(v,j){return v+correction[j];});const d=qMultiply(liveQ,[0,omega[0],omega[1],omega[2]]);liveQ=qNormalize(liveQ.map(function(v,j){return v+d[j]*0.5*dt;}));
            }
          }
          samples=[{t:time,q:liveQ.slice(),accel:accel,gyro:gyro,mag:null}];mode="live";currentIndex=0;updateReadouts(samples[0]);liveStatus.textContent="Live";setMessage("รับข้อมูลจาก ESP32 · firmware status: "+data.status+" · แยกจากผล ML");
        } catch(error) { liveStatus.textContent="Offline";setMessage("เชื่อมต่อไม่ได้: "+error.message,true);dataset$("datasetDevice").textContent="ESP ไม่พร้อม"; }
        finally { liveBusy=false; }
      },50);
    }
    window.addEventListener("load", function () { initializeModelTestNavigation(); startLive(); });
"""


def main() -> None:
    html = SOURCE.read_text(encoding="utf-8")
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    if "id=\"datasetCapture\"" not in html:
        html = html.replace('<section class="stage-grid">', CAPTURE_HTML + '\n    <section class="stage-grid">', 1)
    if "id=\"datasetCaptureStyle\"" not in html:
        html = html.replace("</head>", "<style id=\"datasetCaptureStyle\">" + CAPTURE_CSS + "</style></head>", 1)
    if "window.POSE_LABELS" not in html:
        script = LIVE_AND_CAPTURE_SCRIPT.replace("__POSE_LABELS__", json.dumps(config, ensure_ascii=False))
        html = html.replace("</script>", script + "\n  </script>", 1)
    payload = gzip.compress(html.encode("utf-8"), mtime=0)
    hex_array = ",".join(f"0x{byte:02x}" for byte in payload)
    source = ("#pragma once\n#include <Arduino.h>\n"
              f"const uint8_t webpage_html_gz[] PROGMEM = {{{hex_array}}};\n"
              f"const size_t webpage_html_gz_len = {len(payload)};\n")
    OUTPUT.write_text(source, encoding="utf-8", newline="\n")
    print(f"Embedded {len(html)} HTML bytes ({len(payload)} gzip bytes) from {SOURCE.name}")


if __name__ == "__main__":
    main()
