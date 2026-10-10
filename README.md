# IMU Pose Data and Model Prototype

This project reads one MPU6050 on one ESP32-C3 and classifies actions for the
selected body part. Wrist is the current trained prototype. Elbow has action
labels, neck/lumbar/pelvis now have starter label sets, and shoulder remains an
extension point. Each new part needs labeled recordings and a newly trained
model. Add future action IDs in `config/pose_labels.json`. IDs must stay unique
across body parts.

## Initial neck, lumbar, and pelvis labels

The recorder offers neutral, flexion/extension, and left/right tilt labels for
the neck and lower back. Pelvis labels cover neutral, anterior/posterior tilt,
and left/right tilt. These describe the orientation of the sensor-mounted
segment; one IMU cannot isolate lower-spine motion from pelvis motion or report
a clinical joint angle. Static yaw/rotation labels are omitted because this
six-axis IMU has no absolute heading reference. Calibrate separately after
moving the sensor to each position. Phase labels remain provisional until their
time boundaries are reviewed.

## Hardware scope

- One ESP32-C3 reads one IMU at about 20 Hz. It measures one installed position
  at a time; this firmware does not synchronize multiple IMUs.
- `sensor_uid` is derived from the device MAC and is separate from the selected
  `mount_site`, `body_part`, side, and action label.
- The body-position selector on `/calibrate` changes the persisted mount site.
  Side comes from left/right-specific site names, or from the selected
  PlatformIO environment when the mount site has no side in its name.
- Firmware threshold status remains a separate legacy diagnostic. The ML UI
  labels it as firmware status and never uses it as an action annotation.

## Build and upload firmware

Install PlatformIO Core or use the PlatformIO IDE. Build without uploading:

```powershell
pio run -e esp32c3-wrist
```

Available environments include `esp32c3-wrist`, `esp32c3-right-forearm`,
`esp32c3-left-forearm`, `esp32c3-right-upper-arm`,
`esp32c3-left-upper-arm`, `esp32c3-chest`, `esp32c3-neck`,
`esp32c3-shoulder`, and `esp32c3-elbow`. The web calibration page can select
the physical position at runtime; the environment sets the default and side.

Upload only when intentionally flashing a connected board:

```powershell
pio run -e esp32c3-wrist -t upload --upload-port COM8
```

Every PlatformIO build runs `scratch/build_html_pre.py`, which rebuilds
`src/webpage.h` from the latest `index2.html`, `config/pose_labels.json`, and
`scratch/build_html.py`. The recorder UI does not need a separate manual build.

## Calibrate the sensor

Connect to the ESP32 Wi-Fi access point (`ESP32-IMU`), then open the IP printed
in the serial monitor followed by `/calibrate`. `/calibate` redirects to the
correct spelling. Select the physical mount position, place the sensor in the
neutral pose you plan to use, and start calibration.

Calibration gathers 40 consecutive still samples (about two seconds), estimates
neutral roll/pitch and gyroscope bias, and times out after ten seconds. Motion,
an IMU read failure, or a sample gap clears the current still interval. The old
profile stays stored until the new profile is saved successfully. The saved
record includes device UID, mount site, calibration ID, method version, and
completion uptime milliseconds paired with the calibration boot ID. The ESP has
no real-time clock, so that time is not a wall-clock timestamp. Calibrate again after moving or
reattaching the sensor. Relative sensor orientation is a sensor reference; it
does not represent a joint angle in every mounting arrangement.

## Collect labeled data from the ESP page

Connect to the same ESP Wi-Fi network and open the device root page. Use the
dataset recorder to select body part and action, then enter subject, session,
trial, and side. The mount site follows the position reported by firmware and
cannot silently overwrite the device UID. Actions shown for each body part are
read from `config/pose_labels.json`.

1. Leave the guided phase timer enabled. Before recording, choose each phase,
   set its duration, and use the up/down buttons to arrange the sequence. Add
   or remove phases as needed. The default sequence is neutral 3 s → approaching
   3 s → holding 5 s → returning 3 s → neutral 2 s.
2. Start while the person is in the first phase. Follow the current-phase bar,
   countdown, and audio cue. Thai speech is used when the
   browser has a Thai voice; otherwise short beeps mark the warning and phase
   changes. The capture ends automatically after the final neutral segment.
3. Review the phase boundaries in the annotation editor. Move timestamps when
   needed and check **ตรวจแล้ว/ใช้ฝึก** for each segment, or use **ยืนยันทุกช่วง
   หลังตรวจ** after reviewing the whole sequence. Then download the CSV.

Timer and manual-button times are provisional cues, not movement ground truth.
New recorder CSV rows include `phase_annotation_source` and
`phase_annotation_reviewed`; the trainer excludes unreviewed phase labels.
Older CSVs without the review column keep their existing explicit-label
behavior. Keep the raw samples and review phase boundaries; do not label an
action from `firmware_status` or `steady`.

The CSV stores raw accelerometer values in m/s² and raw gyroscope values in
rad/s, device boot time and sequence, host receive time, calibration metadata,
position metadata, and action/phase labels. A calibration change splits the
recording. Gaps and reboots are marked so training windows cannot cross them.

For a serial capture from a computer, install `pyserial` and set metadata through
the command-line options:

```powershell
python -m pip install pyserial
python tools/capture.py --port COM8 --output imu_output `
  --mount-site wrist --body-part wrist --side right `
  --subject-id P01 --session-id S01 --trial-id S01-wrist-up `
  --action-label wrist_up
```

Set `--phase-label` only when the complete capture truly has one phase. For
normal multi-phase trials, record from the ESP page or annotate phase labels in
the CSV. Capture ignores duplicate serial samples and starts a new file when
the calibration profile changes.

`tools/merge.py` joins multiple recordings by receive time for future
multi-device experiments. It uses actual samples from one reference stream;
unmatched values stay blank. It does not copy or interpolate samples to make a
slower stream appear faster. The current firmware still has no multi-IMU clock
synchronization, so merged data is not proof of synchronized multi-position
measurement.

## Install ML dependencies and train

Use a Python environment for the project:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements-ml.txt
```

Train from supplied CSV paths, or omit paths to read `whist/*.csv`:

```powershell
python tools/train_model.py --output models/pose_model
python tools/train_model.py imu_output\trial1.csv imu_output\trial2.csv `
  --window 1.0 --output models/pose_model
python tools/train_model.py imu_output\trial1.csv imu_output\trial2.csv `
  --window 0.5 --output models/pose_model_0_5s
```

The shared causal feature pipeline is in `tools/imu_features.py`. It uses raw
accelerometer/gyroscope statistics, magnitudes, slopes, site and body-part
context, and calibrated orientation only when a valid calibration ID is
present. Training compares Random Forest with RBF SVM on group-separated
trial/session splits, or by subject when multiple subjects are available. Each
mount site's continuous IMU axes occupy separate feature columns; the model
does not treat sensor coordinates from different installation positions as a
shared body axis. The evaluation JSON reports per-class precision/recall/F1, macro-F1, confusion
matrix, and why evaluation is unavailable when the independent groups do not
cover each class. The chosen estimator and its feature order, units, labels,
window, calibration requirements, and version are saved in
`models/pose_model/model_bundle.joblib`.

The existing `whist/up.csv`, `down.csv`, and `nomal.csv` are explicitly mapped
to wrist action IDs by filename. Their `trial_label=PC` is not treated as an
action label, their data is uncalibrated, and zero `relative_tilt_deg` is not
used as a normal-pose feature. They have no valid phase annotation, so the
prototype trained from them has an action model only; its stage remains
`unknown`. A phase model is only saved when explicit phase annotations have an
independent group-separated evaluation; otherwise live stage stays `unknown`.
The three files are not enough for an independent group-separated
evaluation of all actions. Collect repeated trials per action and more subjects
before interpreting model quality.

## Test the model with CSV replay or live ESP data

Run the Python server on the computer:

```powershell
python tools/serve_model.py --model models/pose_model/model_bundle.joblib `
  --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765/`. CSV mode provides a timeline, play/pause, and
label reference in the prediction log. Download the JSON log to compare model
action/stage with the CSV annotations. Live mode asks the local Python backend
to read buffered sample batches from `/samples` on the ESP, so short HTTP delays
do not discard IMU frames and the browser does not need cross-origin access.
The firmware still provides `/data` for the existing dashboard; enter the ESP's
IP address if it differs from the default.

The ESP Index page also has a **Model Test** button. Start the Python server on
the same PC and use the default `http://127.0.0.1:8765/` address, or enter a
reachable PC/server address in the field. To open it from a different device,
run the server with `--host 0.0.0.0` and enter the PC's reachable IP and port.
The **Calibrate** button opens the ESP's `/calibrate` page directly.

The output distinguishes `approaching`, `holding`, `neutral`, `unknown`, and
`returning`. Stage stays `unknown` until a phase model can be trained from
explicit phase annotations. Scores are model scores, not calibrated
probabilities. Unknown is returned for short or broken windows, missing
calibration when required, or a position/body-part pair not represented in
training. Calibration and smoothing state reset across gaps, reboot, site, or
calibration changes.
