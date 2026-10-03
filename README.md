# PlanKO: one IMU per body point

Each board measures one body point using an ESP32-C3 with an MPU-6050 sensor.
Set `SENSOR_POINT` in `platformio.ini` to one of `chest`, `lumbar`,
`pelvis`, `neck`, `head`, `left_upper_arm`, `right_upper_arm`, `left_forearm`,
`right_forearm`, `left_thigh`, `right_thigh`, `left_shin`, or `right_shin` before
flashing each board. Each board must have a unique point name.

To build and upload to ESP32-C3:

```powershell
pio run -t upload
```

Or specify the upload port explicitly:

```powershell
pio run -t upload --upload-port COM8
```


The firmware sends CSV through USB serial at 115200 baud. It samples whenever
both acceleration and gyro data are ready, calibrates gyro bias while the board
is held still at startup, and reports acceleration in m/s², angular speed in
rad/s, and magnetic field in µT. `boot_ms` and `seq` expose the sample rate,
dropped samples, and board restarts. Magnetometer cells are blank when no fresh
reading was available. The host adds `host_time_s` on receipt so recordings from
different boards can be aligned. Capture prints the measured firmware sample
rate and sequence gaps when recording stops.

## Record one board

Install the Python serial package:

```powershell
python -m pip install pyserial
```

Run one capture process per connected board (each has a different COM port):

```powershell
python tools/capture.py --port COM5 --output D:\plankO\imu_output
```

Stop with Ctrl+C. Capture writes `YYYYMMDDTHHMMSSZ_<point>.csv` and a matching
`_orientation.json` file. The JSON contains only numeric `[roll, pitch, yaw]`
rows in radians, with one row per captured sample. The console reports the
measured firmware sample rate and missing sequence count. Repeat simultaneously
for all body points; the recordings must overlap in time. The sample simulation
files in that folder are read only by these scripts and are never overwritten.

If no board is connected, generate a smooth 9-axis Nano 33 BLE Sense-like sample
CSV and its numeric orientation JSON:

```powershell
python tools/simulate_imu.py --output imu_output/simulated_chest.csv
python tools/view_imu.py imu_output/simulated_chest.csv --autoplay
```

The generated accelerometer and gyro run at 100 Hz; magnetometer updates run at
10 Hz. These are synthetic values for exercising the capture and animation flow,
not measurements from a physical board.

## Animate one IMU recording

Install matplotlib and numpy, then open a per-board capture CSV:

```powershell
python -m pip install matplotlib numpy
python tools/view_imu.py D:\plankO\imu_output\20260101T120000Z_chest.csv
```

Add `--autoplay` to start immediately or `--speed 5` to play five times faster.
For the earlier sample recording:

```powershell
python tools/view_imu.py D:\plankO\test.csv --speed 5 --autoplay
```

The 3D board shows orientation estimated from gyro integration, with
accelerometer correction for tilt and magnetometer correction for heading when
magnetic samples are present. The Play button runs the recorded motion; the
slider selects a time. Older six-axis CSVs still work, but yaw can drift
without magnetometer data. Raw magnetic readings may need hard/soft-iron
calibration before heading is accurate. This visualizes sensor orientation, not
absolute body position; mount alignment determines how board axes map to the body.
Older CSVs with `host_time_s`, `boot_ms`, `accel_x/y/z`, and `gyro_x/y/z` columns
are supported too. To export only numeric `[roll, pitch, yaw]` rows in radians:

```powershell
python tools/view_imu.py D:\plankO\test.csv --json-output D:\plankO\test_rotation.json --export-only
```

## Combine points

Pass one capture CSV per point from the same session:

```powershell
python tools/merge.py D:\plankO\imu_output\*_chest.csv D:\plankO\imu_output\*_lumbar.csv --output D:\plankO\imu_output\hardware_wide.csv
```

The merged file has `time_s,accel_chest_x,...,gyro_lumbar_z,...,mag_lumbar_z,...`
columns like `imu_samples.csv`. Missing samples are blank. The capture files retain the
original host timestamps and board sequence numbers. Choose files from one run;
wildcards can accidentally select older runs when the folder contains several.

## Compare to the simulated body

```powershell
python -m pip install matplotlib
python tools/view_body.py --simulation D:\plankO\imu_output\plank_back_demo.csv --hardware D:\plankO\imu_output\hardware_wide.csv --offset 0
```

The slider shows simulated body positions and colors each recorded point by the
difference in acceleration magnitude. Set `--offset` to align the same movement
phase in the two recordings. This is a signal check, not a reconstructed body:
`pos_*` and `axis_*` in the demo files are MuJoCo ground truth. Real IMUs do not
measure absolute position. Reconstructing pose from hardware will require
sensor mounting calibration, orientation estimation, body segment lengths, and
synchronized movement start times.
