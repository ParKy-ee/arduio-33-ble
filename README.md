# PlanKO: one IMU per body point

Each Nano 33 BLE board measures one body point. Flash the environment that matches
the physical board: `nano33ble` for the first generation or `nano33ble_rev2` for
Rev2. Set `SENSOR_POINT` in `platformio.ini` to one of `chest`, `lumbar`,
`pelvis`, `neck`, `head`, `left_upper_arm`, `right_upper_arm`, `left_forearm`,
`right_forearm`, `left_thigh`, `right_thigh`, `left_shin`, or `right_shin` before
flashing each board. Each board must have a unique point name.

The default Upload action uses `nano33ble` (first generation) only. To upload
explicitly, use:

```powershell
pio run -e nano33ble -t upload
```

Only use `-e nano33ble_rev2` for a board marked Rev2. If the upload loses the
COM port after its 1200 bps reset, close Serial Monitor and capture.py, then
double-press RESET to enter bootloader mode (the status LED should pulse).
Run `pio device list` and note the port shown while the board is in bootloader
mode. Retry with that port explicitly:

```powershell
pio run -e nano33ble -t upload --upload-port COM7
```

Replace `COM7` with the port listed on your computer. The bootloader can appear
as a different COM port from the normal sketch.

The firmware sends CSV through USB serial at 115200 baud. It targets 100 Hz and
reports acceleration in m/s² and angular speed in rad/s, matching the simulation
columns. `boot_ms` and `seq` expose dropped samples and board restarts. The host
adds `host_time_s` on receipt so recordings from different boards can be aligned.

## Record one board

Install the Python serial package:

```powershell
python -m pip install pyserial
```

Run one capture process per connected board (each has a different COM port):

```powershell
python tools/capture.py --port COM5 --output D:\plankO\imu_output
```

Stop with Ctrl+C. Capture writes `YYYYMMDDTHHMMSSZ_<point>.csv` and a metadata
JSON in the selected folder. Repeat simultaneously for all body points; the
recordings must overlap in time. The sample simulation files in that folder are
read only by these scripts and are never overwritten.

## Combine points

Pass one capture CSV per point from the same session:

```powershell
python tools/merge.py D:\plankO\imu_output\*_chest.csv D:\plankO\imu_output\*_lumbar.csv --output D:\plankO\imu_output\hardware_wide.csv
```

The merged file has `time_s,accel_chest_x,...,gyro_lumbar_z,...` columns like
`imu_samples.csv`. Missing samples are blank. The capture files retain the
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
