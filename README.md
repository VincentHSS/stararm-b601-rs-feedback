# Star Arm 102 Force Feedback Workbench

<sub>**English** / [简体中文](README.zh.md)</sub>

Local teleoperation and force feedback for a **Star Arm 102 Leader + Seeed reBot B601 RobStride follower**. Includes dynamic damping, stall holding, a 3D pose view and browser settings.

Current source version **0.6.2** · Install from this source tree; existing Releases do not contain this update · [Change history (Chinese)](CHANGELOG.md)

## Features

- **Dynamic feedback**: Leader damping tracks the follower load; stall HOLD works independently
- **Live workbench**: 3D pose, seven joint states and torque history on one page
- **Torque annotations**: numeric joint labels and link colors for motor torque or model residual
- **Seven color curves**: joint selection, sample readings and historical session replay
- **Online settings**: shared and per joint power caps; deadzones, motion margins, torque scales and response settings are fixed displays
- **Chinese and English**: switch with “中文 / EN”; the browser remembers the choice without changing parameter values
- **Local operation**: the interface, models, icons and Three.js are included; no external CDN

J1–J6 and the gripper support ordinary dynamic damping. J6 uses its own wrist rotation residual. The shared cap defaults to 1200 mW and accepts 400–1500 mW. Independent HOLD may request 1500 mW on every joint. A J6 cap previously saved as 0 remains disabled until raised and saved.

## Requirements

- Linux, Python 3.12, LeRobot and both device plugins
- Working serial/SocketCAN connections, correct direction mapping, valid calibration and verified basic following
- A desktop browser with WebGL support

See the [Seeed B601-RS guide](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/) for device setup. Device types are `rebot_arm_102_leader` and `seeed_b601_rs_follower`. B601-DM is not supported by this implementation.

## Install

```bash
git clone https://github.com/VincentHSS/stararm-b601-rs-feedback.git
cd stararm-b601-rs-feedback
conda activate lerobot
python -m pip install --no-deps dist/motorbridge_smart_servo-0.0.4+rsfix1-*.whl
python -m pip install --no-deps .
```

Wheels in [Releases](https://github.com/VincentHSS/stararm-b601-rs-feedback/releases) and `dist/` are older versions. This update changes source only; install from the checkout as shown above.

The included corrected Leader SDK supports Linux x86_64, glibc ≥ 2.34 and Python ≥ 3.9. See [SDK compatibility and build instructions (Chinese)](sdk-fix/README.md) for other platforms. Install `python-can>=4.2,<5` if missing from the environment.

## First binding

Use an upright fixed base and the stock gripper. Generate a local binding using your actual calibration path:

```bash
python -m stararm_rs_feedback.bind \
  --calibration /actual/path/follower1.json \
  --robot-id follower1 \
  --upright-stock-gripper \
  --output feedback.rs-dynamic.json
```

Binding reads existing configuration and calibration without connecting hardware, setting zero or overwriting an existing file. Rebind after changing calibration or mapping. `feedback.rs-dynamic.example.json` documents the format and cannot be used directly with hardware.

Keep your local `feedback.rs-dynamic.json` and `data/` when upgrading. For bindings from 0.5.1 or earlier, add `"arm_motion_range_reset": false` to `policy` while retaining the signature, calibration and other fields.

When upgrading a binding from 0.6.1 or earlier to the new J6 policy, back it up and move it out of the way, then run First binding again using the same existing calibration. Binding is offline and does not recalibrate or set zero. Do not edit hashes to bypass the policy check.

## Start the workbench

```bash
conda activate lerobot
bash run_dashboard.sh
```

Open <http://127.0.0.1:8765/>. Use “中文 / EN” in the header to change language. To choose a different port, run `bash run_dashboard.sh 8766`.

Starting the server does not connect hardware. Exit existing terminal teleoperation sessions, then click **Start feedback**. **Monitor only** still makes the follower track the Leader, but this feedback layer sends no Leader damping or HOLD. Defaults are `can0`, `/dev/ttyUSB0`, follower ID `follower1`, Leader ID `rebot_arm_102_leader` and 30 Hz.

If CAN is down, stop teleoperation and configure your adapter:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Only the shared and individual joint power caps are editable. Higher caps allow stronger damping; lower caps limit strength. Deadzones, motion margins, full output scales and response settings are fixed. Lowering the shared cap also lowers joint caps above it; raising it does not automatically raise individual caps. **Restore default caps** changes only power caps and preserves fixed settings.

Settings are saved locally under `data/web/`. After **Save and apply**, check “Applied vN” to confirm that the control loop accepted them. See [workbench details (Chinese)](WEB.md).

## Terminal operation

```bash
bash run_force_feedback.sh       # Default cap: 1200 mW
bash run_force_feedback.sh 1500  # Cap: 1500 mW; choose one command
```

Shared damping defaults, individual caps and seven joint thresholds are grouped at the start of `stararm_rs_feedback/tuning.py`. Model/communication policy remains in `dynamic.py` under `POLICY`. If changing policy in source, also update the local binding policy. Browser power caps persist separately. See [dynamic policy details (Chinese)](DYNAMIC.md).

## Operation notes

- N·m is estimated motor torque; Iq is filtered motor current. Neither directly measures end effector contact force or payload mass
- mW is the requested Leader power, not measured resistance. Curve colors identify joints; model colors show torque relative to the display range, not a safety limit
- **Stop session** sends SIGINT and performs normal homing/release cleanup. Leave room and support the arms. Closing the browser does not stop a session
- Installing an update does not replace a running process. Stop the old session normally, restart the server and refresh the page
- CAN freshness, faults, temperature, position jumps, limits, continuous Leader gaps and near zero pose checks remain active. Damping continues above the former arm motion model thresholds, where estimation error can increase

## Tests and resources

```bash
python -B -m unittest discover -s tests -q
node tests/test_torque.mjs
node tests/test_motion.mjs
node tests/test_presentation.mjs
node tests/test_i18n.mjs
```

Python tests use device doubles and pseudo terminals. Full fitting tests need NumPy; native transport tests need the corrected SDK. See [validation scope (Chinese)](VALIDATION.md) for historical replay results and hardware validation limits.

- [Workbench guide (Chinese)](WEB.md)
- [Dynamic policy (Chinese)](DYNAMIC.md)
- [Change history (Chinese)](CHANGELOG.md)
- [Interface design (Chinese)](stararm_rs_feedback/web/DESIGN.md)
- [Third party sources and licenses](NOTICE.md)
- [Original Star Arm 102 force feedback project](https://github.com/VincentHSS/stararm102-force-feedback)
