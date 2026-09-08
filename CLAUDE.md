# ArduPilot fork notes

Airbound fork of ArduPilot. The vehicle-specific work here is a **dual axis
tiltrotor** (`Q_TILT_TYPE=4`, `TILT_TYPE_DUAL_AXIS`): two tilting booms where
axis 1 is the V22-style 0-90 degree transition tilt and axis 2 is an
independent thrust-vectoring servo per boom. Boards: `AB-Mod`, `AB-v2`,
`AB-TRT` under `libraries/AP_HAL_ChibiOS/hwdef/`.

## Build and test

Builds run under **cygwin**, not Git Bash or WSL:

```bash
cd /cygdrive/c/Code/ardupilot
./waf configure --board=sitl --toolchain x86_64-pc-cygwin   # once
export PYTHONPATH=/usr/lib/python3.7/site-packages
./waf plane
```

Autotests need a specific interpreter mix, because cygwin's python3.9 has no
numpy and its pymavlink lives in the 3.7 tree:

```bash
export PYTHONPATH=/usr/lib/python3.7/site-packages   # pymavlink
python3.8 Tools/autotest/autotest.py --no-configure --speedup=20 \
    test.QuadPlane.DualAxisTiltrotorTransitionStages
```

- `python3.8` is the only cygwin interpreter with numpy (1.19.4).
- MAVProxy was installed into it with `pip install --no-deps --only-binary :all: MAVProxy`
  (the harness imports `MAVProxy.modules.lib.mp_util` unconditionally).
- Windows python cannot run the harness (needs `pexpect`).

Two environment limitations, both pre-existing and unrelated to vehicle code:

- **Tests that restart SITL for a different frame fail here.**
  `customise_SITL_commandline()` cannot rebind the port, so the new SITL exits.
  Verified with unmodified `CopterTailsitter`, which fails the same way. This
  rules out the `quadplane-tilt*` frames in local runs.
- **A stale `arduplane.exe` cascades failures** across a batch. Run
  `taskkill //F //IM arduplane.exe` between runs; a test that fails in a batch
  is worth re-running alone before believing it.

## Transition architecture

`Transition` (`ArduPlane/transition.h`) is the abstract base. Three concrete
implementations, one per airframe style:

| Class | Where | Used by |
|---|---|---|
| `SLT_Transition` | declared `transition.h`, defined in `quadplane.cpp` | standard quadplanes |
| `Tailsitter_Transition` | `tailsitter.h/.cpp` | tailsitters |
| `Tiltrotor_Transition` (extends `SLT_Transition`) | `tiltrotor.h/.cpp` | every tilt type except dual axis |
| `Tiltrotor_Transition_DualAxis` (extends `Transition`) | `tiltrotor.h/.cpp` | `Q_TILT_TYPE=DualAxis` only |

`Tiltrotor::setup()` allocates exactly one tiltrotor transition and hands it to
`QuadPlane` as its generic `Transition*`. Both are also kept as concrete
pointers (`slt_transition`, `dual_axis_transition`) so tilt code can read each
machine's own state without downcasting.

### Dual axis stage machine

`Stage`: `VTOL`, `BACK_HOLD`, `BACK_BLEND`, `FWD_HOLD`, `FWD_BLEND`, `FW`.

Stages are sub-windows of a single elapsed-time timer started by a VTOL<->FW
**mode edge** — there is no airspeed wait. `update()` drives the FW side,
`VTOL_update()` the VTOL side; the edge is detected by the stage still holding
a value from the other side.

- Forward: `Q_TILT_FTHLD_MS` hold at `Q_TILT_THR_FT`, then `Q_TILT_FTBLD_MS`
  blend to the FBWA/commanded throttle.
- Back: `Q_TILT_FWHLD_MS` hold at `Q_TILT_THR_BT`, then `Q_TILT_BTDLY_MS`
  blend to the VTOL controller's demand.
- A back transition starts **only if fixed wing outputs were driven within the
  last 2 s** (`last_fw_output_ms`, set from `dual_axis_output()`). Without this,
  the `Stage::FW` that `force_transition_complete()` leaves at boot reads as a
  FW->VTOL edge and the first entry into a VTOL mode holds `Q_TILT_THR_BT`.

`update_controllers()` is the single place that selects controllers per stage.
It is called once per loop from `Tiltrotor::dual_axis_output()` and returns the
ESC throttle, so behaviour is identical in every Q-mode and no mode file needs
tiltrotor knowledge.

## Tick ordering (matters a lot)

1. `Plane::stabilize()` calls `transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd)`
2. …then `control_mode->run()` — the active Q-mode sets `nav_roll_cd`/`nav_pitch_cd`,
   calls `set_VTOL_roll_pitch_limit()`, runs its VTOL controller, then
   `stabilize_roll()`/`stabilize_pitch()`
3. `QuadPlane::update()` calls `transition->update()` or `VTOL_update()`
4. `Plane::set_servos()` → `servos.cpp` → `Tiltrotor::dual_axis_output()`

Consequences:

- **Every Q-mode already calls `stabilize_roll()`/`stabilize_pitch()` every
  tick.** Calling them again from `dual_axis_output()` would run those PID loops
  twice per tick. Do not.
- **No Q-mode calls `stabilize_yaw()`** (they centre the rudder instead; QRTL is
  the exception). So the transition can take yaw by calling it from
  `dual_axis_output()`, which runs later and overwrites the centred rudder.
  This is how `Q_TILT_FWHLD_EN` works without touching any mode file.
- To **command** roll/pitch from a transition stage, write `nav_roll_cd`/
  `nav_pitch_cd` from `set_VTOL_roll_pitch_limit()` (VTOL side, called by
  QLOITER/QRTL/QLAND/AUTO/GUIDED after the position controller and before
  `stabilize_*`) or `set_FW_roll_pitch()` (FW side, called before every mode's
  `run()`). `dual_axis_output()` is too late in the tick for that.

## Gotchas

- **`Plane` and `QuadPlane` gate internals behind explicit friend lists.**
  A transition class touching `stabilize_yaw()`, `TECS_controller`,
  `nav_pitch_cd`, `ahrs`, `aparm`, `control_mode`, `hold_stabilize()`,
  `motors_output()`, `assist`, `transition_pitch_max` must be added to both
  lists. Friendship is not inherited, so subclassing `SLT_Transition` grants
  nothing.
- **`quadplane.h` includes `tiltrotor.h` before defining `QuadPlane`.** Inline
  member bodies in `tiltrotor.h` cannot call `quadplane.anything()` — incomplete
  type. Define them in the `.cpp`.
- **`Q_TILT_TYPE` is a live parameter but `Tiltrotor::setup()` is boot-time.**
  Setting it to DualAxis at runtime sends `dual_axis_output()` down the dual
  axis path with a null `dual_axis_transition`. Gate on the pointer, not just
  the type.
- **Dual axis runs on `Q_FRAME_CLASS=10`** (tailsitter class, `AP_MotorsTailsitter`).
  The tailsitter's frame-class heuristic in `Tailsitter::setup()` must exclude
  dual axis, or it saves `Q_TAILSIT_ENABLE=1` and the next boot dies with
  "set TAILSIT_ENABLE 0 or TILT_ENABLE 0". Board defaults also set
  `Q_TAILSIT_ENABLE 0`.
- **`k_throttle` carries three meanings in one tick** for dual axis:
  the FW/pilot commanded throttle, then `AP_MotorsTailsitter`'s collective
  thrust actuator output (captured into `dual_axis_mixout_throttle` for the
  QTHR log), then the blended ESC throttle. The motor library's own throttle
  output is discarded; the boom motors are driven directly through
  `k_throttleLeft`/`k_throttleRight`.
- Roll/pitch/yaw authority in a dual axis forward transition comes from axis 2
  vectoring, not differential throttle — both booms get the same ESC value.

## Logging

- Stage changes emit `gcs().send_text()` ("Fwd trans: throttle hold",
  "Back trans: throttle hold (FW yaw ctrl)", "Fwd trans done", …). These also
  land in the `.bin` `MSG` log, so they double as searchable markers.
- `get_log_transition_state()` feeds the generic quadplane log's
  `transition_state` field, so `Stage` is logged without extra plumbing.
- `TILT` log carries the stage timers and throttles. All five throttle fields
  are percent; `FWThr` is the 0-1 last FW throttle.
- Debug messages `PHID`/`PHIF`/`PITE` cover axis 2 vectoring, `QTHR` traces
  `k_throttle` through the pipeline.

## Tests

`Tools/autotest/quadplane.py`:

- `VTOLTakeoffTransitionRTL` — mission flying VTOL takeoff, forward transition,
  VTOL RTL back transition, asserting `MAV_VTOL_STATE` at each step.
- `DualAxisTiltrotorTransitionStages` — dual axis stage machine on the ground
  (no SITL flight model exists for `Q_TILT_TYPE=4`), driven by mode edges:
  VTOL-to-VTOL runs no transition, each direction reports hold/blend/done,
  `Q_TILT_FWHLD_EN` adds the FW yaw hold, zeroed timers complete immediately.

Both tests found real defects when first run: the null dereference on live
`Q_TILT_TYPE` change, the phantom boot back transition, and the tailsitter
auto-enable collision. Prefer running them after any transition change.
