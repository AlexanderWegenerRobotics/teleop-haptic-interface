# teleop-haptic-interface

Operator-side haptic teleoperation interface. Two Sigma 7 devices map to two Franka Panda arms running in a MuJoCo sim (avatar side).

Avatar / simulation repo: [teleop-avatar](https://github.com/AlexanderWegenerRobotics/teleop-avatar.git)

---

## What it does

The operator holds two Sigma 7 haptic devices. The interface sends position/orientation deltas to the corresponding robot arms and renders force feedback based on how far the arm deviates from where the device commanded it to be. A passivity observer monitors the net energy flowing to the operator and injects damping if the loop starts adding energy (relevant under network latency).

---

## Folder structure

```
config/
  system.yaml          # ports, log dir, mock_motion flag
  haptic_left.yaml     # device id, frame rotation, impedance gains, safety limits
  haptic_right.yaml

include/
  haptic_device.hpp    # IHapticDevice interface
  mock_haptic_device.hpp   # sine-wave mock, no hardware needed
  sigma_device.hpp     # Sigma 7 wrapper (only compiled WITH_SIGMA=ON)
  teleop_controller.hpp
  teleop_session.hpp
  passivity_controller.hpp
  avatar_channel.hpp
  arm_channel.hpp
  udp_arm_channel.hpp
  network/             # UDP primitives (stream + reliable)

src/
  main.cpp
  teleop_session.cpp   # top-level state machine (IDLE / ENGAGED)
  teleop_controller.cpp  # 1 kHz control loop per arm
  passivity_controller.cpp
  udp_arm_channel.cpp
  avatar_channel.cpp
  sigma_device.cpp
  network/
```

---

## How a control step works

1. **Read device** — get position, orientation, velocity from Sigma 7 (or mock)
2. **Compute setpoint** — take the delta from the captured origin pose, rotate into world frame, apply motion scaling → send as `ArmCommandMsg` to the arm over UDP
3. **Compute haptic wrench**
   - position error: `(arm displacement from its origin) − (device displacement from its origin × scale)`
   - orientation error: quaternion difference between commanded and actual arm orientation
   - impedance: `F = K·error − D·velocity`
   - passivity correction: if `E_obs = ∫ F·v dt > 0` (net energy delivered to operator), inject variable damping to drain the surplus
   - rate-limit and magnitude-clamp for safety
4. **Send force** — output to Sigma 7 (or no-op for mock/null)
5. **Log** — device pose, arm pose, impedance force, passivity correction, energy observation → CSV

The origin is captured on SPACE press. Both arms snapshot their own pose and the device pose at that moment; all subsequent errors are relative to that snapshot so there's no jump on engage.

---

## Frame convention

Sigma 7 axes don't match world axes. `frame_rotation` in the haptic YAML defines `R_device_to_world`. Currently:

```
R = diag(-1, -1, 1)   # sigma X=backward → world X=forward, Y=right → Y=left
```

Tune this once real hardware is connected and the arm moves in the wrong direction.

---

## Building

Dependencies via vcpkg: `Eigen3`, `Poco`, `yaml-cpp`, `msgpack-cxx`

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake -DWITH_SIGMA=OFF
cmake --build build --config Release
```

`WITH_SIGMA=OFF` (default) compiles without the Sigma SDK — uses `MockHapticDevice` or `NullHapticDevice` depending on the `mock_motion` flag in `system.yaml`.

---

## Running

```bash
./build/Release/teleop_haptic_interface.exe config/system.yaml
```

- **SPACE** — capture origin and engage
- **ESC / q** — disengage
- **Ctrl-C** — quit

Logs land in `log/` as CSV, one file per arm.

---

## Testing without hardware

Set `session.mock_motion: true` in `system.yaml`. The mock device generates ±20 mm sinusoidal motion on X at 0.1 Hz. With zero network latency the passivity controller should stay dormant (`f_pc ≈ 0`, `e_obs ≤ 0`). This has been verified.

The arm tracking end-to-end (sim actually following commands) is pending a state-machine alignment with the avatar side — the sim expects a HOMING → AWAITING handshake before it will execute ENGAGED commands.
