# Gantry bringup

The gantry uses three M2006 motors on CAN1. Facing the launch direction,
ID 2 is the bottom left lift screw and ID 3 is the bottom right lift screw;
both control pitch. ID 1 is the top yaw screw. The C Board DBUS port
receives the DR16. After homing, both DR16 switches DOWN disable joystick
motion; every other valid switch combination allows it. The left stick's vertical
channel changes pitch, and its horizontal channel changes yaw. The right stick
does not command either axis.

The supplied [gantry.yaml](gantry.yaml) enables manual speed control with
`manual_test_mode: true`. Each motor has one velocity PID, with gains
`kp: 0.15`, `ki: 0`, `kd: 0` and output limited to 0.8 N m. The controller
feeds its joystick speed target directly into these three PIDs. The three
position PID components are absent from this manual bringup configuration.
`geometry_ready: false` disables calibrated position control,
but does not disable this manual mode. To collect feedback without commanding
motion, set both `auto_home: false` and `manual_test_mode: false` while keeping
`geometry_ready: false`.

With `auto_home: true`, launch first waits for fresh feedback from all three
motors and a valid DR16 connection, then drives the two lift screws upward at a
12.5 rad/s target with a 0.8 N m torque cap. The homing sequence runs even if
both switches are DOWN. Yaw and all joystick commands are ignored during homing. Each
lift motor is stopped separately when its commanded and measured output torque
are at least 0.25 N m while output speed stays below 0.05 rad/s for 0.5 s.
Only after **both** have stopped does the controller assign their current
positions zero and enable manual control. Keep the left stick centered once
homing finishes. `/gantry/homing_state` is 0=waiting, 1=seeking, 2=complete,
3=failed; `/gantry/debug/home_left_reached` and `home_right_reached` show which
side has stopped. A feedback or remote loss during seeking, 120 s timeout, or
more than 20 mm of upward screw travel locks motion with state 3. Verify that
both upper mechanical stops really represent the same level: friction or a jam
before the stop can look identical to the sensorless contact test.

After homing, center the left stick once, then use its vertical direction
for both lifts and horizontal direction for yaw. Both axes can run together.
Both switches DOWN disable joystick motion; a mixed DOWN/UP or DOWN/MIDDLE
position still allows it. Releasing the stick removes torque enable. After
feedback or remote loss, center the left stick again.
The target speed reaches 10 output-shaft rad/s at full stick and continues while
the stick is held. The lift commands are adjusted using the measured left/right
angle difference. Above 3 rad (about 0.095 mm of screw travel), only the side
that reduces the relative difference moves at up to 2 rad/s; normal dual-lift
control resumes below 1.5 rad. Yaw remains available. There is no timed or
startup-relative travel cutoff in this test mode because the safe travel has
not yet been measured. A stalled motor is re-armed when its left-stick direction
returns to center; an over-temperature fault still requires both switches DOWN,
a centered left stick, fresh motor feedback and temperatures below the limit.
Debug test states are 0=disarmed, 1=ready, 2=lift, 3=yaw,
4=interlock, 5=both axes, 6=lift skew recovery.
Zero current does not prevent external motion or gravity-driven movement.

For feedback-only checks, disable manual mode as described above and mechanically
support the launcher. The existing ValueCollector records output-shaft angles (rad), velocities
(rad/s), estimated torques (N m) and screw travel (m; lift positions are
relative to their upper stops after successful homing, yaw to startup) at 100 Hz to
`/tmp/gantry_<timestamp>.csv` on the computer running RMCS. Disabled PID outputs
(`control_torque`) are NaN; DjiMotor translates these into
zero current commands. Pitch/yaw angles remain placeholders until geometry is
configured. Stationary zero readings alone do not prove that CAN feedback is present.

From the workspace root, build/install the updated packages and launch:

```sh
colcon build --packages-select rmcs_core rmcs_bringup --symlink-install
source install/setup.bash  # Use setup.zsh in zsh.
ros2 launch rmcs_bringup rmcs.launch.py robot:=gantry
```

To inspect the CSV, open the path printed by ValueCollector or use
`head -n 1 /tmp/gantry_<timestamp>.csv` for column names and
`tail -f /tmp/gantry_<timestamp>.csv` for samples (replace the timestamp).
Only turn a screw gently for direction checks if the supported mechanism permits
manual movement; do not force a jammed or self-locking transmission.

Before enabling controlled motion:

1. Confirm CAN IDs, motor directions and the lead of each screw. The supplied
   lead is 0.2 mm/revolution (0.0002 m/revolution) for both pitch and yaw,
   as corrected by the operator. M2006's built-in reduction ratio is used.
2. Put the launcher at a known, level reference pose. Measure the horizontal
   distance from pivot O to the front (`front_distance_m`), the front height
   above O (`pitch_height_at_start_m`) and the yaw screw's lateral offset
   (`yaw_position_at_start_m`). Starting motor angles are taken as zero
   displacement; restarting at a different pose invalidates the calibration.
3. Measure each axis's safe travel **relative to that reference pose** and
   enter the pitch and yaw travel minimum/maximum. Set conservative pitch and
   yaw angle bounds inside those travel limits.
4. After calibration, configure conservative nonzero PID gains and output limits
   for supervised low-speed commissioning. Set `geometry_ready: true`, start with
   both switches down, then restore and tune the three position PID components
   alongside the three velocity PIDs with mechanical support in place. The
   supplied YAML currently contains only the single-loop manual configuration.

With a fixed pivot O and fixed front distance L, the controller estimates
`yaw = atan2(lateral, L)` and
`pitch = atan2(front_height, hypot(L, lateral))`. The pitch target height
therefore follows the *measured* yaw-screw position while yaw moves. If the
mechanism does not match that geometry, its equations must be changed before
enabling motion.
Yaw pauses when the lift falls behind the required height by more than
`max_pitch_tracking_error_m`, or when the intended yaw would require the
lift to leave its measured travel range.

In manual mode, the two lift speed targets compensate for their measured angle
difference, and excessive difference switches to slow one-side correction.
A stalled motor's CAN command is zeroed individually; the other motor continues
until the lift difference limit intervenes. In
calibrated mode, each lift side needs its measured
travel limits configured.
Feedback or DR16 loss also disables drive. A motor commanded to move with
substantial torque while its speed remains near zero for the configured duration
latches its own stall fault; the CSV records `/gantry/debug/left_stall`,
`right_stall`, and `yaw_stall`. Releasing the corresponding left-stick direction
clears a stall latch, so a later command can retry without relaunching. Motor
temperature is checked too. Since the pitch structure may
move under gravity without motor torque, confirm the mechanical support and
fault behavior during initial testing.

This sensorless stall check detects resistance **after** contact. It cannot
identify the physical end of travel on its own, so measured travel limits
are required for routine operation. Stall thresholds and PID gains in the
configuration need low-speed tuning; the current manual-test gains are not
validated gains for normal operation.
