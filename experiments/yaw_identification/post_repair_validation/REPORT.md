# Mechanical reinstallation validation

Compared the two complete post-repair 0.10 Hz, ±15° trials against the pre-repair final tuned 0.10 Hz trial. Higher-frequency trials are excluded because the new run did not contain matched 0.15/0.25 Hz trials.

| Metric | Before repair | After repair (2 trials) | Change |
|---|---:|---:|---:|
| Yaw RMSE | 0.593° | 0.576° | -2.8% |
| Yaw P95 absolute error | 0.802° | 0.778° | -3.0% |
| Yaw peak error | 0.841° | 0.843° | +0.2% |
| Yaw torque RMS | 0.643 N·m | 0.599 N·m | -6.8% |
| Yaw peak torque | 1.072 N·m | 1.040 N·m | -3.0% |
| Pitch world range | 4.49…4.97° | 4.66…5.13° | — |

The yaw encoder mounting phase changed from a captured center near 70.0° to 216.5°/216.1°. The two post-repair arms differ by 0.40°, consistent with settling/backlash after reassembly. Relative tracking remains healthy and did not degrade.

Both post-repair sessions entered fault 4 during OFF/Releasing, after yaw torque was already zero. Pitch IMU speed crossed the 0.8 rad/s protection threshold while supporting torque was being removed. This is a pitch unloading issue, not yaw oscillation during the completed sweeps. Before the first arm, yaw moved from about 146.3° to 216.5° while commanded yaw torque remained zero; the experiment controller did not command that movement.
