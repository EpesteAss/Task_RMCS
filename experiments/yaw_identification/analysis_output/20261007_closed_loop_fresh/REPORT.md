# Yaw closed-loop second-order identification

Input is the known yaw angle target. The model simulates the complete 24 s response after two initial samples. It includes the configured 0.5 rad/s speed limit.

Equation: theta_dot = omega; omega_dot = -a*omega + k*(reference-theta) + c; |omega| <= 0.5 rad/s.

| Data | Full-run velocity fit | Speed RMSE (rad/s) | Angle RMSE (deg) |
|---|---:|---:|---:|
| Nine-profile evaluation | 84.18% | 0.03628 | 0.631 |

| Profile | Velocity fit |
|---:|---:|
| 0 | 79.80% |
| 1 | 84.14% |
| 2 | 86.54% |
| 3 | 88.20% |
| 4 | 89.31% |
| 5 | 89.96% |
| 6 | 89.98% |
| 7 | 82.59% |
| 8 | 80.18% |

This is a target-to-response model of the controlled yaw system. The torque-to-response model uses a different input, so its percentage must be reported separately. This session is the planned independent validation collected after the model and configuration were frozen.

[Low-speed profile 0 plot](validation_profile_0.svg) | [Profile 8 plot](validation_profile_8.svg) | [Machine-readable results](results.json)
