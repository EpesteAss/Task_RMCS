# Yaw closed-loop second-order identification

Input is the known yaw angle target. The model simulates the complete 24 s response after two initial samples. It includes the configured 0.5 rad/s speed limit.

Equation: theta_dot = omega; omega_dot = -a*omega + k*(reference-theta) + c; |omega| <= 0.5 rad/s.

| Data | Full-run velocity fit | Speed RMSE (rad/s) | Angle RMSE (deg) |
|---|---:|---:|---:|
| Nine-profile evaluation | 84.58% | 0.03546 | 0.599 |

| Profile | Velocity fit |
|---:|---:|
| 0 | 79.33% |
| 1 | 84.25% |
| 2 | 87.11% |
| 3 | 88.35% |
| 4 | 89.79% |
| 5 | 90.02% |
| 6 | 90.85% |
| 7 | 83.47% |
| 8 | 80.29% |

This is a target-to-response model of the controlled yaw system. The torque-to-response model uses a different input, so its percentage must be reported separately. The evaluation session had already been inspected when this model structure was chosen; collect a new session for independent confirmation.

[Low-speed profile 0 plot](validation_profile_0.svg) | [Profile 8 plot](validation_profile_8.svg) | [Machine-readable results](results.json)
