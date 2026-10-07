# Yaw 24 s free-run model refinement

Three groups each contain nine frequency profiles. Group 1 fits candidates; group 2 selects the model; groups 1 and 2 refit coefficients; group 3 tests full-trial free-run prediction.

| Model | Velocity fit | Velocity RMSE (rad/s) | Angle RMSE (deg) |
|---|---:|---:|---:|
| Earlier second-order + torque dead zone | 48.95% | 0.11748 | 5.95 |
| Second-order + torque dead zone + velocity-direction friction | 58.48% | 0.09548 | 4.12 |

Model: theta[k] = p1*theta[k-1] + p2*theta[k-2] + b*phi(u[k-2]) + f*tanh(v[k-1]/0.02) + c; phi(u) = sign(u)*max(abs(u)-0.15,0). v[k-1] comes from the model's two previous angles. Sampling interval is about 0.01 s.

Parameters (p1, p2, b, f, c): 1.98230689, -0.982644283, 0.000277844624, -0.00011026754, -1.09312293e-05

| Profile | Velocity fit |
|---:|---:|
| 0 | -7.32% |
| 1 | 15.28% |
| 2 | 32.76% |
| 3 | 41.95% |
| 4 | 60.72% |
| 5 | 75.03% |
| 6 | 77.60% |
| 7 | 65.73% |
| 8 | 54.82% |

[Profile 8 validation plot](validation_profile_8.svg) | [Machine-readable results](results.json)

Low-frequency profiles remain weak. Treat the third group as independent evidence only when it was collected after the model structure and selection grid were frozen. This is an offline identification model; it does not change the live controller.
