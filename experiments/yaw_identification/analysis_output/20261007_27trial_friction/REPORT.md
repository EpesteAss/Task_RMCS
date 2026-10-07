# Yaw 24 s free-run model refinement

Three groups each contain nine frequency profiles. Group 1 fits candidates; group 2 selects the model; groups 1 and 2 refit coefficients; group 3 tests full-trial free-run prediction.

| Model | Velocity fit | Velocity RMSE (rad/s) | Angle RMSE (deg) |
|---|---:|---:|---:|
| Earlier second-order + torque dead zone | 48.95% | 0.11748 | 5.95 |
| Second-order + torque dead zone + velocity-direction friction | 56.23% | 0.10072 | 4.45 |

Model: theta[k] = p1*theta[k-1] + p2*theta[k-2] + b*phi(u[k-2]) + f*tanh(v[k-1]/0.02) + c; phi(u) = sign(u)*max(abs(u)-0.15,0). v[k-1] comes from the model's two previous angles. Sampling interval is about 0.01 s.

Parameters (p1, p2, b, f, c): 1.98230689, -0.982644283, 0.000277844624, -0.00011026754, -1.09312293e-05

| Profile | Velocity fit |
|---:|---:|
| 0 | -4.87% |
| 1 | 10.84% |
| 2 | 26.87% |
| 3 | 20.42% |
| 4 | 58.99% |
| 5 | 74.60% |
| 6 | 79.37% |
| 7 | 66.28% |
| 8 | 53.82% |

[Profile 8 validation plot](validation_profile_8.svg) | [Machine-readable results](results.json)

Caution: The final session was previously inspected during exploratory analysis. This improves the recorded result, but a new nine-profile session is needed to confirm it without analysis-history bias. Low-frequency profiles remain weak. This is an offline identification model; it does not change the live controller.
