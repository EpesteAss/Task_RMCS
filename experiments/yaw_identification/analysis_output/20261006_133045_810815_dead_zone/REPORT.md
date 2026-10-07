# 24 s free-run yaw prediction: exploratory dead-zone model

Three chronological groups of 6 trials: group 1 fits candidates, group 2 selects the dead zone and delay, groups 1–2 refit, and group 3 compares. All groups contain the same profiles.

| Model | 24 s velocity fit | Velocity RMSE (rad/s) | Angle RMSE (deg) |
|---|---:|---:|---:|
| Original delayed second-order ARX | 20.99% | 0.06629 | 1.174 |
| Second-order ARX + torque dead zone | 26.30% | 0.06183 | 1.131 |

Selected dead zone: 0.20 N·m; delay: 2 samples. Middle-six selection fit: 28.90%.

| Profile | Measured velocity std (rad/s) | Original velocity fit | Dead-zone velocity fit |
|---:|---:|---:|---:|
| 0 | 0.0532 | -18.31% | -22.34% |
| 1 | 0.1412 | 37.04% | 43.68% |
| 2 | 0.1342 | 36.96% | 43.83% |
| 3 | 0.0175 | -139.51% | -97.95% |
| 4 | 0.0158 | -212.72% | -203.21% |
| 5 | 0.0301 | -86.55% | -88.19% |

The dead zone represents an effective low-torque response and is not an independently calibrated friction measurement. The final six trials had already been inspected for the original model, so the comparison is exploratory. A new session is required to confirm generalization.

Three low-dynamic profiles still have negative free-run velocity fit. The modest pooled improvement does not solve long-horizon prediction across all regimes.

[Representative validation plot](validation_profile_1.svg) and [machine-readable results](results.json).
