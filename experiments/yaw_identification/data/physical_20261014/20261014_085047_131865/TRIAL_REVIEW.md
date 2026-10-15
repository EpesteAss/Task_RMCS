# Fixed-Pitch alternating Yaw pulse trial — 2026-10-14 08:51 UTC

Pitch was mechanically clamped. One 4 s torque-only sweep applied three negative/positive pulse pairs: each nonzero interval was 0.4 s at a requested magnitude of 0.9 Nm, separated by 0.1 s of zero request. The actual output was limited to 20 Nm/s slew. The sweep started near 92.82° absolute Yaw.

| Sweep time (s) | Requested torque | Yaw range relative to sweep start |
| --- | ---: | ---: |
| 0.3–0.7 | −0.9 Nm | 0 to −8.7° |
| 0.8–1.2 | +0.9 Nm | −8.7° to +12.6°; peak later +14.8° |
| 1.3–1.7 | −0.9 Nm | +14.8° to −0.3° |
| 1.8–2.2 | +0.9 Nm | −1.5° to +16.3° |
| 2.3–2.7 | −0.9 Nm | +18.1° to +1.8° |
| 2.8–3.2 | +0.9 Nm | +0.3° to +17.0° |

Coasting after the last pulse ended at about +20.0°. The full observed range was −8.7° to +20.1° relative to the start. Maximum absolute IMU Yaw speed was 1.153 rad/s, below the 2.88 rad/s soft guard and 3.6 rad/s hard trip. Yaw torque command ranged from −0.9 to +0.9 Nm. Pitch command stayed exactly zero in the CSV. The trial completed without fault or guard stop; the session confirmed OFF and both commands zero. The experiment process was then stopped.

The encoder shows repeated reversal, so the mechanical Yaw axis did move back and forth. The trial ended about +20° from its start. Most of that net drift arose in the first pulse pair and the final zero-torque coast; individual pulse displacement also depends on entry velocity. Full analysis, including symmetric command impulse and failed held-out model predictions: [REPORT.md](../../physical_analysis/20261014_alternating_holdout/REPORT.md).

Files: `feedback.csv` (raw 100 Hz record), `config.yaml` (actual run settings), `session.json` (session outcome and OFF confirmation).
