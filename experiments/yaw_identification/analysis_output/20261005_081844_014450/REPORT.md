# Yaw identification report

Earlier complete trials train; later complete trials validate; no shared samples
Accepted complete trials: 2; rejected: 0

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -11.729521843633599, 'b': 5.061979272555779, 'c': -0.34440820344338735}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.08838999085460987, 'angle_rmse_deg': 6.992841166987409, 'velocity_fit_percent': 50.92133684952629}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -5.93549872968052, 'b': 5.806407228153176, 'c': -0.4044848328954776, 'f': -1.6377983004919563}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.10333080451108445, 'angle_rmse_deg': 7.378565052198219, 'velocity_fit_percent': 42.62542965969236}

Physical coefficient signs plausible: True

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect both models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
