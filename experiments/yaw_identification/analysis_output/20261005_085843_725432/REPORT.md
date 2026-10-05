# Yaw identification report

Earlier complete trials train; later complete trials validate; no shared samples
Accepted complete trials: 2; rejected: 0

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -10.868417365577217, 'b': 6.215190379681561, 'c': -0.40375404466922477}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.06475932066625156, 'angle_rmse_deg': 6.654912130892776, 'velocity_fit_percent': 70.12251174091834}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -4.773246472996492, 'b': 6.773445808731005, 'c': -0.4221093647243265, 'f': -1.7739793574715317}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.07934712212789824, 'angle_rmse_deg': 7.461625168612873, 'velocity_fit_percent': 63.39225480782951}

Physical coefficient signs plausible: True

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect both models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
