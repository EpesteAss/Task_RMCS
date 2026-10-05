# Yaw identification report

Earlier complete trials train; later complete trials validate; no shared samples
Accepted complete trials: 70; rejected: 1

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -9.95726266021961, 'b': 3.410326216708562, 'c': -0.16552947779585653}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.07963275065856036, 'angle_rmse_deg': 6.712087483380571, 'velocity_fit_percent': 40.2090634997737}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -2.782595315847302, 'b': 4.233244338496617, 'c': -0.1692324523773797, 'f': -1.7173509894882453}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.11373671744314234, 'angle_rmse_deg': 12.527126660099821, 'velocity_fit_percent': 14.602662922631671}

Physical coefficient signs plausible: True

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect both models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
