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

## delayed_arx2
theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*u[k-delay]+c

{'p1': 1.8195866243814927, 'p2': -0.8253792040196725, 'b': 0.00037786833388645376, 'c': -7.545385595621298e-05}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.04850260237058315, 'velocity_fit_percent': 63.58262154136054, 'angle_rmse_deg': 0.5891249247304654, 'angle_fit_percent': 45.11170889656402}

Physical coefficient signs plausible: True

Selected delay: 4 samples (0.040 s), using only a nested split inside the training trials.
Settled-regime estimation trials: [30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49]

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect all models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
