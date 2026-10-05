# Yaw identification report

Only one trial: first 70% trains, last 30% validates; independent trial still needed
Accepted complete trials: 1; rejected: 0

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -8.357417301214172, 'b': 3.693269735133735, 'c': -0.4338534886722907}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.07512980995150939, 'angle_rmse_deg': 2.554346343800284, 'velocity_fit_percent': 57.0676223392723}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -5.429263175794688, 'b': 4.206399006284806, 'c': -0.5012180142267566, 'f': -0.9420432431604674}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.06984066205975481, 'angle_rmse_deg': 2.6578246141706616, 'velocity_fit_percent': 60.0900670245285}

Physical coefficient signs plausible: True

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect both models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
