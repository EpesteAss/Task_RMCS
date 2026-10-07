# Yaw identification report

Earlier complete trials train; later complete trials validate; no shared samples
Accepted complete trials: 18; rejected: 0
Excitation profile counts: {'0': 3, '1': 3, '2': 3, '3': 3, '4': 3, '5': 3}

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -3.0343392176499537, 'b': 1.4035838937917096, 'c': -7.676246084818253e-05}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.1006940422906652, 'angle_rmse_deg': 13.528237623136045, 'velocity_fit_percent': -20.018200571705513}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -1.630376005541413, 'b': 1.639778475108051, 'c': 0.010846238266118234, 'f': -0.3344653152037768}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.08413057199465625, 'angle_rmse_deg': 8.155966520274111, 'velocity_fit_percent': -0.2760405101248198}

Physical coefficient signs plausible: True

## delayed_arx2
theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*u[k-delay]+c

{'p1': 1.9416749197935042, 'p2': -0.9432806190058988, 'b': 0.0001871025587298044, 'c': 4.620588500257213e-06}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.06628995986345508, 'velocity_fit_percent': 20.988357227565402, 'angle_rmse_deg': 1.1744624568788236, 'angle_fit_percent': -23.315723997080884}

Physical coefficient signs plausible: True

Selected delay: 1 samples (0.010 s), using only a nested split inside the training trials.
Settled-regime estimation trials: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

## filtered_velocity_arx2_one_step
vf[k+1]=q1*vf[k]+q2*vf[k-1]+b*u[k]+c

{'q1': 1.6085900478434, 'q2': -0.637960189278951, 'b': 0.008636639311504511, 'c': 9.145787136296853e-06}

{'velocity_rmse_rad_s': 0.00799237500952762, 'velocity_fit_percent': 90.26863544486562}

Physical coefficient signs plausible: True

This metric predicts only the next sample from measured filtered velocity; it is not a full-trial free-run simulation.

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect all models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
