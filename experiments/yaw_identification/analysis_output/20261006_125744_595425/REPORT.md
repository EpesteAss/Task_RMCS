# Yaw identification report

Earlier complete trials train; later complete trials validate; no shared samples
Accepted complete trials: 6; rejected: 0
Excitation profile counts: {'0': 1, '1': 1, '2': 1, '3': 1, '4': 1, '5': 1}

## linear
theta_dot=omega; omega_dot=a*omega+b*u+c

{'a': -5.492649495119752, 'b': 1.9012236132298732, 'c': -0.08832577918521888}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.07088584848008427, 'angle_rmse_deg': 4.166245435482489, 'velocity_fit_percent': -194.10188430807852}

Physical coefficient signs plausible: True

## friction
theta_dot=omega; omega_dot=a*omega+b*u+c+f*tanh(omega/0.02)

{'a': -2.9471240503601766, 'b': 2.2840462532207724, 'c': -0.10647895774054779, 'f': -0.5071948853478762}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.057448551222886585, 'angle_rmse_deg': 4.608455242225836, 'velocity_fit_percent': -138.35120165299358}

Physical coefficient signs plausible: True

## delayed_arx2
theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*u[k-delay]+c

{'p1': 1.90660301084739, 'p2': -0.9085717323227239, 'b': 0.0002541871176722, 'c': -7.665984585500497e-06}

{'simulation_diverged': False, 'velocity_rmse_rad_s': 0.04687882950761154, 'velocity_fit_percent': -94.49794829244765, 'angle_rmse_deg': 1.221941997014212, 'angle_fit_percent': -75.35940203175606}

Physical coefficient signs plausible: True

Selected delay: 1 samples (0.010 s), using only a nested split inside the training trials.
Settled-regime estimation trials: [1, 2, 3, 4]

## filtered_velocity_arx2_one_step
vf[k+1]=q1*vf[k]+q2*vf[k-1]+b*u[k]+c

{'q1': 1.5101142629700024, 'q2': -0.5543682486090956, 'b': 0.011944943960996084, 'c': -0.000553059146669585}

{'velocity_rmse_rad_s': 0.007407171503790087, 'velocity_fit_percent': 66.40161560081754}

Physical coefficient signs plausible: True

This metric predicts only the next sample from measured filtered velocity; it is not a full-trial free-run simulation.

Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests.

A negative validation fit means worse than predicting mean velocity.
Longer logs alone do not prove the model is accurate. Inspect all models before tuning.
Plots saved as SVG (no matplotlib required). Open them in a browser.
