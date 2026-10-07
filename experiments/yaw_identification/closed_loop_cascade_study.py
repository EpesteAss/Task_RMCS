#!/usr/bin/env python3
"""Offline cascade-model candidate; never modifies the hardware controller."""
import argparse
import json
from pathlib import Path

import numpy as np

from closed_loop_identification import read_config, grouped_arrays
from identify import read_log, trials


def simulate(p, group, limit, friction, substeps=4):
    rate, gain = np.exp(p[:2])
    force = np.exp(p[2]) if friction else 0.0
    bias = p[-1]
    angle = group['angle'][:, 1].copy()
    velocity = group['velocity'][:, 1].copy()
    angles = group['angle'].copy()
    velocities = group['velocity'].copy()
    for k in range(2, group['time'].shape[1]):
        dt = group['time'][:, k] - group['time'][:, k-1]
        # Resolve the fast friction transition inside each 10 ms log sample.
        # One Euler step can produce artificial chatter around zero velocity.
        for _ in range(substeps):
            request = np.clip(gain*(group['reference'][:, k-1]-angle), -limit, limit)
            acceleration = rate*(request-velocity) - force*np.tanh(velocity/.02) + bias
            velocity = velocity + dt/substeps*acceleration
            angle = angle + dt/substeps*velocity
        angles[:, k], velocities[:, k] = angle, velocity
    return angles, velocities


def fit(group, limit, friction, initial=None):
    p = (np.array([np.log(20), np.log(12), -3., 0.]) if friction
         else np.array([np.log(20), np.log(12), 0.])) if initial is None else initial.copy()
    def residual(x):
        return (simulate(x, group, limit, friction)[1][:, 2:]-group['velocity'][:, 2:]).ravel()
    current = residual(p)
    loss = np.mean(current**2)
    lam = .01
    for _ in range(35):
        columns = []
        for j in range(len(p)):
            proposed = p.copy(); proposed[j] += .001
            columns.append((residual(proposed)-current)/.001)
        jac = np.column_stack(columns)
        hessian, gradient = jac.T@jac/len(current), jac.T@current/len(current)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(hessian+lam*np.eye(len(p)), -gradient)
            proposed = p+step
            if (np.max(np.abs(proposed[:-1])) > 5 or abs(proposed[-1]) > 5
                    or proposed[0] > np.log(100)
                    or (friction and proposed[2] > np.log(10))):
                lam *= 5
                continue
            candidate = residual(proposed)
            if np.isfinite(candidate).all() and np.mean(candidate**2) < loss:
                p, current, loss = proposed, candidate, np.mean(candidate**2)
                lam = max(lam/2, 1e-8)
                accepted = True
                break
            lam *= 4
        if not accepted or np.linalg.norm(step) < 1e-6:
            break
    return p


def low_pass(values, time, tau):
    output = values.copy()
    for k in range(1, values.shape[1]):
        dt = time[:, k]-time[:, k-1]
        alpha = dt/(tau+dt)
        output[:, k] = output[:, k-1]+alpha*(values[:, k]-output[:, k-1])
    return output


def metrics(predicted, measured):
    error = predicted-measured
    each = 100*(1-np.sqrt(np.mean(error**2, axis=1))/np.std(measured, axis=1))
    return {'velocity_fit_percent': float(100*(1-np.sqrt(np.mean(error**2))/np.std(measured))),
            'velocity_rmse_rad_s': float(np.sqrt(np.mean(error**2))),
            'per_profile_fit_percent': each.tolist()}


def evaluate(p, group, limit, friction):
    angle, velocity = simulate(p, group, limit, friction)
    raw = metrics(velocity, group['velocity'])
    tau = .05
    filtered = metrics(low_pass(velocity, group['time'], tau),
                       low_pass(group['velocity'], group['time'], tau))
    return {**raw,
            'angle_rmse_deg': float(np.rad2deg(np.sqrt(np.mean((angle-group['angle'])**2)))),
            'filtered_velocity': {'filter_tau_s': tau, **filtered}}


def analyze(first_log, validation_log, config_path):
    config = read_config(config_path)
    for path in (first_log.parent/'config.yaml', validation_log.parent/'config.yaml'):
        if path.exists() and read_config(path) != config:
            raise ValueError(f'Config mismatch: {path}')
    earlier, _ = trials(read_log(first_log))
    later, rejected = trials(read_log(validation_log))
    if len(earlier) < 18 or len(later) < 9 or rejected:
        raise ValueError('Expected 18 training/selection trials and at least 9 complete validation trials')
    # The collector may append sessions to one CSV. Only the newest complete
    # nine-profile session is the validation group.
    groups = [earlier[:9], earlier[9:18], later[-9:]]
    if any([t['profile'] for t in g] != list(range(9)) for g in groups):
        raise ValueError('Expected matching profiles 0 through 8')
    a, b, holdout = [grouped_arrays(g, config) for g in groups]
    combined = {k: np.concatenate((a[k], b[k])) for k in a}
    limit = config['speed_limit_rad_s']
    candidates = []
    for friction in (False, True):
        params = fit(a, limit, friction)
        metrics = evaluate(params, b, limit, friction)
        candidates.append((metrics['velocity_fit_percent'], friction, params, metrics))
    _, friction, params, selection = max(candidates, key=lambda x:x[0])
    params = fit(combined, limit, friction, params)
    return {'input': 'known target angle',
            'model': 'theta_dot=omega; omega_dot=rate*(clip(gain*(reference-theta),+-limit)-omega)-friction*tanh(omega/0.02)+bias',
            'sources': {'training_selection': str(first_log), 'evaluation': str(validation_log)},
            'friction_enabled': friction, 'parameters_log_coordinates': params.tolist(),
            'parameters': {'velocity_response_rate_per_s': float(np.exp(params[0])),
                           'angle_gain_per_s': float(np.exp(params[1])),
                           'friction_acceleration_rad_s2': float(np.exp(params[2])) if friction else 0.,
                           'bias_rad_s2': float(params[-1])},
            'selection': selection, 'validation': evaluate(params, holdout, limit, friction),
            'status': 'Independent only when this script and configuration were frozen before the evaluation session was collected.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first-log', type=Path, required=True)
    parser.add_argument('--validation-log', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('identification_rich.yaml'))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.first_log, args.validation_log, args.config)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
