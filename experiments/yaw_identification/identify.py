#!/usr/bin/env python3
"""Fit second-order yaw models from complete sweeps, with held-out free-run validation."""
import argparse
from array import array
import csv
import json
import math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def has_completed_sweep(path):
    """Cheaply reject empty/aborted logs before --latest selects one."""
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        names = next(reader, [])
        try:
            state_index = names.index("/yaw_experiment/state")
            completed_index = names.index("/yaw_experiment/completed")
        except ValueError:
            return False
        was_sweeping = False
        for row in reader:
            if len(row) != len(names):
                continue
            try:
                state = float(row[state_index])
                completed = float(row[completed_index])
            except ValueError:
                continue
            if state == 3:
                was_sweeping = True
            elif was_sweeping:
                if state == 2 and completed == 1:
                    return True
                was_sweeping = False
    return False


def read_log(path):
    # Bounded numeric storage: avoid retaining millions of Python string/dict objects.
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        names = next(reader)
        columns = [array("d") for _ in names]
        bad = False
        for row in reader:
            if bad:
                raise ValueError("Malformed row inside CSV; only a truncated final row may be skipped")
            try:
                if len(row) != len(names):
                    raise ValueError("incomplete row")
                values = [float(x) for x in row]
            except ValueError:
                bad = True
                continue
            for col, value in zip(columns, values):
                col.append(value)
    return {name: np.asarray(col) for name, col in zip(names, columns)}


def trials(data):
    states = data["/yaw_experiment/state"]
    active = np.flatnonzero(states == 3)
    result, rejected = [], []
    if not len(active):
        raise ValueError("No sweep samples. Run arm then sweep/session first")
    for number, idx in enumerate(np.split(active, np.flatnonzero(np.diff(active) > 1) + 1), 1):
        after = idx[-1] + 1
        if (after >= len(states) or states[after] != 2
                or data["/yaw_experiment/completed"][after] != 1):
            rejected.append({"trial": number, "reason": "interrupted or incomplete"})
            continue
        t = data["/yaw_experiment/time_s"][idx]
        theta = np.unwrap(data["/gimbal/yaw/angle"][idx])
        omega = data["/gimbal/yaw/velocity_imu"][idx]
        torque = data["/gimbal/yaw/control_torque"][idx]
        dt = np.diff(t)
        if (len(t) < 100 or not all(np.isfinite(v).all() for v in (t, theta, omega, torque))
                or np.any(dt <= 0) or np.max(dt) >= .05 or np.ptp(theta) < .001):
            rejected.append({"trial": number, "reason": "bad timestamps, insufficient movement or nonfinite feedback"})
            continue
        profile = 0
        if "/yaw_experiment/excitation_profile" in data:
            profile = int(round(float(np.median(
                data["/yaw_experiment/excitation_profile"][idx]))))
        result.append({"number": number, "profile": profile,
                       "t": t-t[0], "theta": theta-theta[0],
                       "omega": omega, "u": torque})
    if not result:
        raise ValueError(f"No usable completed sweeps: {rejected}")
    return result, rejected


def sliced(trial, start, stop):
    return {key: value[start:stop] if isinstance(value, np.ndarray) else value
            for key, value in trial.items()}


def fit(training, friction):
    design, response = [], []
    for tr in training:
        t, v, u = tr["t"], tr["omega"], tr["u"]
        step = max(2, round(.05 / np.median(np.diff(t))))
        # Integral regression avoids differentiating noisy IMU samples at 100 Hz.
        for i in range(0, len(t)-step, step):
            j = i+step
            dt = np.diff(t[i:j+1])
            row = [np.sum(.5*(v[i:j]+v[i+1:j+1])*dt),
                   np.sum(u[i:j]*dt), t[j]-t[i]]
            if friction:
                sign = np.tanh(v[i:j+1]/.02)
                row.append(np.sum(.5*(sign[:-1]+sign[1:])*dt))
            design.append(row)
            response.append(v[j]-v[i])
    x, y = np.asarray(design), np.asarray(response)
    scale = np.linalg.norm(x, axis=0)
    if len(x) < 20 or np.any(scale < 1e-10) or np.linalg.matrix_rank(x/scale) < x.shape[1]:
        raise ValueError("Insufficient independent excitation for regression")
    coeff = np.linalg.lstsq(x/scale, y, rcond=None)[0] / scale
    return coeff, float(np.linalg.cond(x/scale)), len(x)


def simulate(tr, coeff):
    a, b, c = coeff[:3]
    f = coeff[3] if len(coeff) == 4 else 0.0
    state = np.array([tr["theta"][0], tr["omega"][0]], dtype=float)
    trajectory = [state.copy()]
    for i, dt in enumerate(np.diff(tr["t"])):
        count = max(1, math.ceil(dt/.002))
        h, u = dt/count, tr["u"][i]
        def derivative(s):
            return np.array([s[1], a*s[1]+b*u+c+f*np.tanh(s[1]/.02)])
        for _ in range(count):
            k1 = derivative(state)
            k2 = derivative(state + h*k1/2)
            k3 = derivative(state + h*k2/2)
            k4 = derivative(state + h*k3)
            state += h*(k1+2*k2+2*k3+k4)/6
            if not np.isfinite(state).all() or np.max(np.abs(state)) > 1e6:
                return None
        trajectory.append(state.copy())
    return np.asarray(trajectory)


def evaluate(validation, coeff):
    errors_v, errors_a, actual_v, plots = [], [], [], []
    for tr in validation:
        pred = simulate(tr, coeff)
        if pred is None:
            return {"simulation_diverged": True}, []
        errors_v.extend(pred[:, 1]-tr["omega"])
        errors_a.extend(pred[:, 0]-tr["theta"])
        actual_v.extend(tr["omega"])
        plots.append((tr, pred))
    ev, ea, v = np.asarray(errors_v), np.asarray(errors_a), np.asarray(actual_v)
    rmse = float(np.sqrt(np.mean(ev**2)))
    return {"simulation_diverged": False, "velocity_rmse_rad_s": rmse,
            "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(ea**2)))),
            "velocity_fit_percent": float(100*(1-rmse/max(float(np.std(v)), 1e-12)))}, plots


def fit_delayed_arx2(training, delay_samples):
    """Fit theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*u[k-d]+c."""
    design, response = [], []
    first = max(2, delay_samples)
    for tr in training:
        theta, torque = tr["theta"], tr["u"]
        for k in range(first, len(theta)):
            design.append([theta[k-1], theta[k-2], torque[k-delay_samples], 1.0])
            response.append(theta[k])
    x, y = np.asarray(design), np.asarray(response)
    scale = np.linalg.norm(x, axis=0)
    if len(x) < 20 or np.any(scale < 1e-10) or np.linalg.matrix_rank(x/scale) < x.shape[1]:
        raise ValueError("Insufficient independent excitation for delayed ARX regression")
    coeff = np.linalg.lstsq(x/scale, y, rcond=None)[0] / scale
    return coeff, float(np.linalg.cond(x/scale)), len(x)


def simulate_delayed_arx2(tr, coeff, delay_samples):
    p1, p2, b, c = coeff
    first = max(2, delay_samples)
    theta = np.empty_like(tr["theta"])
    theta[:first] = tr["theta"][:first]
    for k in range(first, len(theta)):
        theta[k] = p1*theta[k-1] + p2*theta[k-2] + b*tr["u"][k-delay_samples] + c
        if not np.isfinite(theta[k]) or abs(theta[k]) > 1e6:
            return None
    omega = np.gradient(theta, tr["t"])
    return np.column_stack((theta, omega))


def evaluate_delayed_arx2(validation, coeff, delay_samples):
    errors_v, errors_a, actual_v, actual_a, plots = [], [], [], [], []
    for tr in validation:
        pred = simulate_delayed_arx2(tr, coeff, delay_samples)
        if pred is None:
            return {"simulation_diverged": True}, []
        errors_v.extend(pred[:, 1]-tr["omega"])
        errors_a.extend(pred[:, 0]-tr["theta"])
        actual_v.extend(tr["omega"])
        actual_a.extend(tr["theta"])
        plots.append((tr, pred))
    ev, ea = np.asarray(errors_v), np.asarray(errors_a)
    velocity, angle = np.asarray(actual_v), np.asarray(actual_a)
    vrmse, armse = float(np.sqrt(np.mean(ev**2))), float(np.sqrt(np.mean(ea**2)))
    return {
        "simulation_diverged": False,
        "velocity_rmse_rad_s": vrmse,
        "velocity_fit_percent": float(100*(1-vrmse/max(float(np.std(velocity)), 1e-12))),
        "angle_rmse_deg": float(np.rad2deg(armse)),
        "angle_fit_percent": float(100*(1-armse/max(float(np.std(angle)), 1e-12))),
    }, plots


def filtered_velocity(trial, tau=0.012):
    """Apply the same first-order velocity filter used by the experiment controller."""
    measured = trial["omega"]
    result = np.empty_like(measured)
    result[0] = measured[0]
    for k, dt in enumerate(np.diff(trial["t"]), 1):
        alpha = dt / (tau + dt)
        result[k] = result[k-1] + alpha * (measured[k] - result[k-1])
    return result


def fit_filtered_velocity_arx2(training, tau=0.012):
    """Fit vf[k+1]=q1*vf[k]+q2*vf[k-1]+b*u[k]+c."""
    design, response = [], []
    for tr in training:
        velocity = filtered_velocity(tr, tau)
        for k in range(1, len(velocity)-1):
            design.append([velocity[k], velocity[k-1], tr["u"][k], 1.0])
            response.append(velocity[k+1])
    x, y = np.asarray(design), np.asarray(response)
    scale = np.linalg.norm(x, axis=0)
    if len(x) < 20 or np.any(scale < 1e-10) or np.linalg.matrix_rank(x/scale) < x.shape[1]:
        raise ValueError("Insufficient independent excitation for filtered velocity ARX2")
    coeff = np.linalg.lstsq(x/scale, y, rcond=None)[0] / scale
    return coeff, float(np.linalg.cond(x/scale)), len(x)


def evaluate_filtered_velocity_arx2(validation, coeff, tau=0.012):
    errors, actual, plots = [], [], []
    for tr in validation:
        velocity = filtered_velocity(tr, tau)
        predicted = (coeff[0] * velocity[1:-1] + coeff[1] * velocity[:-2]
                     + coeff[2] * tr["u"][1:-1] + coeff[3])
        measured = velocity[2:]
        errors.extend(predicted - measured)
        actual.extend(measured)
        plots.append({"trial": tr["number"], "time": tr["t"][2:],
                      "measured": measured, "predicted": predicted})
    error, measured = np.asarray(errors), np.asarray(actual)
    rmse = float(np.sqrt(np.mean(error**2)))
    return {
        "velocity_rmse_rad_s": rmse,
        "velocity_fit_percent": float(100 * (1 - rmse/max(float(np.std(measured)), 1e-12))),
    }, plots


def select_arx_delay(training, maximum_delay=6):
    """Select delay inside the training set, preserving final validation isolation."""
    recent = training[-min(20, len(training)):]
    if len(recent) < 4:
        return 0, recent, []
    cut = len(recent)//2
    estimation, selection = recent[:cut], recent[cut:]
    candidates = []
    for delay in range(maximum_delay+1):
        coeff, _, _ = fit_delayed_arx2(estimation, delay)
        poles = np.roots([1.0, -coeff[0], -coeff[1]])
        metrics, _ = evaluate_delayed_arx2(selection, coeff, delay)
        candidates.append({"delay_samples": delay,
                           "stable": bool(np.max(np.abs(poles)) < 1.0),
                           "velocity_fit_percent": metrics.get("velocity_fit_percent", -math.inf),
                           "angle_fit_percent": metrics.get("angle_fit_percent", -math.inf)})
    usable = [item for item in candidates if item["stable"]]
    if not usable:
        raise ValueError("No stable delayed ARX candidate on the training-only selection split")
    selected = max(usable, key=lambda item: item["velocity_fit_percent"])
    return selected["delay_samples"], recent, candidates


def analyze(path, out):
    accepted, rejected = trials(read_log(path))
    out.mkdir(parents=True, exist_ok=True)
    if len(accepted) >= 2:
        cut = min(len(accepted)-1, max(1, int(.7*len(accepted))))
        training, validation = accepted[:cut], accepted[cut:]
        split_note = "Earlier complete trials train; later complete trials validate; no shared samples"
    else:
        cut = int(.7*len(accepted[0]["t"]))
        training = [sliced(accepted[0], 0, cut)]
        validation = [sliced(accepted[0], cut, None)]
        split_note = "Only one trial: first 70% trains, last 30% validates; independent trial still needed"
    profile_counts = {str(profile): sum(t["profile"] == profile for t in accepted)
                      for profile in sorted({t["profile"] for t in accepted})}
    report = {"source": str(path.resolve()), "accepted_trials": len(accepted), "rejected_trials": rejected,
              "split": split_note, "training_trials": [t["number"] for t in training],
              "validation_trials": [t["number"] for t in validation],
              "excitation_profile_counts": profile_counts, "models": {},
              "note": "Second-order angle model; torque input is commanded Nm, not independently calibrated shaft torque. Closed-loop noise/friction can bias estimates. Do not deploy gains from these fits without separate matched tests."}
    all_plots = {}
    for name, friction in (("linear", False), ("friction", True)):
        coeff, condition, windows = fit(training, friction)
        metrics, predictions = evaluate(validation, coeff)
        physically_plausible = bool(coeff[0] < 0 and coeff[1] > 0 and (not friction or coeff[3] <= 0))
        report["models"][name] = {
            "equations": "theta_dot=omega; omega_dot=a*omega+b*u+c" + ("+f*tanh(omega/0.02)" if friction else ""),
            "coefficients": dict(zip(("a", "b", "c", "f"), map(float, coeff))),
            "normalized_design_condition": condition, "training_windows": windows,
            "physical_signs_plausible": physically_plausible, "validation": metrics,
            "interpretation": "Candidate only; inspect held-out free-run plots and repeatability"}
        all_plots[name] = predictions
    # The fixed-parameter continuous model is retained for physical
    # interpretation.  This discrete second-order model additionally captures
    # command/measurement delay and the later, settled operating regime.  Its
    # delay is selected using a split wholly inside the training trials; the
    # final validation trials remain untouched.
    delay, arx_training, delay_candidates = select_arx_delay(training)
    arx_coeff, arx_condition, arx_samples = fit_delayed_arx2(arx_training, delay)
    arx_metrics, arx_predictions = evaluate_delayed_arx2(validation, arx_coeff, delay)
    arx_poles = np.roots([1.0, -arx_coeff[0], -arx_coeff[1]])
    sample_period = float(np.median(np.concatenate([np.diff(t["t"]) for t in arx_training])))
    report["models"]["delayed_arx2"] = {
        "equations": "theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*u[k-delay]+c",
        "coefficients": dict(zip(("p1", "p2", "b", "c"), map(float, arx_coeff))),
        "poles": [[float(x.real), float(x.imag)] for x in arx_poles],
        "delay_samples": int(delay),
        "delay_seconds": delay*sample_period,
        "delay_selection": "training-only nested split; best held-in velocity fit",
        "delay_candidates": delay_candidates,
        "estimation_trials": [t["number"] for t in arx_training],
        "normalized_design_condition": arx_condition,
        "training_samples": arx_samples,
        "physical_signs_plausible": bool(arx_coeff[2] > 0 and np.max(np.abs(arx_poles)) < 1),
        "validation": arx_metrics,
        "interpretation": "Predictive model for the settled operating regime; continuous model remains the controller-design model",
    }
    all_plots["delayed_arx2"] = arx_predictions
    # A one-step predictor answers a different question from free-run
    # simulation: given the current measured state, how accurately can the
    # model predict the next 10 ms sample?  Filtering uses the same 12 ms time
    # constant as the controller and the final validation trials remain held
    # out.  Report it separately so it cannot be mistaken for a 20 s free run.
    filtered_tau = 0.012
    filtered_coeff, filtered_condition, filtered_samples = fit_filtered_velocity_arx2(
        training, filtered_tau)
    filtered_metrics, filtered_plots = evaluate_filtered_velocity_arx2(
        validation, filtered_coeff, filtered_tau)
    filtered_poles = np.roots([1.0, -filtered_coeff[0], -filtered_coeff[1]])
    report["models"]["filtered_velocity_arx2_one_step"] = {
        "equations": "vf[k+1]=q1*vf[k]+q2*vf[k-1]+b*u[k]+c",
        "coefficients": dict(zip(("q1", "q2", "b", "c"), map(float, filtered_coeff))),
        "velocity_filter_tau_s": filtered_tau,
        "prediction_horizon_s": sample_period,
        "normalized_design_condition": filtered_condition,
        "training_samples": filtered_samples,
        "physical_signs_plausible": bool(filtered_coeff[2] > 0
                                          and np.max(np.abs(filtered_poles)) < 1),
        "validation": filtered_metrics,
        "interpretation": ("Strict held-out one-step prediction of controller-filtered velocity; "
                           "not comparable to full-trial free-run fit"),
    }
    # Always emit this dependency-free plot.  The one-step predictor has a
    # different data shape from the free-run simulations below and should not
    # be forced through the free-run plotting interface.
    if filtered_plots:
        from svg_plot import one_step_validation_plot
        one_step_validation_plot(out/"filtered_velocity_arx2_one_step_validation.svg",
                                 filtered_plots[-1])
    (out/"results.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    lines = ["# Yaw identification report", "", split_note,
             f"Accepted complete trials: {len(accepted)}; rejected: {len(rejected)}",
             f"Excitation profile counts: {profile_counts}", ""]
    for name, model in report["models"].items():
        lines.extend([f"## {name}", model["equations"], "", str(model["coefficients"]),
                      "", str(model["validation"]), "",
                      f"Physical coefficient signs plausible: {model['physical_signs_plausible']}", ""])
        if name == "delayed_arx2":
            lines.extend([f"Selected delay: {model['delay_samples']} samples "
                          f"({model['delay_seconds']:.3f} s), using only a nested split "
                          "inside the training trials.",
                          f"Settled-regime estimation trials: {model['estimation_trials']}", ""])
        elif name == "filtered_velocity_arx2_one_step":
            lines.extend(["This metric predicts only the next sample from measured filtered velocity; "
                          "it is not a full-trial free-run simulation.", ""])
    lines.extend([report["note"], "", "A negative validation fit means worse than predicting mean velocity.",
                  "Longer logs alone do not prove the model is accurate. Inspect all models before tuning."])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for name, predictions in all_plots.items():
            if not predictions:
                continue
            tr, pred = predictions[-1]
            t = tr["t"]-tr["t"][0]
            fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
            axes[0].plot(t, np.rad2deg(tr["theta"]), label="measured")
            axes[0].plot(t, np.rad2deg(pred[:, 0]), label="free-run model")
            axes[0].set_ylabel("Yaw (deg)")
            axes[1].plot(t, tr["omega"], label="measured")
            axes[1].plot(t, pred[:, 1], label="free-run model")
            axes[1].set_ylabel("Velocity (rad/s)")
            axes[2].plot(t, tr["u"], label="total torque command")
            axes[2].set(ylabel="Torque (Nm)", xlabel="Time (s)")
            for ax in axes:
                ax.grid(True); ax.legend()
            fig.tight_layout(); fig.savefig(out/f"{name}_validation.png", dpi=150); plt.close(fig)
    except ImportError:
        from svg_plot import validation_plot
        for name, predictions in all_plots.items():
            if predictions:
                validation_plot(out/f"{name}_validation.svg", *predictions[-1])
        lines.append("Plots saved as SVG (no matplotlib required). Open them in a browser.")
    (out/"REPORT.md").write_text("\n".join(lines)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path)
    parser.add_argument("--latest", action="store_true")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    path = args.csv
    if args.latest:
        candidates = list((HERE/"data").glob("*/feedback.csv")) + list((HERE/"data").glob("*.csv"))
        if not candidates:
            parser.error("No recordings found in data/")
        path = next((candidate for candidate in
                     sorted(candidates, key=lambda p: p.stat().st_mtime, reverse=True)
                     if has_completed_sweep(candidate)), None)
        if path is None:
            parser.error("No recording contains a completed sweep")
    if path is None:
        parser.error("Supply a CSV path or --latest")
    out = args.out or HERE/"analysis_output"/(path.parent.name if path.name == "feedback.csv" else path.stem)
    report = analyze(path, out)
    print(json.dumps(report, indent=2))
    print(f"Report and plots: {out}")


if __name__ == "__main__":
    main()
