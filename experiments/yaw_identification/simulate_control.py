#!/usr/bin/env python3
"""Model-only screening of baseline and tuned control candidates before hardware tests."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent


class Pid:
    def __init__(self, p: dict, prefix: str):
        self.kp, self.ki, self.kd = (p[f"{prefix}_{x}"] for x in ("kp", "ki", "kd"))
        self.imin, self.imax = (p[f"{prefix}_integral_{x}"] for x in ("min", "max"))
        self.omin, self.omax = (p[f"{prefix}_output_{x}"] for x in ("min", "max"))
        self.integral, self.last = 0.0, None

    def update(self, error: float) -> float:
        output = self.kp * error + self.ki * self.integral
        self.integral = float(np.clip(self.integral + error, self.imin, self.imax))
        if self.last is not None:
            output += self.kd * (error - self.last)
        self.last = error
        return float(np.clip(output, self.omin, self.omax))


def reference(t: float, duration=20.0, amplitude=math.radians(5), frequency=.25, ramp=1.0):
    k, envelope, velocity, acceleration = math.pi/ramp, 1.0, 0.0, 0.0
    if t < ramp:
        envelope = .5*(1-math.cos(k*t)); velocity = .5*k*math.sin(k*t)
        acceleration = .5*k*k*math.cos(k*t)
    elif duration-t < ramp:
        remaining = duration-t
        envelope = .5*(1-math.cos(k*remaining)); velocity = -.5*k*math.sin(k*remaining)
        acceleration = .5*k*k*math.cos(k*remaining)
    omega, sine, cosine = 2*math.pi*frequency, math.sin(2*math.pi*frequency*t), math.cos(2*math.pi*frequency*t)
    return (amplitude*envelope*sine,
            amplitude*(velocity*sine+envelope*omega*cosine),
            amplitude*(acceleration*sine+2*velocity*omega*cosine-envelope*omega*omega*sine))


def simulate(control: dict, plant: tuple[float, float, float], dt=.001, duration=20.0) -> dict:
    angle_pid, velocity_pid = Pid(control, "yaw_angle"), Pid(control, "yaw_velocity")
    angle = velocity = torque = 0.0
    errors, torques, velocities = [], [], []
    a, b, c = plant
    for index in range(round(duration/dt)):
        target, ref_velocity, ref_acceleration = reference(index*dt)
        error = target-angle
        feedforward = (control["yaw_velocity_ff_torque_gain"]*ref_velocity
                       + control["yaw_acceleration_ff_torque_gain"]*ref_acceleration
                       + control["yaw_bias_ff_torque"])
        raw = velocity_pid.update(angle_pid.update(error)-velocity)+feedforward
        limited = float(np.clip(raw, -control["yaw_torque_limit"], control["yaw_torque_limit"]))
        step = control["yaw_torque_slew_Nm_s"]*dt
        torque += float(np.clip(limited-torque, -step, step))
        # RK4 integration with constant torque over one controller interval.
        state = np.array([angle, velocity])
        derivative = lambda x: np.array([x[1], a*x[1]+b*torque+c])
        k1 = derivative(state); k2 = derivative(state+dt*k1/2)
        k3 = derivative(state+dt*k2/2); k4 = derivative(state+dt*k3)
        angle, velocity = state+dt*(k1+2*k2+2*k3+k4)/6
        errors.append(error); torques.append(torque); velocities.append(velocity)
    errors, torques, velocities = map(np.asarray, (errors, torques, velocities))
    return {
        "angle_rmse_deg": math.degrees(float(np.sqrt(np.mean(errors**2)))),
        "angle_p95_abs_deg": math.degrees(float(np.percentile(np.abs(errors), 95))),
        "torque_rms_Nm": float(np.sqrt(np.mean(torques**2))),
        "peak_abs_torque_Nm": float(np.max(np.abs(torques))),
        "peak_abs_velocity_rad_s": float(np.max(np.abs(velocities))),
    }


def main() -> None:
    record = json.loads((HERE/"optimization_record.json").read_text())
    tuned_v2 = yaml.safe_load((HERE/"control_tuned.yaml").read_text())[
        "yaw_experiment"]["ros__parameters"]
    candidates = {
        "baseline": record["baseline"],
        "tuned_v1_candidate": record["tuned_candidate"],
        "tuned_v2_candidate": tuned_v2,
    }
    stable = record["model"]
    plants = {
        "stable_trials_31_70": (stable["a"], stable["b"], stable["c"]),
        "all_trials_model": (-9.95726266021961, 3.410326216708562, -0.16552947779585653),
    }
    result = {plant_name: {
        name: simulate(control, plant)
        for name, control in candidates.items()
    } for plant_name, plant in plants.items()}
    for plant in result.values():
        plant["predicted_change_percent"] = {
            name: {
                key: 100*(plant[name][key]/plant["baseline"][key]-1)
                for key in ("angle_rmse_deg", "angle_p95_abs_deg", "torque_rms_Nm",
                            "peak_abs_torque_Nm")
            }
            for name in ("tuned_v1_candidate", "tuned_v2_candidate")
        }
    output = {
        "status": "model_screening_only_hardware_test_required",
        "models": result,
        "safety_gate": "peak velocity below 1 rad/s and torque within each configured limit",
    }
    (HERE/"control_simulation.json").write_text(json.dumps(output, indent=2)+"\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
