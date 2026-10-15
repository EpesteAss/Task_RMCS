#!/usr/bin/env python3
"""Rebuild report figures from the archived October 14 Yaw trials."""

from __future__ import annotations

import json
import csv
import sys
from html import escape
from pathlib import Path

import numpy as np

from identify import read_log, trials
from physical_identification import simulate

HERE = Path(__file__).resolve().parent
ANALYSIS = HERE / "physical_analysis"
DATA = HERE / "data/physical_20261014"
OUT = ANALYSIS / "20261014_report_figures"
BLUE = "#1565c0"
ORANGE = "#d35400"
RED = "#c0392b"
GRID = "#dbe3ed"
INK = "#25364d"


def svg_start(width: int, height: int, title: str) -> list[str]:
    return [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            f'<rect width="{width}" height="{height}" fill="white"/>',
            '<g font-family="DejaVu Sans, sans-serif" font-size="17" fill="#25364d">',
            f'<text x="90" y="38" font-size="24" font-weight="bold">{escape(title)}</text>']


def save(name: str, parts: list[str]) -> None:
    (OUT / name).write_text("\n".join(parts + ["</g></svg>"]) + "\n", encoding="utf-8")


def waveform_fit() -> None:
    names = [
        ("0.9 Nm square", "20261014_alternating_09nm_near_121deg_holdout", BLUE),
        ("0.5 Nm square", "20261014_alternating_05nm_holdout", BLUE),
        ("Hann 0.5 Hz +", "20261014_sine_05hz_holdout", RED),
        ("Hann 0.5 Hz reverse", "20261014_sine_reverse_05hz_holdout", RED),
        ("Pure sine 0.5 Hz +", "20261014_pure_sine_05hz_holdout", ORANGE),
        ("Pure sine 0.5 Hz reverse", "20261014_pure_sine_reverse_05hz_holdout", ORANGE),
    ]
    parts = svg_start(1000, 600, "Frozen model: velocity fit across held-out waveforms")
    left, right, top, bottom = 300, 945, 115, 485
    low, high = -60, 100
    px = lambda v: left + (v-low)/(high-low)*(right-left)
    parts += ['<text x="300" y="70">Same 1.3–3.3 s evaluation window; higher is better</text>']
    for tick in (-50, 0, 50, 100):
        x = px(tick)
        parts += [f'<path d="M {x:.1f} {top-12} V {bottom}" stroke="{GRID}"/>',
                  f'<text x="{x:.1f}" y="{bottom+28}" text-anchor="middle">{tick}%</text>']
    zero = px(0)
    parts += [f'<path d="M {zero:.1f} {top-12} V {bottom}" stroke="{INK}" stroke-width="2"/>']
    for index, (label, folder, color) in enumerate(names):
        info = json.loads((ANALYSIS / folder / "validation.json").read_text())
        assert info["time_window_s"] == [1.3, 3.3]
        value = info["metrics"]["velocity_fit_percent"]
        y = top + index*61
        x = px(value)
        parts += [f'<text x="210" y="{y+11}" text-anchor="end">{escape(label)}</text>',
                  f'<rect x="{min(x,zero):.1f}" y="{y-10}" width="{abs(x-zero):.1f}" height="30" rx="3" fill="{color}"/>']
        if value >= 0:
            parts += [f'<text x="{x+9:.1f}" y="{y+11}" font-weight="bold">{value:.1f}%</text>']
        else:
            parts += [f'<text x="{(x+zero)/2:.1f}" y="{y+11}" text-anchor="middle" fill="white" font-weight="bold">{value:.1f}%</text>']
    parts += ['<text x="300" y="555" font-size="15">Blue: square  |  Red: Hann-windowed cosine  |  Orange: symmetric pure sine</text>']
    save("waveform_fit.svg", parts)


def hann_peak() -> None:
    ids = ["20261014_124032_022575", "20261014_124403_440206", "20261014_124448_315562"]
    parts = svg_start(1000, 480, "Hann-windowed cosine: recorded torque extrema")
    parts += ['<text x="90" y="70">The positive and negative peaks change differently with frequency</text>']
    left, right, center = 140, 920, 530
    scale = (right-center)/1.05
    for tick in (-1, -.5, 0, .5, 1):
        x = center + tick*scale
        parts += [f'<path d="M {x:.1f} 105 V 370" stroke="{INK if tick==0 else GRID}" stroke-width="{2 if tick==0 else 1}"/>',
                  f'<text x="{x:.1f}" y="402" text-anchor="middle">{tick:+.1f}</text>']
    for i, folder in enumerate(ids):
        trial = trials(read_log(DATA / folder / "feedback.csv"))[0][0]
        neg, pos = float(np.min(trial["u"])), float(np.max(trial["u"]))
        y = 135 + i*86
        parts += [f'<text x="110" y="{y+17}" text-anchor="end">{["0.5", "1", "2"][i]} Hz</text>',
                  f'<rect x="{center+neg*scale:.1f}" y="{y}" width="{-neg*scale:.1f}" height="30" fill="{RED}"/>',
                  f'<rect x="{center}" y="{y}" width="{pos*scale:.1f}" height="30" fill="{BLUE}"/>',
                  f'<text x="{center+neg*scale-8:.1f}" y="{y+21}" text-anchor="end">{neg:.3f}</text>',
                  f'<text x="{center+pos*scale+8:.1f}" y="{y+21}">{pos:+.3f}</text>']
    parts += ['<text x="530" y="440" text-anchor="middle">Recorded Yaw torque command (Nm)</text>']
    save("hann_torque_peaks.svg", parts)


def step_response() -> None:
    """Compare logged command and motor feedback before using step data to fit."""
    runs = [
        ("+0.9 Nm, 0.4 s", "20261014_081351_499212"),
        ("-0.9 Nm, 0.3 s", "20261014_081918_197079"),
    ]
    parts = svg_start(1000, 650, "Complete positive and negative Yaw step trials")
    parts += [f'<path d="M 90 70 H 112" stroke="{BLUE}" stroke-width="4"/>',
              '<text x="122" y="76">Command torque</text>',
              f'<path d="M 330 70 H 352" stroke="{RED}" stroke-width="4"/>',
              '<text x="362" y="76">Motor torque feedback</text>']

    def panel(left: int, top: int, values: list[tuple[np.ndarray, str]],
              t: np.ndarray, low: float, high: float, label: str) -> None:
        nonlocal parts
        width, height = 360, 165
        px = lambda time: left + time / 4 * width
        py = lambda value: top + (high - value) / (high - low) * height
        parts.append(f'<text x="{left}" y="{top-12}" font-weight="bold">{label}</text>')
        for fraction in np.linspace(0, 1, 5):
            x = left + fraction * width
            y = top + fraction * height
            parts += [f'<path d="M {x:.1f} {top} V {top+height}" stroke="{GRID}"/>',
                      f'<path d="M {left} {y:.1f} H {left+width}" stroke="{GRID}"/>',
                      f'<text x="{x:.1f}" y="{top+height+23}" text-anchor="middle">{4*fraction:.0f}</text>',
                      f'<text x="{left-10}" y="{y+5:.1f}" text-anchor="end">{high-(high-low)*fraction:.1f}</text>']
        selected = np.arange(0, len(t), max(1, len(t)//600))
        for signal, color in values:
            points = " ".join(f"{px(t[k]):.1f},{py(signal[k]):.1f}" for k in selected)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>')

    for column, (title, folder) in enumerate(runs):
        data = read_log(DATA / folder / "feedback.csv")
        active = data["/yaw_experiment/state"] == 3
        t = data["/yaw_experiment/time_s"][active]
        t = t - t[0]
        command = data["/gimbal/yaw/control_torque"][active]
        feedback = data["/gimbal/yaw/torque"][active]
        theta = np.unwrap(data["/gimbal/yaw/angle"][active])
        angle = np.rad2deg(theta - theta[0])
        left = 105 + column * 460
        parts.append(f'<text x="{left}" y="106" font-size="20">{title}</text>')
        panel(left, 145, [(command, BLUE), (feedback, RED)], t,
              -1.2, 1.2, "Torque (Nm, logged motor scale)")
        panel(left, 400, [(angle, BLUE)], t, -20, 20, "Angle change (deg)")
    parts.append('<text x="500" y="635" text-anchor="middle">Time since sweep start (s)</text>')
    save("step_response.svg", parts)


def startup_mismatch() -> None:
    trial = trials(read_log(DATA / "20261014_130051_829626/feedback.csv"))[0][0]
    frozen = json.loads((ANALYSIS / "20261014_steady_window_candidate/frozen_model.json").read_text())
    params = frozen["fit"]["parameters_conditional_on_torque_calibration"]
    prediction = simulate(trial, params)
    t = trial["t"]
    measured = np.rad2deg(trial["theta"])
    predicted = np.rad2deg(prediction[:, 0])
    parts = svg_start(1000, 500, "Pure sine 0.5 Hz: startup mismatch")
    parts += [f'<rect x="570" y="54" width="20" height="4" fill="{BLUE}"/>',
              '<text x="600" y="63">Measured angle</text>',
              f'<rect x="780" y="54" width="20" height="4" fill="{ORANGE}"/>',
              '<text x="810" y="63">Free-run model</text>']
    panels = [("Whole 4 s", 0, 4, -40, 70, 95), ("Startup detail", 0, 1.2, -2, 50, 290)]
    for title, t0, t1, low, high, top in panels:
        left, right, height = 90, 945, 120
        px = lambda v: left + (v-t0)/(t1-t0)*(right-left)
        py = lambda v: top + (high-v)/(high-low)*height
        parts += [f'<text x="90" y="{top-16}" font-weight="bold">{title}</text>']
        for frac in np.linspace(0, 1, 5):
            x = left + frac*(right-left)
            value = t0 + frac*(t1-t0)
            parts += [f'<path d="M {x:.1f} {top} V {top+height}" stroke="{GRID}"/>',
                      f'<text x="{x:.1f}" y="{top+height+23}" text-anchor="middle">{value:.1f}</text>']
        for frac in np.linspace(0, 1, 5):
            y = top + frac*height
            value = high - frac*(high-low)
            parts += [f'<path d="M {left} {y:.1f} H {right}" stroke="{GRID}"/>',
                      f'<text x="{left-12}" y="{y+5:.1f}" text-anchor="end">{value:.0f}</text>']
        selected = np.flatnonzero((t>=t0)&(t<=t1))
        selected = selected[::max(1, len(selected)//800)]
        for series, color in [(measured, BLUE), (predicted, ORANGE)]:
            points = " ".join(f"{px(t[k]):.1f},{py(series[k]):.1f}" for k in selected)
            parts += [f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3"/>']
        parts += [f'<text x="520" y="{top+height+39}" text-anchor="middle">Time since sweep start (s) · angle change (deg)</text>']
    for when in (.25, 1.0):
        k = int(np.argmin(abs(t-when)))
        x = 90 + when/1.2*(945-90)
        parts += [f'<circle cx="{x:.1f}" cy="{290+(50-measured[k])/52*120:.1f}" r="5" fill="{BLUE}"/>',
                  f'<circle cx="{x:.1f}" cy="{290+(50-predicted[k])/52*120:.1f}" r="5" fill="{ORANGE}"/>']
    save("startup_mismatch.svg", parts)


def blind_prediction_compact() -> None:
    """Keep the angle and velocity evidence legible in the PDF."""
    source = ANALYSIS / "20261014_global_narx_holdout/holdout_prediction.csv"
    with source.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    t = np.array([float(row["time_s"]) for row in rows])
    signals = [
        ("Yaw angle (deg)", "measured_angle_deg", "predicted_angle_deg"),
        ("Yaw velocity (rad/s)", "measured_velocity_rad_s", "predicted_velocity_rad_s"),
    ]
    parts = svg_start(1000, 590, "Independent holdout: measured and recursive prediction")
    parts += [f'<path d="M 555 60 H 585" stroke="{BLUE}" stroke-width="4"/>',
              '<text x="595" y="66">Measured</text>',
              f'<path d="M 725 60 H 755" stroke="{ORANGE}" stroke-width="4"/>',
              '<text x="765" y="66">Free-run prediction</text>']
    for panel_index, (label, measured_key, predicted_key) in enumerate(signals):
        top = 115 + panel_index*235
        left, right, height = 90, 945, 155
        measured = np.array([float(row[measured_key]) for row in rows])
        predicted = np.array([float(row[predicted_key]) for row in rows])
        low = float(min(np.min(measured), np.min(predicted)))
        high = float(max(np.max(measured), np.max(predicted)))
        margin = max((high-low)*.08, .01)
        low, high = low-margin, high+margin
        px = lambda seconds: left + seconds/4*(right-left)
        py = lambda value: top + (high-value)/(high-low)*height
        parts.append(f'<text x="{left}" y="{top-15}" font-weight="bold">{label}</text>')
        for fraction in np.linspace(0, 1, 5):
            x = left + fraction*(right-left)
            y = top + fraction*height
            parts += [f'<path d="M {x:.1f} {top} V {top+height}" stroke="{GRID}"/>',
                      f'<path d="M {left} {y:.1f} H {right}" stroke="{GRID}"/>',
                      f'<text x="{x:.1f}" y="{top+height+22}" text-anchor="middle">{4*fraction:.0f}</text>',
                      f'<text x="{left-8}" y="{y+5:.1f}" text-anchor="end">{high-fraction*(high-low):.2f}</text>']
        warm_x = px(.32)
        parts += [f'<path d="M {warm_x:.1f} {top} V {top+height}" stroke="#6a778b" stroke-dasharray="5 5"/>']
        for values, color in ((measured, BLUE), (predicted, ORANGE)):
            points = " ".join(f"{px(t[k]):.1f},{py(values[k]):.1f}" for k in range(len(t)))
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>')
    parts += ['<text x="520" y="567" text-anchor="middle">Time (s) · dashed line: end of 0.32 s measured-history initialization</text>']
    save("blind_prediction_compact.svg", parts)


def main() -> None:
    OUT.mkdir(exist_ok=True)
    waveform_fit()
    step_response()
    hann_peak()
    startup_mismatch()
    blind_prediction_compact()
    print(f"Created five figures in {OUT}")


if __name__ == "__main__":
    main()
