"""Dependency-free static plots for deployment containers without matplotlib."""
from html import escape
import numpy as np


def validation_plot(path, trial, prediction):
    t = trial["t"] - trial["t"][0]
    panels = [
        ("Yaw (deg)", [("Measured", np.rad2deg(trial["theta"])),
                       ("Free-run model", np.rad2deg(prediction[:, 0]))]),
        ("Velocity (rad/s)", [("Measured", trial["omega"]), ("Free-run model", prediction[:, 1])]),
        ("Torque (Nm)", [("Total command", trial["u"])])]
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="880" viewBox="0 0 1000 880">',
           '<rect width="1000" height="880" fill="white"/>',
           '<g font-family="sans-serif" font-size="13" fill="#222">',
           '<text x="80" y="25" font-size="18">Held-out trial: measured response and free-run prediction</text>']
    duration = max(float(t[-1]), 1e-9)
    colors = ["#1565c0", "#d35400"]
    for panel, (label, traces) in enumerate(panels):
        left, top, width, height = 90, 65 + panel*265, 860, 190
        low = min(float(np.min(v)) for _, v in traces)
        high = max(float(np.max(v)) for _, v in traces)
        margin = max((high-low)*.08, .001)
        low, high = low-margin, high+margin
        svg.append(f'<text x="{left}" y="{top-18}">{escape(label)}</text>')
        for j in range(6):
            y, value = top+height-j*height/5, low+(high-low)*j/5
            x, seconds = left+j*width/5, duration*j/5
            svg.append(f'<path d="M {left} {y} H {left+width} M {x} {top} V {top+height}" stroke="#ddd" fill="none"/>')
            svg.append(f'<text x="{left-8}" y="{y+4}" text-anchor="end">{value:.3g}</text>')
            svg.append(f'<text x="{x}" y="{top+height+20}" text-anchor="middle">{seconds:.3g}</text>')
        idx = np.unique(np.linspace(0, len(t)-1, min(2000, len(t))).astype(int))
        for i, (name, values) in enumerate(traces):
            points = " ".join(f"{left+t[k]/duration*width:.2f},{top+(high-values[k])/(high-low)*height:.2f}" for k in idx)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{colors[i]}" stroke-width="1.3"/>')
            svg.append(f'<text x="{left+420+i*170}" y="{top-18}" fill="{colors[i]}">{escape(name)}</text>')
        svg.append(f'<text x="{left+width/2}" y="{top+height+42}" text-anchor="middle">Time (s)</text>')
    svg.append('</g></svg>')
    path.write_text("\n".join(svg)+"\n")


def one_step_validation_plot(path, prediction):
    """Plot a held-out one-step velocity prediction with explicit axes."""
    t = prediction["time"] - prediction["time"][0]
    measured = prediction["measured"]
    predicted = prediction["predicted"]
    panels = [
        ("Filtered velocity (rad/s)", [("Measured", measured),
                                        ("One-step prediction", predicted)]),
        ("Prediction error (rad/s)", [("Predicted - measured", predicted-measured)]),
    ]
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="620" viewBox="0 0 1000 620">',
           '<rect width="1000" height="620" fill="white"/>',
           '<g font-family="sans-serif" font-size="13" fill="#222">',
           f'<text x="80" y="25" font-size="18">Held-out profile {prediction["trial"]}: 10 ms one-step prediction</text>']
    duration = max(float(t[-1]), 1e-9)
    colors = ["#1565c0", "#d35400"]
    for panel, (label, traces) in enumerate(panels):
        left, top, width, height = 90, 75 + panel*270, 860, 190
        low = min(float(np.min(v)) for _, v in traces)
        high = max(float(np.max(v)) for _, v in traces)
        margin = max((high-low)*.08, .0005)
        low, high = low-margin, high+margin
        svg.append(f'<text x="{left}" y="{top-18}">{escape(label)}</text>')
        for j in range(6):
            y, value = top+height-j*height/5, low+(high-low)*j/5
            x, seconds = left+j*width/5, duration*j/5
            svg.append(f'<path d="M {left} {y} H {left+width} M {x} {top} V {top+height}" stroke="#ddd" fill="none"/>')
            svg.append(f'<text x="{left-8}" y="{y+4}" text-anchor="end">{value:.3g}</text>')
            svg.append(f'<text x="{x}" y="{top+height+20}" text-anchor="middle">{seconds:.3g}</text>')
        idx = np.unique(np.linspace(0, len(t)-1, min(2000, len(t))).astype(int))
        for i, (name, values) in enumerate(traces):
            points = " ".join(
                f"{left+t[k]/duration*width:.2f},{top+(high-values[k])/(high-low)*height:.2f}"
                for k in idx)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{colors[i]}" stroke-width="1.3"/>')
            svg.append(f'<text x="{left+430+i*190}" y="{top-18}" fill="{colors[i]}">{escape(name)}</text>')
        svg.append(f'<text x="{left+width/2}" y="{top+height+42}" text-anchor="middle">Time (s)</text>')
    svg.append('</g></svg>')
    path.write_text("\n".join(svg)+"\n")
