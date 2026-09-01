#!/usr/bin/env python3
"""
Interactive time-domain / frequency-domain viewer for fields in an ArduPilot
dataflash (.bin) log. Supplements log_for_claude.py (which dumps a log to
CSV/markdown for reading in an LLM context) with an actual GUI for a human to
pan/zoom a signal, inspect its FFT, and flip between the two views.

Reads the .bin log directly (not the downsampled CSVs log_for_claude.py
writes) so the FFT sees the real sample rate rather than a decimated one.

Usage:
    python Tools/scripts/log_analysis/log_fft_view.py LOGFILE.bin --fields IMU.GyrX,IMU.GyrY
    python Tools/scripts/log_analysis/log_fft_view.py LOGFILE.bin --fields ATT.Roll --start 60 --end 90
    python Tools/scripts/log_analysis/log_fft_view.py LOGFILE.bin --fields TILT.LPos --window hamming --log-mag

Each --fields entry is MSGTYPE.FieldName (see log_for_claude.py's
summary.md, or mavlogdump.py --dump, for what's available in a given log).
Message types that log multiple instances under one name (IMU, BAT, ESC, ...
- anything with an 'I' field) default to whichever instance is seen first,
with a warning printed; use MSGTYPE[N].FieldName, e.g. IMU[0].AccX, to pick
a specific instance or to compare two, e.g. IMU[0].AccX,IMU[1].AccX.

Once the window opens:
  - Top axes: time domain. Bottom axes: frequency domain (magnitude
    spectrum) of whatever is currently visible in the time domain axes -
    zoom/pan the top plot (toolbar magnifying glass / pan tool, or scroll
    wheel) and the FFT below recomputes over just that window, so you can
    isolate an oscillation and see exactly what frequency it's at.
  - "view" radio buttons switch between showing both axes, time only, or
    frequency only.
  - "log mag (dB)" checkbox toggles the frequency axis between linear
    magnitude and dB.
  - checkboxes on the right toggle individual signals on/off when several
    --fields are overlaid.
  - hover the mouse over either plot to read exact (x, y) values off the
    matplotlib toolbar's coordinate readout.
  - the top 10 spectral peaks (by magnitude, descending) of whatever's
    currently visible are marked on the frequency plot with numbered
    triangles, and printed to the console - once at startup, and again for
    the current view any time you press 'p' with the plot window focused.

Requires pymavlink, numpy, scipy and matplotlib (all already dependencies
of the ardupilot build/analysis tooling).
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                 "modules", "mavlink"))

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.widgets import CheckButtons, RadioButtons  # noqa: E402
from scipy.signal import find_peaks  # noqa: E402

from pymavlink import mavutil  # noqa: E402

WINDOWS = {
    "none": np.ones,
    "hann": np.hanning,
    "hamming": np.hamming,
    "blackman": np.blackman,
}
N_PEAKS = 10


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logfile", help="path to .bin dataflash log")
    p.add_argument("--fields", required=True,
                    help="comma-separated MSGTYPE.FieldName (or "
                         "MSGTYPE[instance].FieldName) list, e.g. "
                         "IMU[0].GyrX,IMU[0].GyrY")
    p.add_argument("--start", type=float, default=None,
                    help="only load data after this many seconds from log start")
    p.add_argument("--end", type=float, default=None,
                    help="only load data before this many seconds from log start")
    p.add_argument("--window", choices=sorted(WINDOWS), default="hann",
                    help="FFT window function, to reduce spectral leakage "
                         "(default: hann; use 'none' for a raw rectangular window)")
    p.add_argument("--log-mag", action="store_true",
                    help="start with the frequency axis in dB instead of linear magnitude")
    return p.parse_args()


def load_fields(logfile, field_specs, start, end):
    """Single pass over the log, collecting (t_rel, value) for each
    requested MSGTYPE.FieldName. Returns {spec: (np.array(t), np.array(y))}."""
    spec_re = re.compile(r"^([A-Za-z0-9_]+)(?:\[(\d+)\])?\.(.+)$")
    wanted = {}
    for spec in field_specs:
        m = spec_re.match(spec)
        if not m:
            sys.exit(f"--fields entry '{spec}' isn't MSGTYPE.FieldName "
                     f"or MSGTYPE[instance].FieldName")
        mtype, instance, field = m.group(1), m.group(2), m.group(3)
        instance = int(instance) if instance is not None else None
        wanted.setdefault(mtype, []).append((spec, field, instance))

    series_t = {spec: [] for spec in field_specs}
    series_y = {spec: [] for spec in field_specs}
    # for specs that didn't pin an instance explicitly, lock onto whichever
    # instance is seen first for that spec, the first time it's seen, and
    # warn - a message type with an 'I' field (IMU, BAT, ESC, ...) logs
    # several independent instances under one type name, and interleaving
    # them makes for a meaningless time series and a bogus FFT.
    default_instance = {}
    warned = set()

    mlog = mavutil.mavlink_connection(logfile, dialect="ardupilotmega")
    t0 = None
    while True:
        msg = mlog.recv_match(type=list(wanted.keys()), blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        ts = getattr(msg, "_timestamp", None)
        if t0 is None and ts:
            t0 = ts
        t_rel = (ts - t0) if (ts and t0) else None
        if t_rel is None:
            continue
        if start is not None and t_rel < start:
            continue
        if end is not None and t_rel > end:
            continue
        msg_instance = getattr(msg, "I", None)
        for spec, field, instance in wanted[mtype]:
            if instance is not None:
                if msg_instance is None or msg_instance != instance:
                    continue
            elif msg_instance is not None:
                if spec not in default_instance:
                    default_instance[spec] = msg_instance
                    if spec not in warned:
                        warned.add(spec)
                        print(f"note: {mtype} has multiple instances (field 'I'); "
                              f"defaulting {spec} to instance {msg_instance} - use "
                              f"{mtype}[N].{field} to pick a specific one",
                              file=sys.stderr)
                if msg_instance != default_instance[spec]:
                    continue
            value = getattr(msg, field, None)
            if value is None:
                continue
            series_t[spec].append(t_rel)
            series_y[spec].append(value)

    out = {}
    missing = []
    for spec in field_specs:
        if not series_t[spec]:
            missing.append(spec)
            continue
        out[spec] = (np.array(series_t[spec]), np.array(series_y[spec], dtype=float))
    if missing:
        print(f"warning: no data found for {', '.join(missing)}", file=sys.stderr)
    return out


def compute_fft(t, y, window_name):
    """Resample (t, y) onto a uniform grid (median dt of the samples) and
    return (freq, magnitude). Real ArduPilot log streams are periodic at a
    configured rate but individual timestamps still jitter slightly, so we
    interpolate rather than assume a fixed dt."""
    if len(t) < 8:
        return np.array([]), np.array([])
    dt = np.median(np.diff(t))
    if dt <= 0:
        return np.array([]), np.array([])
    n = int((t[-1] - t[0]) / dt)
    if n < 8:
        return np.array([]), np.array([])
    t_uniform = t[0] + np.arange(n) * dt
    y_uniform = np.interp(t_uniform, t, y)
    y_uniform = y_uniform - np.mean(y_uniform)
    win = WINDOWS[window_name](n)
    spectrum = np.fft.rfft(y_uniform * win)
    freq = np.fft.rfftfreq(n, dt)
    mag = np.abs(spectrum) / max(np.sum(win), 1e-9)
    return freq, mag


def top_peaks(freq, mag, n=N_PEAKS, min_sep_bins=3):
    """Return up to n (freq, mag) local maxima, largest magnitude first.
    min_sep_bins is deliberately a bin count, not an absolute Hz value:
    frequency resolution (Hz/bin) shrinks as the visible time window grows,
    so a fixed Hz separation becomes an enormous number of bins for a long
    window - wide enough to swallow genuinely distinct physical peaks (e.g.
    phugoid vs short-period, half a Hz apart) into a single reported one. A
    small bin count instead just suppresses same-lobe leakage/noise wiggle
    right next to a peak, regardless of window length."""
    if len(freq) < 3:
        return []
    idx, _ = find_peaks(mag, distance=max(1, min_sep_bins))
    idx = idx[idx > 0]  # drop the DC bin
    if len(idx) == 0:
        return []
    idx = idx[np.argsort(mag[idx])[::-1][:n]]
    return [(freq[i], mag[i]) for i in idx]


def main():
    args = parse_args()
    field_specs = [f.strip() for f in args.fields.split(",") if f.strip()]
    data = load_fields(args.logfile, field_specs, args.start, args.end)
    if not data:
        sys.exit("no data loaded for any requested field")

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    fig = plt.figure(figsize=(12, 7))
    fig.canvas.manager.set_window_title(os.path.basename(args.logfile))
    ax_time = fig.add_axes((0.08, 0.55, 0.72, 0.38))
    ax_freq = fig.add_axes((0.08, 0.09, 0.72, 0.38))
    ax_time.set_xlabel("time (s)")
    ax_time.set_ylabel("value")
    ax_freq.set_xlabel("frequency (Hz)")
    ax_freq.set_ylabel("magnitude")
    ax_time.grid(True, alpha=0.3)
    ax_freq.grid(True, alpha=0.3)

    time_lines = {}
    freq_lines = {}
    peak_markers = {}
    peak_labels = {}
    last_peaks = {}
    state = {"log_mag": args.log_mag, "updating": False}

    for i, spec in enumerate(field_specs):
        if spec not in data:
            continue
        color = colors[i % len(colors)]
        t, y = data[spec]
        (tl,) = ax_time.plot(t, y, label=spec, color=color, linewidth=0.8)
        (fl,) = ax_freq.plot([], [], label=spec, color=color, linewidth=0.9)
        (pm,) = ax_freq.plot([], [], marker="v", linestyle="None",
                              markersize=6, color=color, markeredgecolor="black",
                              markeredgewidth=0.5, zorder=5)
        time_lines[spec] = tl
        freq_lines[spec] = fl
        peak_markers[spec] = pm
        peak_labels[spec] = [
            ax_freq.text(0, 0, "", fontsize=7, color=color, ha="center",
                         va="bottom", rotation=90, visible=False, zorder=6)
            for _ in range(N_PEAKS)
        ]

    ax_time.legend(loc="upper right", fontsize=8)
    ax_freq.legend(loc="upper right", fontsize=8)
    # force the autoscale that plot() only *requests* to actually happen now,
    # so the first recompute_freq() below sees the real data range rather
    # than the (0, 1) default an axes starts with before its first draw.
    ax_time.relim()
    ax_time.autoscale_view()

    def mag_for_display(mag):
        if state["log_mag"]:
            return 20.0 * np.log10(np.maximum(mag, 1e-12))
        return mag

    def recompute_freq(_event=None):
        if state["updating"]:
            return
        state["updating"] = True
        xlim = ax_time.get_xlim()
        any_data = False
        for spec, fl in freq_lines.items():
            if not fl.get_visible():
                peak_markers[spec].set_data([], [])
                for label in peak_labels[spec]:
                    label.set_visible(False)
                continue
            t, y = data[spec]
            mask = (t >= xlim[0]) & (t <= xlim[1])
            freq, mag = compute_fft(t[mask], y[mask], args.window)
            fl.set_data(freq, mag_for_display(mag))
            if len(freq):
                any_data = True

            peaks = top_peaks(freq, mag)
            last_peaks[spec] = peaks
            peak_freqs = [f for f, _ in peaks]
            peak_disp = list(mag_for_display(np.array([m for _, m in peaks]))) if peaks else []
            peak_markers[spec].set_data(peak_freqs, peak_disp)
            for i, label in enumerate(peak_labels[spec]):
                if i < len(peaks):
                    label.set_position((peak_freqs[i], peak_disp[i]))
                    label.set_text(f"#{i + 1} {peak_freqs[i]:.2f}Hz")
                    label.set_visible(True)
                else:
                    label.set_visible(False)
        ax_freq.set_ylabel("magnitude (dB)" if state["log_mag"] else "magnitude")
        if any_data:
            ax_freq.relim()
            ax_freq.autoscale_view()
        fig.canvas.draw_idle()
        state["updating"] = False

    ax_time.callbacks.connect("xlim_changed", recompute_freq)

    def print_peaks(_event=None):
        xlim = ax_time.get_xlim()
        print(f"\nTop {N_PEAKS} peaks, visible window {xlim[0]:.2f}s-{xlim[1]:.2f}s:")
        for spec, peaks in last_peaks.items():
            if not freq_lines[spec].get_visible():
                continue
            print(f"  {spec}:")
            if not peaks:
                print("    (no peaks - window too short, or too few samples)")
                continue
            for i, (f, mag) in enumerate(peaks, 1):
                db = 20.0 * np.log10(max(mag, 1e-12))
                print(f"    {i:2d}. {f:9.3f} Hz   mag={mag:12.6g}   ({db:6.1f} dB)")

    fig.canvas.mpl_connect("key_press_event",
                            lambda e: print_peaks() if e.key == "p" else None)

    def on_scroll(event):
        if event.inaxes is not ax_time or event.xdata is None:
            return
        scale = 1 / 1.2 if event.button == "up" else 1.2
        lo, hi = ax_time.get_xlim()
        rel = (event.xdata - lo) / (hi - lo)
        width = (hi - lo) * scale
        ax_time.set_xlim(event.xdata - width * rel, event.xdata + width * (1 - rel))

    fig.canvas.mpl_connect("scroll_event", on_scroll)

    # signal visibility toggles
    ax_check = fig.add_axes((0.83, 0.55, 0.15, 0.38))
    ax_check.axis("off")
    labels = [s for s in field_specs if s in data]
    check = CheckButtons(ax_check, labels, [True] * len(labels))

    def on_check(label):
        time_lines[label].set_visible(not time_lines[label].get_visible())
        freq_lines[label].set_visible(not freq_lines[label].get_visible())
        recompute_freq()
        fig.canvas.draw_idle()

    check.on_clicked(on_check)

    # time / frequency / both view switch
    ax_view = fig.add_axes((0.83, 0.30, 0.15, 0.15))
    ax_view.axis("off")
    view_radio = RadioButtons(ax_view, ("both", "time", "freq"))

    def on_view(label):
        ax_time.set_visible(label in ("both", "time"))
        ax_freq.set_visible(label in ("both", "freq"))
        if label == "time":
            ax_time.set_position((0.08, 0.09, 0.72, 0.84))
        elif label == "freq":
            ax_freq.set_position((0.08, 0.09, 0.72, 0.84))
        else:
            ax_time.set_position((0.08, 0.55, 0.72, 0.38))
            ax_freq.set_position((0.08, 0.09, 0.72, 0.38))
        fig.canvas.draw_idle()

    view_radio.on_clicked(on_view)

    # linear / dB magnitude toggle
    ax_logmag = fig.add_axes((0.83, 0.15, 0.15, 0.08))
    ax_logmag.axis("off")
    logmag_check = CheckButtons(ax_logmag, ["log mag (dB)"], [args.log_mag])

    def on_logmag(_label):
        state["log_mag"] = not state["log_mag"]
        recompute_freq()

    logmag_check.on_clicked(on_logmag)

    recompute_freq()
    print_peaks()
    plt.show()


if __name__ == "__main__":
    main()
