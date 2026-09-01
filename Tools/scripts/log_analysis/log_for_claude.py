#!/usr/bin/env python3
"""
Turn an ArduPilot dataflash (.bin) log into a small set of text/CSV files
that are cheap to read in an LLM context window, so params and behaviour in
the log can be discussed against the source tree.

By default it writes ONE CSV per message type that actually appears in the
log (TILT, CTUN, QTUN, PSCD, RATE, PIQP/PIQR/PIQY, BAT, IMU, AETR, ... -
whatever the log has), because it's rarely obvious in advance which field is
going to matter for a given bug. Each CSV is independently downsampled to
--max-rows, so asking for everything doesn't mean any one file is huge - the
cost of "give me everything" is more small files, not bigger ones. Pass
--messages to restrict to a subset once you know what you need.

It writes, into an output directory:
  summary.md    - log time range, firmware/board info (MSG), flight mode
                  timeline, EV/ERR events, a parameter section (params that
                  changed in-flight always; full param list with
                  --full-params), and a manifest of every message type seen
                  with its field names and row counts.
  <TYPE>.csv    - one file per message type, one row per sample, with a Ts
                  (seconds since log start) column prepended. Long series
                  are uniformly downsampled to --max-rows so a 400 Hz log
                  doesn't turn into a multi-MB file.

Usage:
    python Tools/scripts/log_for_claude.py LOGFILE.bin
    python Tools/scripts/log_for_claude.py LOGFILE.bin --start 120 --end 180 --max-rows 5000
    python Tools/scripts/log_for_claude.py LOGFILE.bin --messages TILT,CTUN,RCOU

Then point Claude Code at the output directory: summary.md first for the
overview, then whichever <TYPE>.csv files turn out to be relevant - there's
no need to read every CSV into context, just the ones the discussion
actually calls for.

Requires pymavlink (either `pip install pymavlink`, or run from the
ardupilot repo root so modules/mavlink/pymavlink is importable).

For an interactive human-facing look at a signal (pan/zoom, FFT, time vs
frequency domain), see the companion script log_fft_view.py in this same
directory.
"""
import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                 "modules", "mavlink"))

from pymavlink import mavutil  # noqa: E402

SPECIAL_TYPES = ("MODE", "EV", "ERR", "MSG", "PARM")


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logfile", help="path to .bin dataflash log")
    p.add_argument("--out", default=None,
                    help="output directory (default: <logfile>_claude/)")
    p.add_argument("--messages", default=None,
                    help="comma-separated message types to write as CSV "
                         "(default: every type found in the log)")
    p.add_argument("--max-rows", type=int, default=3000,
                    help="max rows per CSV; longer series are uniformly "
                         "downsampled (default: 3000)")
    p.add_argument("--start", type=float, default=None,
                    help="only include data after this many seconds from log start")
    p.add_argument("--end", type=float, default=None,
                    help="only include data before this many seconds from log start")
    p.add_argument("--full-params", action="store_true",
                    help="also dump every parameter's final value, not just "
                         "ones that changed in-flight")
    return p.parse_args()


def in_window(t_rel, start, end):
    if t_rel is None:
        return True
    if start is not None and t_rel < start:
        return False
    if end is not None and t_rel > end:
        return False
    return True


def scan(mlog, start, end):
    """First pass: tally per-type counts/fieldnames and collect the small
    aggregates (mode changes, events, text messages, param dump/changes)
    without buffering every sample in memory."""
    t0 = None
    t_bounds = [None, None]
    counts = {}
    fieldnames = {}
    modes = []
    events = []
    text_msgs = []
    param_final = {}
    param_default = {}
    param_changes = []
    seen_param_once = set()

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype in ("BAD_DATA", "PARSE_ERROR"):
            continue

        ts = getattr(msg, "_timestamp", None)
        if t0 is None and ts:
            t0 = ts
        t_rel = (ts - t0) if (ts and t0) else None
        if not in_window(t_rel, start, end):
            continue
        if t_rel is not None:
            t_bounds[0] = t_rel if t_bounds[0] is None else min(t_bounds[0], t_rel)
            t_bounds[1] = t_rel if t_bounds[1] is None else max(t_bounds[1], t_rel)

        counts[mtype] = counts.get(mtype, 0) + 1
        if mtype not in fieldnames:
            d = msg.to_dict()
            d.pop("mavpackettype", None)
            fieldnames[mtype] = list(d.keys())

        if mtype == "MODE":
            modes.append((t_rel, msg.to_dict()))
        elif mtype in ("EV", "ERR"):
            events.append((t_rel, mtype, msg.to_dict()))
        elif mtype == "MSG":
            text_msgs.append((t_rel, getattr(msg, "Message", "")))
        elif mtype == "PARM":
            name = msg.Name
            value = msg.Value
            default = getattr(msg, "Default", None)
            if default is not None:
                param_default[name] = default
            if name in seen_param_once and param_final.get(name) != value:
                param_changes.append((t_rel, name, value))
            seen_param_once.add(name)
            param_final[name] = value

    return dict(
        t0=t0, t_start=t_bounds[0], t_end=t_bounds[1], counts=counts,
        fieldnames=fieldnames, modes=modes, events=events,
        text_msgs=text_msgs, param_final=param_final,
        param_default=param_default, param_changes=param_changes,
    )


def write_csvs(mlog, out_dir, t0, counts, fieldnames, write_types, max_rows, start, end):
    """Second pass (after mlog.rewind()): stream rows straight to disk,
    decimating on the fly with a running stride per type so we never hold
    more than one row per type in memory at once."""
    stride = {t: max(1.0, counts[t] / float(max_rows)) for t in write_types}
    next_keep = {t: 0.0 for t in write_types}
    seen_idx = {t: 0 for t in write_types}
    written = {t: 0 for t in write_types}

    files = {}
    writers = {}
    for t in write_types:
        path = os.path.join(out_dir, f"{t}.csv")
        fh = open(path, "w", newline="")
        w = csv.DictWriter(fh, fieldnames=["Ts"] + fieldnames[t])
        w.writeheader()
        files[t] = fh
        writers[t] = w

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype not in writers:
            continue

        ts = getattr(msg, "_timestamp", None)
        t_rel = (ts - t0) if (ts and t0) else None
        if not in_window(t_rel, start, end):
            continue

        if seen_idx[mtype] >= next_keep[mtype]:
            d = msg.to_dict()
            d.pop("mavpackettype", None)
            row = {"Ts": round(t_rel, 3) if t_rel is not None else ""}
            row.update(d)
            writers[mtype].writerow(row)
            next_keep[mtype] += stride[mtype]
            written[mtype] += 1
        seen_idx[mtype] += 1

    for fh in files.values():
        fh.close()

    return written


def main():
    args = parse_args()
    out_dir = args.out or (os.path.splitext(args.logfile)[0] + "_claude")
    os.makedirs(out_dir, exist_ok=True)

    mlog = mavutil.mavlink_connection(args.logfile, dialect="ardupilotmega")
    scanned = scan(mlog, args.start, args.end)

    if args.messages:
        requested = {m.strip().upper() for m in args.messages.split(",") if m.strip()}
        write_types = [t for t in scanned["counts"] if t in requested]
        missing = sorted(requested - set(scanned["counts"]))
    else:
        write_types = list(scanned["counts"].keys())
        missing = []

    mlog.rewind()
    written = write_csvs(mlog, out_dir, scanned["t0"], scanned["counts"],
                          scanned["fieldnames"], write_types, args.max_rows,
                          args.start, args.end)

    t_start, t_end = scanned["t_start"], scanned["t_end"]
    counts, fieldnames = scanned["counts"], scanned["fieldnames"]

    summary_path = os.path.join(out_dir, "summary.md")
    with open(summary_path, "w") as f:
        f.write(f"# Log summary: {os.path.basename(args.logfile)}\n\n")
        if t_start is not None:
            f.write(f"Time range covered: {t_start:.1f}s - {t_end:.1f}s "
                    f"(duration {t_end - t_start:.1f}s)\n\n")

        f.write("## Message type counts\n\n")
        f.write("| Type | Count | Fields |\n|---|---|---|\n")
        for mtype in sorted(counts):
            fields = ",".join(fieldnames.get(mtype, []))
            f.write(f"| {mtype} | {counts[mtype]} | {fields} |\n")
        f.write("\n")

        f.write("## Firmware / board (MSG text)\n\n")
        for t_rel, text in scanned["text_msgs"]:
            f.write(f"- t={t_rel:.1f}s: {text}\n")
        f.write("\n")

        f.write("## Flight mode timeline\n\n")
        for t_rel, d in scanned["modes"]:
            f.write(f"- t={t_rel:.1f}s: {d}\n")
        f.write("\n")

        f.write("## Events / errors (EV, ERR)\n\n")
        for t_rel, mtype, d in scanned["events"]:
            f.write(f"- t={t_rel:.1f}s [{mtype}]: {d}\n")
        f.write("\n")

        f.write("## Parameters changed in-flight\n\n")
        if scanned["param_changes"]:
            for t_rel, name, value in scanned["param_changes"]:
                f.write(f"- t={t_rel:.1f}s: {name} -> {value}\n")
        else:
            f.write("(none)\n")
        f.write("\n")

        if args.full_params:
            f.write("## Full parameter list (final value)\n\n")
            f.write("| Name | Value | Default |\n|---|---|---|\n")
            for name in sorted(scanned["param_final"]):
                f.write(f"| {name} | {scanned['param_final'][name]} | "
                        f"{scanned['param_default'].get(name, '')} |\n")
            f.write("\n")

        f.write("## CSV files written\n\n")
        for mtype in sorted(written):
            f.write(f"- {mtype}.csv: {written[mtype]} rows (from {counts[mtype]} "
                     f"raw samples)\n")
        if missing:
            f.write(f"\nRequested but not present in this log: {', '.join(missing)}\n")

    print(f"Wrote {summary_path}")
    print(f"Wrote {len(written)} CSV files to {out_dir}")


if __name__ == "__main__":
    main()
