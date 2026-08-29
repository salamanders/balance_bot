#!/usr/bin/env python3
"""
Blackbox Forensics Tool for Balance Bot Telemetry.

Analyzes flight_data.csv to classify kick-up attempts, measure pulse widths,
assess energy delivery, and categorize failure modes.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import statistics
import sys
from typing import TextIO


@dataclass
class TelemetryRow:
    timestamp: float
    state: str
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    left_pwm: float
    right_pwm: float
    pid_target: float


@dataclass
class PulseInfo:
    start_time: float
    duration_ms: float
    peak_left_pwm: float
    peak_right_pwm: float
    start_pitch: float
    min_dist_to_vertical: float
    max_pitch_reached: float
    reached_crossover_zone: bool  # |pitch| < 15 deg


@dataclass
class SessionSummary:
    session_id: int
    start_time: float
    end_time: float
    duration_s: float
    start_state: str
    final_state: str
    pulse_count: int
    pulses: list[PulseInfo]
    outcome: str  # BALANCED, CRASHED, TIMEOUT, ABORTED
    failure_class: str  # NONE, NO_MOVEMENT, INSUFFICIENT_ENERGY, WRONG_TIMING, CAUGHT_THEN_LOST


def parse_telemetry_file(filepath: Path) -> list[TelemetryRow]:
    rows: list[TelemetryRow] = []
    with filepath.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append(
                    TelemetryRow(
                        timestamp=float(r["timestamp"]),
                        state=r["state"].strip(),
                        pitch_angle=float(r["pitch_angle"]),
                        pitch_rate=float(r["pitch_rate"]),
                        yaw_rate=float(r["yaw_rate"]),
                        left_pwm=float(r["left_pwm"]),
                        right_pwm=float(r["right_pwm"]),
                        pid_target=float(r["pid_target"]),
                    )
                )
            except (ValueError, KeyError):
                continue
    return rows


def analyze_kickup_sessions(rows: list[TelemetryRow]) -> list[SessionSummary]:
    if not rows:
        return []

    # Segment rows by KickupState sessions
    sessions: list[SessionSummary] = []
    in_kickup = False
    current_session_rows: list[TelemetryRow] = []
    session_id = 0

    for i, r in enumerate(rows):
        if r.state == "KickupState":
            if not in_kickup:
                in_kickup = True
                current_session_rows = [r]
                session_id += 1
            else:
                current_session_rows.append(r)
        else:
            if in_kickup:
                in_kickup = False
                # Collect trailing context (up to 3 seconds after KickupState)
                trailing_rows: list[TelemetryRow] = []
                for j in range(i, min(len(rows), i + 300)):
                    trailing_rows.append(rows[j])
                    if rows[j].state == "KickupState":
                        break
                session = _evaluate_session(session_id, current_session_rows, trailing_rows)
                sessions.append(session)
                current_session_rows = []

    if in_kickup and current_session_rows:
        session = _evaluate_session(session_id, current_session_rows, [])
        sessions.append(session)

    return sessions


def _evaluate_session(
    session_id: int,
    kickup_rows: list[TelemetryRow],
    trailing_rows: list[TelemetryRow],
) -> SessionSummary:
    start_time = kickup_rows[0].timestamp
    end_time = kickup_rows[-1].timestamp
    duration_s = max(0.0, end_time - start_time)

    # Detect pulses within kickup_rows
    pulses: list[PulseInfo] = []
    in_pulse = False
    pulse_rows: list[TelemetryRow] = []

    for r in kickup_rows:
        is_active = abs(r.left_pwm) > 1.0 or abs(r.right_pwm) > 1.0
        if is_active:
            if not in_pulse:
                in_pulse = True
                pulse_rows = [r]
            else:
                pulse_rows.append(r)
        else:
            if in_pulse:
                in_pulse = False
                pulse_info = _evaluate_pulse(pulse_rows, kickup_rows)
                pulses.append(pulse_info)
                pulse_rows = []

    if in_pulse and pulse_rows:
        pulse_info = _evaluate_pulse(pulse_rows, kickup_rows)
        pulses.append(pulse_info)

    # Determine outcome & failure classification
    final_state = trailing_rows[-1].state if trailing_rows else kickup_rows[-1].state
    states_seen = {r.state for r in trailing_rows}

    if "BalancingState" in states_seen:
        if "CrashedState" in states_seen or "FatalErrorState" in states_seen:
            outcome = "CRASHED"
            failure_class = "CAUGHT_THEN_LOST"
        else:
            outcome = "BALANCED"
            failure_class = "NONE"
    elif "CrashedState" in states_seen or "FatalErrorState" in states_seen:
        outcome = "CRASHED"
        failure_class = _classify_failure_without_balance(pulses, kickup_rows)
    else:
        outcome = "TIMEOUT"
        failure_class = _classify_failure_without_balance(pulses, kickup_rows)

    return SessionSummary(
        session_id=session_id,
        start_time=start_time,
        end_time=end_time,
        duration_s=duration_s,
        start_state=kickup_rows[0].state,
        final_state=final_state,
        pulse_count=len(pulses),
        pulses=pulses,
        outcome=outcome,
        failure_class=failure_class,
    )


def _evaluate_pulse(
    pulse_rows: list[TelemetryRow],
    session_rows: list[TelemetryRow],
) -> PulseInfo:
    start_t = pulse_rows[0].timestamp
    end_t = pulse_rows[-1].timestamp
    # If single row, estimate 10ms
    duration_ms = max(10.0, (end_t - start_t) * 1000.0)
    peak_l = max((abs(r.left_pwm) for r in pulse_rows), default=0.0)
    peak_r = max((abs(r.right_pwm) for r in pulse_rows), default=0.0)
    start_pitch = pulse_rows[0].pitch_angle

    # Look up to 1.5s after pulse onset in session
    onset_t = start_t
    window_rows = [r for r in session_rows if onset_t <= r.timestamp <= onset_t + 1.5]
    if not window_rows:
        window_rows = pulse_rows

    min_dist = min((abs(r.pitch_angle) for r in window_rows), default=abs(start_pitch))
    max_pitch = max((abs(r.pitch_angle) for r in window_rows), default=abs(start_pitch))
    reached_crossover = min_dist < 15.0

    return PulseInfo(
        start_time=start_t,
        duration_ms=duration_ms,
        peak_left_pwm=peak_l,
        peak_right_pwm=peak_r,
        start_pitch=start_pitch,
        min_dist_to_vertical=min_dist,
        max_pitch_reached=max_pitch,
        reached_crossover_zone=reached_crossover,
    )


def _classify_failure_without_balance(
    pulses: list[PulseInfo],
    rows: list[TelemetryRow],
) -> str:
    if not pulses:
        return "NO_MOVEMENT"

    # Check if peak pitch moved significantly
    start_pitch = rows[0].pitch_angle
    max_delta = max((abs(r.pitch_angle - start_pitch) for r in rows), default=0.0)
    min_dist_to_vert = min((abs(r.pitch_angle) for r in rows), default=90.0)

    if max_delta < 3.0:
        return "NO_MOVEMENT"
    if min_dist_to_vert > 30.0:
        return "INSUFFICIENT_ENERGY"
    if any(p.reached_crossover_zone for p in pulses) or min_dist_to_vert <= 15.0:
        return "WRONG_TIMING"

    return "INSUFFICIENT_ENERGY"


def print_markdown_report(sessions: list[SessionSummary], out: TextIO = sys.stdout) -> None:
    out.write("# Balance Bot - Blackbox Forensics Report\n\n")
    if not sessions:
        out.write("No KickupState sessions found in telemetry log.\n")
        return

    all_pulses = [p for s in sessions for p in s.pulses]
    durations = [p.duration_ms for p in all_pulses] if all_pulses else []
    avg_pulse_ms = statistics.mean(durations) if durations else 0.0
    max_pulse_ms = max(durations) if durations else 0.0

    out.write("## Executive Summary\n\n")
    out.write(f"- **Total Kickup Sessions:** {len(sessions)}\n")
    out.write(f"- **Total Pulses Detected:** {len(all_pulses)}\n")
    out.write(f"- **Average Pulse Width:** {avg_pulse_ms:.1f} ms\n")
    out.write(f"- **Max Pulse Width:** {max_pulse_ms:.1f} ms\n")

    # F0 Verification Check
    out.write("\n### F0 Pulse Collapse Diagnostic\n")
    if durations and all(d <= 25.0 for d in durations):
        out.write("> **ALERT:** All pulses <= 25ms. **F0 (Pulse Collapse Defect) is CONFIRMED.**\n\n")
    elif durations:
        out.write(f"> **INFO:** Observed pulse range: {min(durations):.1f}ms - {max(durations):.1f}ms.\n\n")

    # Failure Classification Breakdown
    failure_counts: dict[str, int] = {}
    for s in sessions:
        failure_counts[s.failure_class] = failure_counts.get(s.failure_class, 0) + 1

    out.write("## Failure Class Breakdown\n\n")
    out.write("| Failure Class | Count | Description |\n")
    out.write("| :--- | :--- | :--- |\n")
    out.write(f"| `NO_MOVEMENT` | {failure_counts.get('NO_MOVEMENT', 0)} | Robot pitch changed < 3° during pulse |\n")
    out.write(f"| `INSUFFICIENT_ENERGY` | {failure_counts.get('INSUFFICIENT_ENERGY', 0)} | Moved but never reached within 30° of vertical |\n")
    out.write(f"| `WRONG_TIMING` | {failure_counts.get('WRONG_TIMING', 0)} | Reached crossover zone (<15°) but failed to catch |\n")
    out.write(f"| `CAUGHT_THEN_LOST` | {failure_counts.get('CAUGHT_THEN_LOST', 0)} | Transitioned to BalancingState then crashed |\n")
    out.write(f"| `NONE (Success)` | {failure_counts.get('NONE', 0)} | Balanced successfully |\n\n")

    out.write("## Session Details\n\n")
    out.write("| Session # | Duration (s) | Pulses | Peak PWM | Min Dist to Vert (°) | Outcome | Class |\n")
    out.write("| :---: | :---: | :---: | :---: | :---: | :---: | :--- |\n")
    for s in sessions:
        peak_pwm = max((max(p.peak_left_pwm, p.peak_right_pwm) for p in s.pulses), default=0.0)
        min_dist = min((p.min_dist_to_vertical for p in s.pulses), default=90.0)
        out.write(
            f"| {s.session_id} | {s.duration_s:.2f} | {s.pulse_count} | {peak_pwm:.1f} | "
            f"{min_dist:.1f}° | {s.outcome} | `{s.failure_class}` |\n"
        )


def export_csv_summary(sessions: list[SessionSummary], csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "session_id",
                "duration_s",
                "pulse_count",
                "peak_pwm",
                "min_dist_to_vertical",
                "outcome",
                "failure_class",
            ]
        )
        for s in sessions:
            peak_pwm = max((max(p.peak_left_pwm, p.peak_right_pwm) for p in s.pulses), default=0.0)
            min_dist = min((p.min_dist_to_vertical for p in s.pulses), default=90.0)
            writer.writerow(
                [
                    s.session_id,
                    round(s.duration_s, 2),
                    s.pulse_count,
                    round(peak_pwm, 1),
                    round(min_dist, 1),
                    s.outcome,
                    s.failure_class,
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze balance bot blackbox telemetry.")
    parser.add_argument(
        "file",
        nargs="?",
        default="flight_data.csv",
        help="Path to telemetry flight_data.csv (default: flight_data.csv)",
    )
    parser.add_argument(
        "--csv",
        dest="csv_out",
        type=str,
        default=None,
        help="Optional path to output summary CSV",
    )
    args = parser.parse_args()

    filepath = Path(args.file)
    if not filepath.exists():
        print(f"File not found: {filepath}", file=sys.stderr)
        sys.exit(1)

    rows = parse_telemetry_file(filepath)
    sessions = analyze_kickup_sessions(rows)
    print_markdown_report(sessions)

    if args.csv_out:
        csv_out_path = Path(args.csv_out)
        export_csv_summary(sessions, csv_out_path)
        print(f"\nSummary exported to {csv_out_path}")


if __name__ == "__main__":
    main()
