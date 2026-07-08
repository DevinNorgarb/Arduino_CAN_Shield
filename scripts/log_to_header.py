#!/usr/bin/env python3
"""Convert a recorded CAN capture (candump .log) into an ECU-simulator table.

The main firmware's dashboard records raw frames in SocketCAN "candump" format:

    (0.123456) rx 7E8#03410C1AF0000000
    (0.120000) tx 7DF#02010C0000000000

This script pairs each OBD request (tx) with the response frames that followed
it (rx), grouped by (mode, pid). The result is written as a C header consumed by
the ecu-sim firmware, which replays the real recorded responses on demand -
stepping through the drive one sample per request so the dashboard animates.

Usage:
    python scripts/log_to_header.py capture.log
    python scripts/log_to_header.py capture.log -o src/ecu_sim/recorded_responses.h

Direction is taken from the interface field (tx/rx). If a capture uses generic
interface names (e.g. can0), direction is inferred from the CAN ID instead.
"""

import argparse
import re
import sys
from pathlib import Path

# Request IDs: OBD functional broadcast + physical ECU addresses (7E0-7E7).
REQUEST_IDS = {0x7DF, 0x18DB33F1}
PHYSICAL_REQUEST_RANGE = range(0x7E0, 0x7E8)

# OBD response IDs (7E8-7EF, plus the 29-bit response). Other bus broadcasts
# captured in the raw log (e.g. VW network-management frames) are ignored so
# they don't get glued onto the ECU responses in the simulator table.
RESPONSE_RANGE = range(0x7E8, 0x7F0)
EXTENDED_RESPONSE_ID = 0x18DAF110

# mcp_can flags extended frames by OR-ing this into the ID; older captures may
# still contain it, so mask it off everywhere.
EXT_FLAG = 0x80000000


def is_response_id(can_id):
    return can_id in RESPONSE_RANGE or can_id == EXTENDED_RESPONSE_ID

# Cap responses stored per (mode, pid) so a long drive doesn't blow up flash.
MAX_SEQUENCES_PER_KEY = 1500

LINE_RE = re.compile(
    r"^\(([\d.]+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$"
)


class Frame:
    __slots__ = ("tx", "can_id", "data")

    def __init__(self, tx, can_id, data):
        self.tx = tx
        self.can_id = can_id
        self.data = data


def is_request_id(can_id):
    return can_id in REQUEST_IDS or can_id in PHYSICAL_REQUEST_RANGE


def parse_log(path):
    frames = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = LINE_RE.match(line)
        if not m:
            print(f"warning: skipping unparseable line {lineno}: {line}", file=sys.stderr)
            continue
        _, iface, id_hex, data_hex = m.groups()
        can_id = int(id_hex, 16) & ~EXT_FLAG
        data = bytes.fromhex(data_hex) if data_hex else b""

        iface_l = iface.lower()
        if iface_l in ("tx", "rx"):
            tx = iface_l == "tx"
        else:
            tx = is_request_id(can_id)
        frames.append(Frame(tx, can_id, data))
    return frames


def group_responses(frames):
    """Return {(mode, pid, has_pid): [sequence, ...]}.

    A sequence is a list of (can_id, data) response frames for one request.
    Flow-control frames (PCI 0x30) are treated as delimiters, not new requests,
    so multi-frame ISO-TP replies stay grouped with their request.
    """
    responses = {}
    current_key = None
    current_seq = []

    def flush():
        nonlocal current_seq
        if current_key is not None and current_seq:
            responses.setdefault(current_key, []).append(current_seq)
        current_seq = []

    for f in frames:
        if not f.data:
            continue
        pci = f.data[0] & 0xF0

        if f.tx:
            if pci == 0x30:  # flow control - part of an in-progress exchange
                continue
            flush()
            num_bytes = f.data[0] & 0x0F
            mode = f.data[1] if len(f.data) > 1 else 0
            has_pid = num_bytes >= 2 and len(f.data) > 2
            pid = f.data[2] if has_pid else 0
            current_key = (mode, pid, has_pid)
        else:
            # Only real OBD responses belong to the request; skip unrelated
            # bus broadcasts captured in the raw log.
            if current_key is not None and is_response_id(f.can_id):
                current_seq.append((f.can_id, f.data))

    flush()
    return responses


def decimate(sequences, limit):
    if len(sequences) <= limit:
        return sequences
    step = len(sequences) / limit
    return [sequences[int(i * step)] for i in range(limit)]


def c_frame(can_id, data):
    padded = list(data[:8]) + [0] * (8 - len(data))
    bytes_str = ", ".join(f"0x{b:02X}" for b in padded)
    return f"  {{0x{can_id:X}, {len(data)}, {{{bytes_str}}}}}"


def render_header(responses):
    frames_out = []
    resp_out = []
    keys_out = []

    for key in sorted(responses.keys()):
        mode, pid, has_pid = key
        sequences = decimate(responses[key], MAX_SEQUENCES_PER_KEY)
        resp_start = len(resp_out)
        for seq in sequences:
            frame_start = len(frames_out)
            for can_id, data in seq:
                frames_out.append(c_frame(can_id, data))
            resp_out.append(f"  {{{frame_start}, {len(seq)}}}")
        keys_out.append(
            f"  {{0x{mode:02X}, 0x{pid:02X}, {1 if has_pid else 0}, "
            f"{resp_start}, {len(sequences)}}}"
        )

    lines = [
        "#pragma once",
        "",
        "// Generated by scripts/log_to_header.py - do not edit by hand.",
        "// ECU-simulator response table built from a recorded CAN capture.",
        "",
        "#include <stdint.h>",
        "",
        "struct SimFrame {",
        "  uint32_t id;",
        "  uint8_t len;",
        "  uint8_t data[8];",
        "};",
        "",
        "// One response = a contiguous run of SimFrames (multiple = ISO-TP multi-frame).",
        "struct SimResponse {",
        "  uint16_t frameStart;",
        "  uint8_t frameCount;",
        "};",
        "",
        "// One request key = a contiguous run of SimResponses (the recorded timeline).",
        "struct SimKey {",
        "  uint8_t mode;",
        "  uint8_t pid;",
        "  uint8_t hasPid;",
        "  uint16_t respStart;",
        "  uint16_t respCount;",
        "};",
        "",
        f"const SimFrame kSimFrames[] = {{",
        ",\n".join(frames_out) if frames_out else "  {0, 0, {0}}",
        "};",
        "",
        f"const SimResponse kSimResponses[] = {{",
        ",\n".join(resp_out) if resp_out else "  {0, 0}",
        "};",
        "",
        f"const SimKey kSimKeys[] = {{",
        ",\n".join(keys_out) if keys_out else "  {0, 0, 0, 0, 0}",
        "};",
        "",
        f"const uint16_t kSimKeyCount = {len(keys_out)};",
        "",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", type=Path, help="recorded candump .log file")
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("src/ecu_sim/recorded_responses.h"),
                        help="output header path (default: %(default)s)")
    args = parser.parse_args()

    if not args.logfile.exists():
        parser.error(f"log file not found: {args.logfile}")

    frames = parse_log(args.logfile)
    responses = group_responses(frames)
    if not responses:
        parser.error("no request/response pairs found - is this a valid capture?")

    header = render_header(responses)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header)

    total_seqs = sum(len(v) for v in responses.values())
    print(f"Parsed {len(frames)} frames -> {len(responses)} PID keys, "
          f"{total_seqs} recorded responses")
    print(f"Wrote {args.output}")
    for (mode, pid, has_pid) in sorted(responses.keys()):
        label = f"mode 0x{mode:02X}" + (f" pid 0x{pid:02X}" if has_pid else "")
        print(f"  {label}: {len(responses[(mode, pid, has_pid)])} samples")


if __name__ == "__main__":
    main()
