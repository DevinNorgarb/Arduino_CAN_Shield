#!/usr/bin/env python3
"""Pull the dashboard HTML out of src/web_dashboard.cpp (the PROGMEM raw string)."""

from __future__ import annotations

import sys
from pathlib import Path

START = 'R"rawliteral('
END = ')rawliteral"'


def main() -> None:
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    text = src.read_text(encoding="utf-8")
    begin = text.find(START)
    end = text.find(END, begin + len(START))
    if begin < 0 or end < 0:
        sys.exit(f"dashboard raw string not found in {src}")
    html = text[begin + len(START) : end].lstrip("\n")
    dst.write_text(html, encoding="utf-8")
    print(f"wrote {dst} ({len(html)} bytes)", flush=True)


if __name__ == "__main__":
    main()
