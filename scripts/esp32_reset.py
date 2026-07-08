#!/usr/bin/env python3
"""Put ESP32 into serial bootloader via CP210x DTR/RTS."""

import sys
import time

import serial


def enter_bootloader(port: str) -> None:
    ser = serial.Serial(port, 115200, timeout=0.1)
    try:
        # RTS -> EN, DTR -> GPIO0 on typical NodeMCU-32S auto-reset circuit.
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.05)
        ser.dtr = True
        time.sleep(0.25)
        ser.dtr = False
    finally:
        ser.close()
    time.sleep(0.05)


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <serial-port>", file=sys.stderr)
        return 1

    port = sys.argv[1]
    print(f"Resetting ESP32 into bootloader on {port}...")
    enter_bootloader(port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
