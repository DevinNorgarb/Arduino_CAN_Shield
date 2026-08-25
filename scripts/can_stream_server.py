#!/usr/bin/env python3
"""CAN ingest + the same OBD dashboard the ESP32 serves on the hotspot.

ESP32 connects as a WebSocket client to ws://host:port/ and pushes frames / OBD.
Browsers open http://host:port/ and subscribe at /ws.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import sys
from datetime import datetime, timezone
from http import HTTPStatus
from pathlib import Path

try:
    import websockets
    from websockets.datastructures import Headers
    from websockets.http11 import Response
except ImportError:
    sys.exit("Missing dependency. Install with: pip install websockets")


def env(name: str, default: str) -> str:
    value = os.environ.get(name)
    return default if value is None or value == "" else value


DASHBOARD_PATH = Path(__file__).with_name("dashboard.html")
DASHBOARD = DASHBOARD_PATH.read_bytes() if DASHBOARD_PATH.exists() else b""

browsers: set = set()
devices: set = set()
last_obd: str | None = None
out_file = None


def candump_line(frame: dict) -> str:
    t = int(frame.get("t") or 0)
    secs, ms = divmod(t, 1000)
    direction = "tx" if frame.get("tx") else "rx"
    cid = int(frame.get("id") or 0)
    ext = bool(frame.get("ext"))
    id_s = f"{cid:08X}" if ext or cid > 0x7FF else f"{cid:03X}"
    data = frame.get("data") or ""
    return f"({secs}.{ms:03d}000) {direction} {id_s}#{data}"


async def broadcast_browsers(text: str) -> None:
    dead = []
    for ws in list(browsers):
        try:
            await ws.send(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        browsers.discard(ws)


async def forward_to_devices(text: str) -> None:
    dead = []
    for ws in list(devices):
        try:
            await ws.send(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        devices.discard(ws)


def log_msg(msg: dict) -> None:
    if out_file is None:
        return
    stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    out_file.write(json.dumps({"recv_at": stamp, "msg": msg}) + "\n")
    out_file.flush()
    os.fsync(out_file.fileno())


async def handle_device(ws) -> None:
    peer = ws.remote_address
    devices.add(ws)
    print(f"device connected {peer}", flush=True)
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                print(f"malformed device message: {raw!r}", flush=True)
                continue

            log_msg(msg)

            if msg.get("type") == "hello":
                print(f"hello ip={msg.get('ip')}", flush=True)
                continue

            frames = msg.get("frames")
            if frames:
                for f in frames:
                    await broadcast_browsers(json.dumps({"can": candump_line(f)}))
                continue

            if "rpm" in msg:
                global last_obd
                last_obd = raw if isinstance(raw, str) else json.dumps(msg)
                await broadcast_browsers(last_obd)
    finally:
        devices.discard(ws)
        print(f"device disconnected {peer}", flush=True)


async def handle_browser(ws) -> None:
    peer = ws.remote_address
    browsers.add(ws)
    print(f"browser connected {peer} ({len(browsers)} dashboards)", flush=True)
    if last_obd:
        try:
            await ws.send(last_obd)
        except Exception:
            pass
    try:
        async for raw in ws:
            cmd = raw if isinstance(raw, str) else raw.decode("utf-8", "replace")
            print(f"command from browser: {cmd}", flush=True)
            await forward_to_devices(cmd)
    finally:
        browsers.discard(ws)
        print(f"browser disconnected {peer}", flush=True)


async def handler(ws) -> None:
    path = getattr(getattr(ws, "request", None), "path", None) or "/"
    if path.startswith("/ws"):
        await handle_browser(ws)
    else:
        await handle_device(ws)


def process_request(connection, request):
    headers = request.headers
    upgrade = ""
    for key, value in headers.raw_items() if hasattr(headers, "raw_items") else []:
        if key.lower() == "upgrade":
            upgrade = value.lower()
            break
    if not upgrade:
        upgrade = (headers.get("Upgrade") or headers.get("upgrade") or "").lower()
    if upgrade == "websocket":
        return None
    if request.path in ("/", "/index.html"):
        if not DASHBOARD:
            return connection.respond(HTTPStatus.INTERNAL_SERVER_ERROR, "dashboard.html missing\n")
        return Response(
            HTTPStatus.OK.value,
            HTTPStatus.OK.phrase,
            Headers(
                [
                    ("Content-Type", "text/html; charset=utf-8"),
                    ("Content-Length", str(len(DASHBOARD))),
                    ("Connection", "close"),
                ]
            ),
            DASHBOARD,
        )
    return connection.respond(HTTPStatus.NOT_FOUND, "not found\n")


async def serve(host: str, port: int) -> None:
    loop = asyncio.get_running_loop()
    stopped = asyncio.Event()

    def request_stop():
        stopped.set()

    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, request_stop)

    async with websockets.serve(
        handler,
        host,
        port,
        process_request=process_request,
        origins=None,
        ping_interval=20,
        ping_timeout=20,
    ):
        print(f"dashboard http://{host}:{port}/  device ws://{host}:{port}/", flush=True)
        await stopped.wait()
        print("shutting down", flush=True)


def main() -> None:
    global out_file
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=env("CAN_STREAM_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(env("CAN_STREAM_PORT", "8765")))
    parser.add_argument("--out", default=env("CAN_STREAM_OUT", "") or None)
    args = parser.parse_args()

    if not DASHBOARD:
        print("warning: dashboard.html not found next to this script", flush=True)

    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        out_file = path.open("a", encoding="utf-8", buffering=1)

    try:
        asyncio.run(serve(args.host, args.port))
    except KeyboardInterrupt:
        pass
    finally:
        if out_file is not None:
            out_file.close()


if __name__ == "__main__":
    main()
