"""Discover CP210x port and enter bootloader immediately before esptool upload."""

import glob
import os
import time

Import("env")

PORT_PATTERNS = (
    "/dev/cu.SLAB_USBtoUART",
    "/dev/cu.usbserial-*",
    "/dev/cu.wchusbserial*",
)


def _find_port():
    for pattern in PORT_PATTERNS:
        for path in sorted(glob.glob(pattern)):
            if os.path.exists(path):
                return path
    return None


def _wait_for_port(timeout_s=45):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        port = _find_port()
        if port:
            return port
        time.sleep(0.5)
    return None


def _reset_script_path():
    return os.path.join(env["PROJECT_DIR"], "scripts", "esp32_reset.py")


def before_upload(source, target, env):
    port = env.subst("$UPLOAD_PORT")
    if not port or not os.path.exists(port):
        print("ESP32 USB port not set or missing — scanning for CP210x...")
        port = _wait_for_port()
        if not port:
            raise RuntimeError(
                "No ESP32 serial port found. Plug in the board via USB, then retry.\n"
                "Expected: /dev/cu.SLAB_USBtoUART or /dev/cu.usbserial-*"
            )
        env.Replace(UPLOAD_PORT=port)
        print(f"Using upload port: {port}")

    pythonexe = env.subst("$PYTHONEXE")
    reset_script = _reset_script_path()
    upload_cmd = env.subst("$UPLOADCMD")

    env.Replace(
        UPLOADCMD=f'"{pythonexe}" "{reset_script}" "{port}" && {upload_cmd}'
    )


env.AddPreAction("upload", before_upload)
