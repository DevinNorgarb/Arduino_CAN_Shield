# Arduino CAN Shield / NodeMCU-32S OBD Reader

Read OBD-II data from a vehicle using an MCP2515-based OBD CAN adapter connected to a NodeMCU-32S (ESP32), with a built-in web dashboard over your phone's hotspot.

## In the vehicle: use your phone hotspot

**Recommended setup:** turn on your phone's Personal Hotspot / Mobile Hotspot. The NodeMCU joins it as a WiFi client. Your phone stays on cellular data, so maps, messages, and music keep working.

```
Phone (hotspot + cellular) ←→ NodeMCU-32S ←→ OBD port
         ↑
    browser opens http://obd.local
```

| Approach | Phone keeps internet? | Good for driving? |
|----------|----------------------|-------------------|
| **Phone hotspot** (recommended) | Yes | Yes |
| ESP32 creates its own WiFi | No | No — phone loses cellular |

### Quick start

1. On your phone: enable hotspot and note the network name + password.
2. Put those in `include/config.h` and upload firmware.
3. In the car: turn on hotspot, plug in the NodeMCU, wait ~10 seconds.
4. On your phone browser: open **http://obd.local** (or the IP from serial monitor).

The ESP32 retries the hotspot automatically if you turn it on after starting the car.

**Tip:** bookmark `http://obd.local` on your phone. Hotspot drains battery faster — keep the phone plugged in on long drives.

## Wiring

### NodeMCU-32S ↔ MCP2515 (SPI side)

| MCP2515 module | NodeMCU-32S | Notes |
|----------------|-------------|-------|
| VCC            | **5V / VIN** | Use 5V, **not 3V3** — the TJA1050/MCP2551 transceiver needs 5V to drive the bus. On 3V3 the SPI works but every send fails with "no ACK". |
| GND            | GND         | |
| SCK            | D18         | |
| MISO (SO)      | D19         | |
| MOSI (SI)      | D23         | |
| CS             | D5          | |
| INT            | D4          | |

### MCP2515 ↔ OBD-II port (CAN bus side)

| MCP2515 terminal | OBD-II pin | Signal |
|------------------|-----------|--------|
| CANH (H)         | **Pin 6** | CAN High |
| CANL (L)         | **Pin 14**| CAN Low |
| GND              | **Pin 5** | Signal ground (share ground — required) |
| (12V→5V buck) VIN| Pin 16    | +12V battery, only if powering from the car |

> **Front/back mirror gotcha (the #1 wiring mistake):** looking at the adapter
> plug from the *front* (metal pins) shows the pin numbering **mirrored** vs the
> *back* (solder side). It's easy to swap CAN-H (pin 6) and CAN-L (pin 14).
> Swapping H↔L is harmless — if unsure, just try it both ways. See the
> interactive OBD-II wiring canvas for all three views.

Plug the OBD-II end into the vehicle diagnostic port. Turn the ignition on (engine running is better for some PIDs).

Pins, WiFi credentials, and crystal frequency are configured in `include/config.h`.

Set your **phone hotspot** credentials before uploading:

```cpp
#define WIFI_SSID "Devin's iPhone"
#define WIFI_PASSWORD "your-hotspot-password"
```

## Build and upload

```bash
pio run -e nodemcu-32s -t upload
pio device monitor -e nodemcu-32s
```

If upload fails, hold the **BOOT** button on the NodeMCU-32S while connecting, or press BOOT when the terminal shows "Connecting...".

## Web dashboard

After the NodeMCU joins your hotspot, open:

```
http://obd.local
```

If that doesn't resolve on your phone, check the serial monitor for the IP (often `192.168.43.x` on Android or `172.20.10.x` on iPhone hotspot).

Live OBD data is pushed over WebSocket (`/ws`) whenever a new reading arrives.

## Serial output

The firmware also logs decoded values to serial:

```
rpm=850
speed_kmh=0
coolant_c=88
engine_load_pct=22.4
```

## Record & replay CAN data (bench debugging)

Capture raw CAN traffic in the car, then replay it at your desk so you can
develop the firmware and dashboard without a vehicle.

### 1. Record a drive

1. Open the dashboard and expand the **CAN recorder** section.
2. Press **Start recording**, drive / idle to capture the data you want, then
   **Stop recording**.
3. Press **Download .log** to save a `capture-<timestamp>.log` file.

Frames stream live over the WebSocket in SocketCAN
[candump](https://github.com/linux-can/can-utils) format, e.g.:

```
(0.123456) rx 7E8#03410C1AF0000000
(0.120000) tx 7DF#02010C0000000000
```

`tx` = a request the ESP32 sent, `rx` = a response from the car. This file
imports directly into **SavvyCAN** and replays with **canplayer**
(`canplayer vcan0=rx -I capture.log`).

### 2. Build a bench ECU simulator (second ESP32 + MCP2515)

Instead of faking data inside the firmware, a second ESP32 acts like the car's
ECU on a tiny bench bus. The main board runs the **real, unmodified firmware**.

Wire the two MCP2515 modules together — no car needed:

| Simulator MCP2515 | Main board MCP2515 |
|-------------------|--------------------|
| CANH (H)          | CANH (H)           |
| CANL (L)          | CANL (L)           |

Both modules' onboard 120 Ω resistors form a correctly terminated 2-node bus.
Power each board over USB. SPI wiring on the simulator board is identical to the
main board (same pins from `config.h`).

Turn your recording into the simulator's response table, then flash it:

```bash
python scripts/log_to_header.py capture.log   # -> src/ecu_sim/recorded_responses.h
pio run -e esp32-ecu-sim -t upload            # flash the SECOND ESP32
pio run -e nodemcu-32s -t upload              # flash the main board (unchanged)
```

The simulator listens for the main board's OBD requests and replays the real
recorded responses (including multi-frame ISO-TP DTC replies), stepping through
the captured drive one sample per request — so the dashboard gauges, charts, PID
scan, and DTC read all behave like the real car. A synthetic demo table ships by
default, so `esp32-ecu-sim` builds and runs even before you've recorded anything.

## Troubleshooting

- **`request_failed` / send "no ACK"**: The transceiver isn't on the bus. Most common causes: (1) MCP2515 `VCC` on 3V3 instead of **5V**; (2) CAN-H/CAN-L swapped or not connected to OBD pins 6/14; (3) no shared ground (OBD pin 5); (4) ignition not ON.
- **CAN init failed**: Verify SPI wiring and whether your module uses an 8 MHz or 16 MHz crystal (`CAN_CLOCK` in `config.h`).
- **PID timeouts**: Ignition must be on; some vehicles sleep the OBD port. Try engine running. Unsupported PIDs auto-disable after a few timeouts and show `--`.
- **Honda / extended IDs**: Set `OBD_USE_EXTENDED_ID` to `true` in `config.h`.

## Vehicle notes

- **VW Polo 2018 (MQB)**: Standard ISO 15765-4 CAN at 500 kbps, 11-bit IDs (request `0x7DF`, response `0x7E8`) — the default config works. Ignition must be fully ON (dash lit) or engine running; the gateway sleeps otherwise.
