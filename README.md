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

## Troubleshooting

- **`request_failed` / send "no ACK"**: The transceiver isn't on the bus. Most common causes: (1) MCP2515 `VCC` on 3V3 instead of **5V**; (2) CAN-H/CAN-L swapped or not connected to OBD pins 6/14; (3) no shared ground (OBD pin 5); (4) ignition not ON.
- **CAN init failed**: Verify SPI wiring and whether your module uses an 8 MHz or 16 MHz crystal (`CAN_CLOCK` in `config.h`).
- **PID timeouts**: Ignition must be on; some vehicles sleep the OBD port. Try engine running. Unsupported PIDs auto-disable after a few timeouts and show `--`.
- **Honda / extended IDs**: Set `OBD_USE_EXTENDED_ID` to `true` in `config.h`.

## Vehicle notes

- **VW Polo 2018 (MQB)**: Standard ISO 15765-4 CAN at 500 kbps, 11-bit IDs (request `0x7DF`, response `0x7E8`) — the default config works. Ignition must be fully ON (dash lit) or engine running; the gateway sleeps otherwise.
