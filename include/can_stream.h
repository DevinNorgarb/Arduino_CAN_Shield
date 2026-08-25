#pragma once

#include <Arduino.h>

// Outbound WebSocket client that publishes live CAN frames to CAN_STREAM_HOST.
// Independent of the dashboard recorder: when the host is set and WiFi is up,
// every frame at the CAN I/O choke point is batched and sent as JSON.

void canStreamLoop();
void canStreamStop();

// Copy a frame into the outbound batch. Drops it if the uplink is paused,
// disconnected, or the batch cannot be flushed.
void canStreamPublish(bool tx, unsigned long id, bool extended, uint8_t len,
                      const uint8_t *data);

// Send a pre-built JSON payload (OBD dashboard state) on the same uplink.
void canStreamSendText(const String &payload);

bool canStreamConfigured();
bool canStreamConnected();
bool canStreamWantFrames();
bool canStreamPaused();
void canStreamSetPaused(bool paused);

uint32_t canStreamSent();
uint32_t canStreamDropped();
const char *canStreamHost();
