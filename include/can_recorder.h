#pragma once

#include <Arduino.h>

// Raw CAN capture. When active, every frame that passes through the CAN I/O
// wrappers is formatted as a SocketCAN "candump" log line and streamed to the
// web dashboard, where it can be downloaded as a .log file. The format is
// directly importable into SavvyCAN and replayable with canplayer.
//
//   (0.123456) rx 7E8#03410C1AF0000000
//   (0.120000) tx 7DF#02010C0000000000
//
// The relative timestamp is seconds.microseconds from the start of recording;
// the interface field encodes direction (tx = our request, rx = ECU response).

void canRecordStart();
void canRecordStop();
bool canRecordActive();
uint32_t canRecordCount();

void canRecordFrame(bool tx, unsigned long id, uint8_t len, const uint8_t *data);
