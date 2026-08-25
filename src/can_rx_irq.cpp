#include "can_rx_irq.h"

#include "config.h"

namespace {

volatile bool gRxPending = false;
bool gIrqAttached = false;

void IRAM_ATTR onCanRxIrq() {
  gRxPending = true;
}

}  // namespace

void canRxIrqInitPin() {
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
}

void canRxIrqEnable() {
  if (digitalRead(CAN_INT_PIN) == LOW) {
    gRxPending = true;
  }

  if (!gIrqAttached) {
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), onCanRxIrq, FALLING);
    gIrqAttached = true;
  }
}

void canRxIrqDisable() {
  if (gIrqAttached) {
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN));
    gIrqAttached = false;
  }
  gRxPending = false;
}

bool canRxIrqPending() {
  if (gRxPending) {
    return true;
  }
  // INT stays low while unread frames remain; catch cases the edge ISR missed.
  return gIrqAttached && digitalRead(CAN_INT_PIN) == LOW;
}

void canRxIrqAckDrain() {
  gRxPending = (gIrqAttached && digitalRead(CAN_INT_PIN) == LOW);
}
