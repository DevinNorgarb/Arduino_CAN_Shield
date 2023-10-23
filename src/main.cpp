#include <CAN.h>

void decodeOBDData();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver");

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      if (CAN.packetId() == 0x7E8) { // OBD-II response ID
        decodeOBDData();
      } else {
        while (CAN.available()) {
          Serial.print((char)CAN.read());
        }
        Serial.println();
      }
    }

    Serial.println();
  }
}

void decodeOBDData() {
  byte pid = CAN.read();
  Serial.print("PID: ");
  Serial.println(pid, HEX);

  switch (pid) {
    case 0x05:
      int temp = CAN.read() - 40;
      Serial.print("Engine Coolant Temperature: ");
      Serial.println(temp);
      break;

    case 0x0C:
      int rpm = ((CAN.read() * 256) + CAN.read()) / 4;
      Serial.print("Engine RPM: ");
      Serial.println(rpm);
      break;

    case 0x0D:
      Serial.print("Vehicle Speed: ");
      Serial.println(CAN.read());
      break;

    case 0x03:
      byte fuelSystemStatus = CAN.read();
      Serial.print("Fuel System Status: ");
      Serial.println(fuelSystemStatus);
      break;

    case 0x04:
      byte engineLoad = CAN.read();
      float calculatedEngineLoad = (engineLoad * 100.0) / 255.0; // Convert to percentage
      Serial.print("Calculated Engine Load: ");
      Serial.println(calculatedEngineLoad, 2); // Print with 2 decimal places
      break;


    // ... Add other PID cases as needed

    default:
      while (CAN.available()) {
        CAN.read(); // Consume any remaining bytes
      }
      break;
  }
}

// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.


// This examples queries the ECU for the Mode 01 OBD-II PID's it supports and prints the supported
// OBD-II PID's to the serial monitor

// A full list of PID's and their meaning can be found here:
//   https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_01




// Show PIDS
// #include <CAN.h>

// // Most cars support 11-bit adddress, others (like Honda),
// // require 29-bit (extended) addressing, set the next line
// // to true to use extended addressing
// const bool useStandardAddressing = true;

// void setup() {
//   Serial.begin(9600);

//   Serial.println("CAN OBD-II supported pids");

//   // start the CAN bus at 500 kbps
//   if (!CAN.begin(500E3)) {
//     Serial.println("Starting CAN failed!");
//     while (1);
//   }

//   // add filter to only receive the CAN bus ID's we care about
//   if (useStandardAddressing) {
//     CAN.filter(0x7e8);
//   } else {
//     CAN.filterExtended(0x18daf110);
//   }
// }

// void loop() {
//   for (int pid = 0x00; pid < 0xe0; pid += 0x20) {
//     if (useStandardAddressing) {
//       CAN.beginPacket(0x7df, 8);
//     } else {
//       CAN.beginExtendedPacket(0x18db33f1, 8);
//     }
//     CAN.write(0x02); // number of additional bytes
//     CAN.write(0x01); // show current data
//     CAN.write(pid);  // PID
//     CAN.endPacket();

//     // wait for response
//     while (CAN.parsePacket() == 0 ||
//            CAN.read() < 6 ||         // correct length
//            CAN.read() != 0x41 ||     // correct mode
//            CAN.read() != pid);       // correct PID

//     unsigned long pidsSupported = 0;

//     for (int i = 0; i < 4; i++) {
//       pidsSupported <<= 8;
//       pidsSupported |= CAN.read();
//     }

//     for (unsigned int i = 31; i > 0; i--) {
//       if (pidsSupported & (1UL << i)) {
//         int pidSupported = pid + (32 - i);

//         Serial.print("0x");
//         if (pidSupported < 16) {
//           Serial.print("0");
//         }
//         Serial.println(pidSupported, HEX);
//       }
//     }

//     if ((pidsSupported & 0x00000001) == 0x00000000) {
//       // next round not supported, all done
//       break;
//     }
//   }

//   Serial.println("That's all folks!");

//   while (1); // all done
// }

// CAN OBD-II supported pids
// 0x01
// 0x03
// 0x04
// 0x05
// 0x06
// 0x07
// 0x0B
// 0x0C
// 0x0D
// 0x0E
// 0x0F
// 0x11
// 0x13
// 0x14
// 0x15
// 0x1C
// 0x1F
// 0x21
// 0x23
// 0x2E
// 0x2F
// 0x30
// 0x31
// 0x33
// 0x3C
// 0x41
// 0x42
// 0x43
// 0x44
// 0x45
// 0x46
// 0x47
// 0x48
// 0x49
// 0x4A
// 0x4C
// 0x4F
// 0x50
// 0x51
// 0x56
// 0x70
// That's all folks!