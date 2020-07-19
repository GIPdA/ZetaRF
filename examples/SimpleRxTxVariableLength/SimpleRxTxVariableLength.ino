/*
 * Zeta RF v2 Getting Started Code Example for VL (Variable Length) packets.
 * Basic example on how to send messages back and forth between two modules using variable size packets.
 *
 * Usage: write this sample on both boards and chat over serial!
 */

#include <ZetaRf.hpp>

//
constexpr size_t MaxPacketLength {32};

ZetaRf868_VL<ZetaRf::nSEL<10>, ZetaRf::SDN<9>, ZetaRf::nIRQ<8>> zeta;

char data[MaxPacketLength] = "Hello World!";


void setup()
{
  Serial.begin(115200);
  while (!Serial); // Might wait for actual serial terminal to connect when over USB
  
  Serial.println("Starting Zeta TxRx VL...");

  // Initialize Zeta module with default packet size (0 for VL)
  if (!zeta.beginWithMaxPacketLengthOf(MaxPacketLength)) {
    Serial.println(F("ZetaRf begin failed. Check wiring?"));
    while(true);
  }

  // Print some info about the chip
  auto const& pi = zeta.readPartInformation();
  Serial.println("----------");
  Serial.print("Chip rev: "); Serial.println(pi.CHIPREV);
  Serial.print("Part    : "); Serial.println(pi.PART, HEX);
  Serial.print("PBuild  : "); Serial.println(pi.PBUILD);
  Serial.print("ID      : "); Serial.println(pi.ID);
  Serial.print("Customer: "); Serial.println(pi.CUSTOMER);
  Serial.print("Rom ID  : "); Serial.println(pi.ROMID);
  Serial.print("Bond    : "); Serial.println(pi.BOND);
  Serial.print('\n');

  /*auto const& fi = zeta.readFunctionRevisionInformation();
  Serial.print("Rev Ext   : "); Serial.println(fi.REVEXT);
  Serial.print("Rev Branch: "); Serial.println(fi.REVBRANCH);
  Serial.print("Rev Int   : "); Serial.println(fi.REVINT);
  Serial.print("Patch     : "); Serial.println(fi.PATCH);
  Serial.print("Func      : "); Serial.println(fi.FUNC);
  Serial.print("SVN Flags : "); Serial.println(fi.SVNFLAGS);
  Serial.print("SVN Rev   : "); Serial.println(fi.SVNREV, HEX);
  Serial.println("----------");//*/
  

  // Start continuous listening on channel 4 (auto-returns to listening after reception of a packet)
  //zeta.startListeningOnChannel(4);

  // Or just listen for one packet then wait (needed to be able to read a packet's RSSI)
  zeta.startListeningSinglePacketOnChannel(4);

  Serial.println(F("Init done."));
}



void loop()
{
  // At least one of the zeta.checkFor*Event*() methods must be called in order to update the library.
  // checkForEvent() returns any event of: Event::CrcError | Event::PacketTransmitted | Event::PacketReceived | Event::LatchedRssi
  //                                     | Event::TxFifoAlmostEmpty | Event::RxFifoAlmostFull | Event::FifoUnderflowOrOverflowError
  // checkForAnyEventOf(filter) enables you to filter events.
  // checkForAllEventsOf(filter) will return ZetaRf::Event::None unless all events in filter are present.
  //
  // Unfiltered events are accessible via zeta.events(). Use zeta.clearEvents([event]) to clear all or specified events.

  if (ZetaRf::Events const ev = zeta.checkForEvent()) {
    if (ev & ZetaRf::Event::DeviceBusy) {
      // DeviceBusy error usually means the radio module is unresponding and need a reset.
      Serial.println(F("Error: Device Busy! Restarting..."));

      if (!zeta.begin()) {
        Serial.println(F("ZetaRf begin failed after comm error."));
        while (true);
      }
      zeta.restartListeningSinglePacket();
    }

    if (ev & ZetaRf::Event::PacketReceived) {
      // We'll read data later
      // Get RSSI (only valid in single packet RX, before going back to RX)
      uint8_t rssi = zeta.latchedRssiValue();

      // Restart listening on the same channel
      zeta.restartListeningSinglePacket();

      Serial.print(F("Packet received with RSSI: "));
      Serial.println(rssi);
    }

    if (ev & ZetaRf::Event::PacketTransmitted) {
      // Back to RX afer TX
      zeta.restartListeningSinglePacket();

      Serial.println(F("Packet transmitted"));
    }
  }



  // Read incoming packet and print it
  if (zeta.available()) {
    uint8_t readSize {0};
    if (zeta.readVariableLengthPacketTo((uint8_t*)data, &readSize)) {

      //zeta.restartListeningSinglePacket(); // If not in checkForEvent

      // Print!
      Serial.print("RX (");
      Serial.print(int(readSize));
      Serial.print(") >");
      Serial.write(data, readSize);
      Serial.println("<");

      // Print in HEX
      /*for (uint8_t i = 0; i < zeta.packetLength(); i++) {
        Serial.print(data[i], HEX);
      }
      Serial.println("<");//*/
    }
  }


  // Send any data received from serial
  if (Serial.available()) {
    // Check FIFO space first
    if (zeta.requestBytesAvailableInTxFifo() > 0) {
      int const s = Serial.readBytes(data, MaxPacketLength);

      Serial.print("TX (");
      Serial.print(s);
      Serial.print(") >");
      Serial.write(data, s);
      Serial.println("<");

      // Send buffer
      zeta.sendVariableLengthPacketOnChannel(4, (const uint8_t*)data, s);
    }
  }

  delay(10); // Hooo, not too fast! But you could...
}
