/*
 * Zeta RF v2 Getting Started Code Example.
 * Basic example on how to send messages back and forth between two modules using fixed size packets.
 *
 * Usage: write this sample on both boards and chat over serial!
 */

#include <ZetaRF.h>

// Zeta modules transmit messages using fixed size packets, define here the max size you want to use. For variable length packets, see the corresponding example.
// Radio configurations also sets a default packet size, usually 8, that you can get using zeta.defaultPacketLength().
constexpr size_t ZetaRFPacketLength {8};

ZetaRF868<ZetaRF::nSEL<10>, ZetaRF::SDN<9>, ZetaRF::nIRQ<8>> zeta;
//old- ZetaRF zeta(10,9,8);

char data[ZetaRFPacketLength] = "Hello ";


void setup()
{
  Serial.begin(115200);
  while (!Serial); // Might wait for actual serial terminal to connect when over USB
  
  Serial.println("Starting Zeta TxRx...");

  // Initialize Zeta module with a specific packet size
  if (!zeta.beginWithPacketLengthOf(ZetaRFPacketLength)) {
  //old- if (!zeta.begin(4, ZetaRFPacketLength)) {
    Serial.println(F("ZetaRF begin failed. Check wiring?"));
    while(true);
  }

  // Print some info about the chip
  EZRadioReply::PartInfo const& pi = zeta.readPartInformation();
  //old- const Si4455_PartInfo &pi = zeta.readPartInfo();
  Serial.println("----------");
  Serial.print("Chip rev: "); Serial.println(pi.CHIPREV);
  Serial.print("Part    : "); Serial.println(pi.PART.U16, HEX);
  Serial.print("PBuild  : "); Serial.println(pi.PBUILD);
  Serial.print("ID      : "); Serial.println(pi.ID.U16);
  Serial.print("Customer: "); Serial.println(pi.CUSTOMER);
  Serial.print("Rom ID  : "); Serial.println(pi.ROMID);
  Serial.print("Bond    : "); Serial.println(pi.BOND);
  Serial.print('\n');

  /*EZRadioReply::FuncInfo const& fi = zeta.readFunctionRevisionInformation();
  //old- const Si4455_FuncInfo &fi = zeta.readFuncInfo();
  Serial.print("Rev Ext   : "); Serial.println(fi.REVEXT);
  Serial.print("Rev Branch: "); Serial.println(fi.REVBRANCH);
  Serial.print("Rev Int   : "); Serial.println(fi.REVINT);
  Serial.print("Patch     : "); Serial.println(fi.PATCH.U16);
  Serial.print("Func      : "); Serial.println(fi.FUNC);
  Serial.print("SVN Flags : "); Serial.println(fi.SVNFLAGS);
  Serial.print("SVN Rev   : "); Serial.println(fi.SVNREV.U32, HEX);
  Serial.println("----------");//*/
  

  // Start continuous listening on channel 4 (auto-returns to listening after reception of a packet)
  // ! BEWARE ! Auto-return to RX may fail after a while and leave the radio module unresponding (call begin again to fix).
  //zeta.startListeningOnChannel(4);
  //old- zeta.startListening();
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
  // checkForAllEventsOf(filter) will return ZetaRF::Event::None unless all events in filter are present.
  //
  // Unfiltered events are accessible via zeta.events(). Use zeta.clearEvents([event]) to clear all or specified events.

  if (ZetaRF::Events const ev = zeta.checkForEvent()) {
    if (ev & ZetaRF::Event::DeviceBusy) {
      // DeviceBusy error usually means the radio module is unresponding and need a reset.
      Serial.println(F("Error: Device Busy! Restarting..."));

      if (!zeta.beginWithPacketLengthOf(ZetaRFPacketLength)) {
        Serial.println(F("ZetaRF begin failed after comm error."));
        while (true);
      }
      zeta.restartListeningSinglePacket();
    }
    /*if (ev & ZetaRF::Event::LatchedRssi) {
      // Latched RSSI is cleared when restarting RX
      uint8_t rssi = zeta.latchedRssiValue();
      Serial.print(F("RSSI: "));
      Serial.println(rssi);
    }//*/
    if (ev & ZetaRF::Event::PacketReceived) {
      // We'll read data later
      // Get RSSI (only valid in single packet RX, before going back to RX)
      uint8_t rssi = zeta.latchedRssiValue();

      // Restart listening on the same channel
      zeta.restartListeningSinglePacket();

      Serial.print(F("Packet received with RSSI: "));
      Serial.println(rssi);
    }
    if (ev & ZetaRF::Event::PacketTransmitted) {
      // Back to RX afer TX
      zeta.restartListeningSinglePacket();

      Serial.println(F("Packet transmitted"));
    }
    /*if (ev & ZetaRF::Event::TxFifoAlmostEmpty) {
      Serial.println("TX Fifo almost empty");
    }//*/
  }



  // Read incoming packet and print it
  if (zeta.available()) {
    if (zeta.readPacketTo((uint8_t*)data)) { // Uses packet length set at zeta.beginWithPacketLengthOf(). Data buffer must be large enough!
      //zeta.readPacketTo((uint8_t*)data, ZetaRFPacketLength); // Alternative way, but be careful to not read more than the packet length.
      //old- zeta.readPacket(data)

      //zeta.restartListeningSinglePacket(); // If not in checkForEvent

      // Print!
      Serial.print("RX> ");
      Serial.write(data, ZetaRFPacketLength); Serial.println();
      // Print in HEX
      for (uint8_t i = 0; i < zeta.packetLength(); i++) {
        Serial.print(data[i], HEX);
      }
      Serial.println("<");
    }
  }


  // Send any data received from serial
  if (Serial.available()) {
    // Check FIFO space first
    if (zeta.requestBytesAvailableInTxFifo() >= ZetaRFPacketLength) {
      int s = Serial.readBytes(data, ZetaRFPacketLength);

      // Pad with zeros
      for (unsigned int i = s; i < ZetaRFPacketLength; i++) {
        data[i] = 0;
      }

      Serial.print("TX>");
      Serial.write(data, ZetaRFPacketLength);
      Serial.println("<");

      // Send buffer
      zeta.sendFixedLengthPacketOnChannel(4, (const uint8_t*)data);
      // Module will automatically return to listening mode
    }
  }

  delay(10); // Hooo, not too fast! But you could...
}
