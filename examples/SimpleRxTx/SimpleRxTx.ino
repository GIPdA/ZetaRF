/*
 * Zeta RF Getting Started Code Example.
 * Basic example on how to send messages back and forth between two modules.
 *
 * Usage: write this sample on both boards and send text over serial!
 */

#include <ZetaRF.h>

// Zeta modules transmit messages using fixed size packets, define here the max size you want to use
#define ZETARF_PACKET_LENGTH 8

ZetaRF868<ZetaRF::nSEL<10>, ZetaRF::SDN<9>, ZetaRF::nIRQ<8>> zeta;
//ZetaRF868<ChipSelectPin<10>, ShutdownPin<9>, IrqPin<8>> zeta; // Alternative naming
//old- ZetaRF2 zeta(10,9,8);
constexpr size_t ZetaRFPacketLength {8};

char data[ZetaRFPacketLength] = "Hello ";


void setup()
{
  Serial.begin(115200);
  while (!Serial); // Might wait for actual serial terminal to connect when USB
  
  Serial.println("Starting Zeta TxRx...");

  // Initialize Zeta module with a specific packet size
  if (!zeta.beginWithPacketLengthOf(ZetaRFPacketLength)) {
  //old- if (!zeta.begin(4, ZetaRFPacketLength)) {
    Serial.println(F("Zeta begin failed"));
    while(true);
  }

  // Print some info about the chip
  const Si4455_PartInfo &pi = zeta.readPartInformation();
  //old- const Si4455_PartInfo &pi = zeta.readPartInfo();
  Serial.println("----------");
  Serial.print("Chip rev: "); Serial.println(pi.CHIPREV);
  Serial.print("Part    : "); Serial.println(pi.PART.U16);
  Serial.print("PBuild  : "); Serial.println(pi.PBUILD);
  Serial.print("ID      : "); Serial.println(pi.ID.U16);
  Serial.print("Customer: "); Serial.println(pi.CUSTOMER);
  Serial.print("Rom ID  : "); Serial.println(pi.ROMID);
  Serial.print("Bond    : "); Serial.println(pi.BOND);
  Serial.print('\n');

  const Si4455_FuncInfo &fi = zeta.readFunctionRevisionInformation();
  //old- const Si4455_FuncInfo &fi = zeta.readFuncInfo();
  Serial.print("Rev Ext   : "); Serial.println(fi.REVEXT);
  Serial.print("Rev Branch: "); Serial.println(fi.REVBRANCH);
  Serial.print("Rev Int   : "); Serial.println(fi.REVINT);
  Serial.print("Patch     : "); Serial.println(fi.PATCH.U16);
  Serial.print("Func      : "); Serial.println(fi.FUNC);
  Serial.print("SVN Flags : "); Serial.println(fi.SVNFLAGS);
  Serial.print("SVN Rev   : "); Serial.println(fi.SVNREV.U32);
  Serial.println("----------");//*/
  

  // Start continuous listening on channel 4 (auto-returns to listening after reception of a packet)
  zeta.startListeningOnChannel(4);
  //old- zeta.startListening();
  // Or just listen for one packet then wait (needed to be able to read a packet's RSSI)
  //zeta.startListeningSinglePacketOnChannel(4);

  Serial.println("Init done.");
}



void loop()
{
  // Read incoming packet and print it
  if (zeta.available()) {
    if (zeta.readPacketTo((uint8_t*)data)) { // Uses packet length set at zeta.beginWithPacketLengthOf(). Buffer must be large enough!
      //zeta.readPacketTo((uint8_t*)data, ZetaRFPacketLength); // Alternative way, but be careful to not read more than the packet length.

      // Print!
      Serial.write(data, ZetaRFPacketLength); Serial.println();
      // Print in HEX
      for (uint8_t i = 0; i < zeta.packetLength(); i++) {
        Serial.print(data[i], HEX);
      }
      Serial.println();
    }
  }


  // Send any data received from serial
  if (Serial.available()) {
    // Check FIFO space first
    if (zeta.bytesAvailableInTxFifo() >= ZetaRFPacketLength) {
      int s = Serial.readBytes(data, ZetaRFPacketLength);

      // Pad with zeros
      for (int i = s; i < ZetaRFPacketLength; i++) {
        data[i] = 0;
      }

      Serial.print("Sending >");
      Serial.write(data, ZetaRFPacketLength);
      Serial.print("<\n");

      // Send buffer
      zeta.sendFixedLengthPacket(4, (const uint8_t*)data);
      // Module will automatically return to listening mode
    }
  }


/*
  // Check if message was transmitted successfully
  if (zeta.wasDataTransmitted()) {
    Serial.println("msg transmitted");
  }

  // TODO: check fifo status flags
  if (zeta.isTxFifoAlmostEmpty()) {
    //
  }//*/

  delay(10);
}
