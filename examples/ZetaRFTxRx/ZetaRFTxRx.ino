/*
 * Zeta RF Getting Started Code Example.
 * Basic example on how to send messages back and forth between two modules.
 *
 * Usage: write this sample on both boards and send text over serial!
 */

#include <ZetaRF.h>

// Zeta modules transmit messages using fixed size packets, define here the max size you want to use
#define ZETARF_PACKET_LENGTH 16

ZetaRF zeta(10, 9, 8);  // Pins: SPI CS, Shutdown, IRQ

char data[ZETARF_PACKET_LENGTH] = "Hello World!";
bool transmitting = false;


void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting Zeta TxRx...");

  // Initialize Zeta module, specifing channel and packet size
  zeta.begin(2, ZETARF_PACKET_LENGTH);

  // Print some info about the chip
  /*const Si4455_PartInfo &pi = zeta.readPartInfo();
  Serial.println("----------");
  Serial.print("Chip rev: "); Serial.println(pi.CHIPREV);
  Serial.print("Part    : "); Serial.println(pi.PART.U16);
  Serial.print("PBuild  : "); Serial.println(pi.PBUILD);
  Serial.print("ID      : "); Serial.println(pi.ID.U16);
  Serial.print("Customer: "); Serial.println(pi.CUSTOMER);
  Serial.print("Rom ID  : "); Serial.println(pi.ROMID);
  Serial.print("Bond    : "); Serial.println(pi.BOND);
  Serial.print('\n');

  const Si4455_FuncInfo &fi = zeta.readFuncInfo();
  Serial.print("Rev Ext   : "); Serial.println(fi.REVEXT);
  Serial.print("Rev Branch: "); Serial.println(fi.REVBRANCH);
  Serial.print("Rev Int   : "); Serial.println(fi.REVINT);
  Serial.print("Patch     : "); Serial.println(fi.PATCH.U16);
  Serial.print("Func      : "); Serial.println(fi.FUNC);
  Serial.print("SVN Flags : "); Serial.println(fi.SVNFLAGS);
  Serial.print("SVN Rev   : "); Serial.println(fi.SVNREV.U32);
  Serial.println("----------");//*/
  
  // Set module in receive mode
  zeta.startListening();

  Serial.println("Init done.");
}



void loop()
{
  // Send any data received from serial
  if (Serial.available() && !transmitting) {
    int s = Serial.readBytes(data, ZETARF_PACKET_LENGTH);

    // Pad with zeros
    for (int i = s; i < ZETARF_PACKET_LENGTH; i++) {
      data[i] = 0;
    }

    Serial.print("Sending >");
    Serial.write(data, ZETARF_PACKET_LENGTH);
    Serial.print("<\n");

    // Send buffer
    transmitting = true;  // Only one at a time!
    zeta.sendPacket((const uint8_t*)data);  // Use channel set with begin()
    // Module will automatically return to listening mode
  }

  // Check if message was transmitted successfully
  if (zeta.checkTransmitted()) {
    transmitting = false;
    Serial.println("msg transmitted");
  }//*/

  // Check incoming messages and print
  if (zeta.checkReceived()) {
    Serial.print("> ");
    zeta.readPacket((uint8_t*)data);
    Serial.write(data, ZETARF_PACKET_LENGTH);
    Serial.print('\n');
  }//*/
  
  delay(10);
}

