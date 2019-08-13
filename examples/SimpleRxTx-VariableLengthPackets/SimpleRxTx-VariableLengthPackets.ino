/*
 * Zeta RF Getting Started Code Example.
 * Basic example on how to send messages back and forth between two modules.
 *
 * Usage: write this sample on both boards and send text over serial!
 */

#include <ZetaRF.h>

ZetaRF868_VL<ChipSelectPin<10>, ShutdownPin<9>, IrqPin<8>> zeta;

constexpr int MaxBufferSize {20};
char data[MaxBufferSize] = "Hello World!";
bool transmitting = false;


void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting Zeta TxRx...");

  pinMode(14,INPUT_PULLUP);
  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);

  // Initialize Zeta module
  if (!zeta.begin()) {
    Serial.println("Zeta begin failed");
    while (true);
  }
  
  zeta.startListeningOnChannel(4);

  // Print some info about the chip
  const Si4455_PartInfo &pi = zeta.readPartInformation();
  Serial.println("----------");
  Serial.print("Chip rev: "); Serial.println(pi.CHIPREV);
  Serial.print("Part    : "); Serial.println(pi.PART.U16, HEX);
  Serial.print("PBuild  : "); Serial.println(pi.PBUILD);
  Serial.print("ID      : "); Serial.println(pi.ID.U16);
  Serial.print("Customer: "); Serial.println(pi.CUSTOMER);
  Serial.print("Rom ID  : "); Serial.println(pi.ROMID);
  Serial.print("Bond    : "); Serial.println(pi.BOND);
  Serial.print('\n');

  const Si4455_FuncInfo &fi = zeta.readFunctionRevisionInformation();
  Serial.print("Rev Ext   : "); Serial.println(fi.REVEXT);
  Serial.print("Rev Branch: "); Serial.println(fi.REVBRANCH);
  Serial.print("Rev Int   : "); Serial.println(fi.REVINT);
  Serial.print("Patch     : "); Serial.println(fi.PATCH.U16);
  Serial.print("Func      : "); Serial.println(fi.FUNC);
  Serial.print("SVN Flags : "); Serial.println(fi.SVNFLAGS);
  Serial.print("SVN Rev   : "); Serial.println(fi.SVNREV.U32);
  Serial.println("----------");//*/
  
  // Set module in receive mode
  //zeta.startListening();

  Serial.println("Init done.");
}


void sendString(const char* str)
{
  Serial.print("Sending: ");
  Serial.println(str);
  zeta.sendVariableLengthPacket(4, (const uint8_t*)str, strlen(str));
}


void loop()
{
  if (!digitalRead(14)) {
    delay(100);
    sendString("Hello!!!");
  }

  if (!digitalRead(15)) {
    delay(100);
    sendString("Hello Wor");
  }

  if (!digitalRead(16)) {
    delay(100);
    sendString("He");
  }

  if (!digitalRead(17)) {
    delay(100);
    sendString("0132456789ABC");
  }

  
  // Send any data received from serial
  /*if (Serial.available() && !transmitting) {
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
    uint8_t packetSize = 0;
    zeta.readVariableLengthPacket((uint8_t*)data, MaxBufferSize, packetSize);
    Serial.print(packetSize); Serial.print(": ");
    
    uint8_t* data_ = (uint8_t*)data;
    for (int i = 0; i < packetSize; i++) {
      //Serial.print(uint8_t(*data_++), HEX);
      Serial.print(char(*data_++));
    }
    Serial.println();
  }//*/
  
  delay(10);
}

