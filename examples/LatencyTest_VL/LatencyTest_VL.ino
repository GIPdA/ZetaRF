/*
 * Zeta RF v2 Latency Test with VL (Variable Length) packets
 *
 * Usage: write this sample on both boards
 * 
 * Best time on 2 Teensy 3.2 (RSSI ~230 @ 50cm apart)
 *  @ 8b/packet:  4.60ms latency 
 *  @ 16b/packet: 5.75ms latency
 *  @ 32b/packet: 8.03ms latency
 *  Approx delay: 3.457ms + 0.142916667ms/byte
 */

#include <ZetaRf.hpp>

constexpr size_t ZetaRfPacketLength {32};
constexpr size_t ZetaRfPacketMinLength {8};

ZetaRf868_VL<ZetaRf::nSEL<10>, ZetaRf::SDN<9>, ZetaRf::nIRQ<8>> zeta;

char data[ZetaRfPacketLength] = "Hello ";

bool runTest = false; // send 'r' over serial to start, 's' to stop
unsigned long txTime = 0;

unsigned long txCount = 0;
unsigned long rxCount = 0;


void setup()
{
    Serial.begin(115200);
    while (!Serial); // Might wait for actual serial terminal to connect over USB

    Serial.println("Starting ZetaRf Variable Length Latency Test...");

    if (!zeta.beginWithPacketLengthOf(ZetaRfPacketLength)) {
        Serial.println(F("ZetaRf begin failed. Check wiring?"));
        while (true);
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
    //Serial.print("Bond    : "); Serial.println(pi.BOND);
    Serial.print('\n');//*/

    zeta.startListeningSinglePacketOnChannel(4);

    Serial.println("Init done.");
    Serial.println("Type 'r' to start test, 's' to stop.");
}



void loop()
{
    if (ZetaRf::Events const ev = zeta.checkForEvent()) {
        if (ev & ZetaRf::Event::PacketReceived) {
            // Back to RX after TX, don't spend time here for that

            //Serial.print("Device state: ");
            //Serial.println(static_cast<int>(zeta.radioState()));

            if (!runTest) {
                // Send packet back ASAP
                uint8_t readSize {0};
                if (zeta.readVariableLengthPacketTo((uint8_t*)data, &readSize)) {
                    zeta.sendVariableLengthPacketOnChannel(4, (const uint8_t*)data, readSize);
                }
                uint8_t rssi = zeta.latchedRssiValue();
                rxCount++;

                Serial.print("RX (");
                Serial.print(int(readSize));
                Serial.print(") ");
                Serial.print(int(data[0]));
                Serial.print(" # ");
                Serial.print(rxCount);
                Serial.print(" at RSSI ");
                Serial.println(int(rssi));
            } else {
                // Print trip time
                uint8_t readSize {0};
                if (zeta.readVariableLengthPacketTo((uint8_t*)data, &readSize)) {
                    unsigned long const rxTime = micros();
                    float const latency = (rxTime-txTime)/2;

                    uint8_t rssi = zeta.latchedRssiValue();

                    Serial.print("RX (");
                    Serial.print(int(readSize));
                    Serial.print(") ");
                    Serial.print(int(data[0]));
                    Serial.print(" in ");
                    Serial.print(rxTime-txTime);
                    Serial.print("us");

                    Serial.print("   Latency = ");
                    Serial.print(latency/1000.);
                    Serial.print("ms");

                    Serial.print("  RSSI ");
                    Serial.println(int(rssi));
                }
                // Cleanup, not really needed but just in case
                zeta.requestResetRxFifo();
                txTime = 0;
                delay(30);
            }
        }

        if (ev & ZetaRf::Event::PacketTransmitted) {
            zeta.restartListeningSinglePacket();
            Serial.println("Msg transmitted");
        }

        /*if (ev & ZetaRf::Event::LatchedRssi) {
            uint8_t rssi = zeta.latchedRssiValue();
            Serial.print("RSSI: ");
            Serial.println(int(rssi));
        }//*/
    }

    if (runTest) {
        if (txTime == 0) {
            data[0]++;

            //Serial.print("\nDevice state: ");
            //Serial.println(static_cast<int>(zeta.radioState()));
            static size_t sendSize = ZetaRfPacketMinLength-1;
            if (++sendSize > ZetaRfPacketLength) {
                sendSize = ZetaRfPacketMinLength;
            }
            
            txTime = micros();
            if (!zeta.sendVariableLengthPacketOnChannel(4, (const uint8_t*)data, sendSize)) {
                Serial.println("Failed to send!");
                runTest = false;
            } else {
                txCount++;
                
                Serial.print("TX ");
                Serial.print(sendSize);
                Serial.print(" # ");
                Serial.println(txCount);
            }
        }
        /*else if ((micros() - txTime) > 20000) {
            // Spam TX
            txTime = 0;
        }//*/
        else if ((micros() - txTime) > 100000) {
            // No reply
            txTime = 0;
            runTest = false;
            Serial.println("Packet lost - No reply");
        }
    }

    // Send any data received from serial
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') {
            runTest = true;
        }
        if (c == 's') { // Stop test
            runTest = false;
            txTime = 0;

            // Wait and clear RX or they will rebel and speak forever.
            delay(100);
            if (ZetaRf::Events const ev = zeta.checkForEvent()) {
                if (ev & ZetaRf::Event::PacketTransmitted) {
                    zeta.restartListeningSinglePacket();
                }
            }
            zeta.requestResetRxFifo();
        }

        if (c == 'n') {
            Serial.print("request NOP...");
            if (zeta.requestNop())
                Serial.println(" OK");
            else
                Serial.println(" failed");
        }
        if (c == 'f') {
            Serial.print("request Reset Rx And Tx Fifo...");
            if (zeta.requestResetRxAndTxFifo())
                Serial.println(" OK");
            else
                Serial.println(" failed");
        }
        if (c == 'x') { // Hard reset the radio
            if (!zeta.begin()) {
                Serial.println(F("Zeta begin failed"));
            }
            zeta.startListeningSinglePacketOnChannel(4);
        }
    }

}
