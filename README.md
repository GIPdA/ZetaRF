ZetaRF ![Version 2.0.0](https://img.shields.io/badge/Version-2.0.0-blue.svg)
======
**Written by**: *Benjamin Balga.*
**Copyright**: ***2020***, *Benjamin Balga*, released under BSD license.

**Supported (tested) Platforms**: *(should be compatible with any platform supporting SPI at 1MHz)*      
![Basic Arduino](https://img.shields.io/badge/Arduino-AVR-brightgreen.svg)
![Teensy 2 upto 4.1 including LC](https://img.shields.io/badge/Teensy-2_to_4.1,_LC-brown.svg)
![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi-ARM-black.svg)

--
### ZetaRf got updated to v2! Check your code, this is a breaking update from v1. See 'SimpleRxTx' example for a migration guide and the notes below.
--

## About
ZetaRf is an Arduino library for ZETA modules from [RF Solutions][2], which implements a Silicon Labs [Si4455][3] Low Current Sub-GHz Transceiver, and for Dorji DRF4463F modules which implements a Silicon Labs Si4463 Low Current Sub-GHz Transceiver.

Currently the 2 modules are **not** cross-compatible.

RF Solutions does not provide ready to use sample code in C/C++ to use their ZETA modules directly, but either PIC ASM code or Arduino library if you interface your module with their CODEC chip. This library enables you to communicate directly with the ZETA module without the CODEC chip.
Same for Doji modules.

ZETA modules communicates over high-speed SPI (up to 10MHz) and 2 gpio.

This library is based on code examples from Silicon Labs [Wireless Development Suite][1], largely modified.

**ZetaRf library is optimized for very low latency (a few milliseconds). That costs a bit of reliability, packets may be dropped.** You can expect about 4ms of latency for a 8-byte packet.

--

**ZetaRf version 2**

ZetaRf version 2 has breaking changes from the v1, as well as bug fixes and various improvements like the support for multiple radio configurations simultaneously.

Please report any issue you find! :)


> This library may not be directly compatible with any board implementing Si4455 chip! Currently it was only tested with the ZETA-868-SO and ZETA-433-SO modules from RF Solutions, and DRF4463F modules (868 and 433 MHz) from Dorji.

ZETA module datasheet: <https://www.rfsolutions.co.uk/downloads/1456219226DS-ZETA.pdf>



## Data Packets

You have two options for packet sizing: either *fixed length* or *variable length*. The option is choosen when you create the ZetaRf object (suffix `_VL`)

All packets are transmitted with a 16-bit CRC (IBM-16) and checked at reception.


### Fixed size packets:

All packets are transmitted and received with a fixed size. Receiving a wrongly sized packet will mostly result in CRC errors, but you could receive bad messages that happened to make it through.

Default packet size is usually 8 bytes (configured in the radio config files), but you can specify the size you want by using `beginWithPacketLengthOf(<packet length in bytes>)`. Max is 64 bytes.

Receive and sending methods use this packet length by default unless you change it by calling `setPacketLength(<new length>)`


### Variable length packets:

Variable length packets allows for a more efficient use of the RF space when dealing with various messages of different sizes. A size byte is included in the packet and the RX modem automatically handles it.

Receive and sending methods handle the variable part mostly by themselves. Use `beginWithPacketLengthOf(<max packet length in bytes>)` to limit the maximum packet size at RX. 

At this time you cannot peek the packet size before read.


## Usage

### Power Supply
Zeta modules **only supports 1.8V to 3.6V operation**, and malfunction issues have been reported with 5V-level signals (e.g. 5V Arduino with 3.3V Zeta). Do not use voltage levels greater than VDD. If you can, power everything with 3.3V or use level shifters (voltage divider or else).

### Create the radio object

The library uses templates to be very clear about which config and GPIO you use:

	ZetaRf868<ZetaRf::nSEL<10>, ZetaRf::SDN<9>, ZetaRf::nIRQ<8>> zeta; // Pin D10 for NSEL, pin D9 for SDN, etc

`ZetaRf868` denotes the radio configuration. Several configurations are available, and you can use your own if needed.

- `ZetaRf868` : RF Solution's ZETA module in 868MHz
- `ZetaRf433` : RF Solution's ZETA module in 433MHz
- `ZetaRf868_VL` : RF Solution's ZETA module in 868MHz in variable length packet mode
- `ZetaRf433_VL` : RF Solution's ZETA module in 433MHz in variable length packet mode
- `ZetaRf_DRF4463F_868` : Dorji DRF4463F module in 868MHz
- `ZetaRf_DRF4463F_433` : Dorji DRF4463F module in 433MHz
- `ZetaRf_DRF4463F_868_VL` : Dorji DRF4463F module in 868MHz in variable length packet mode (COMING SOON)
- `ZetaRf_DRF4463F_433_VL` : Dorji DRF4463F module in 868MHz in variable length packet mode (COMING SOON)


### Begin

**Must be called before any other method.**

To use the default radio config packet settings:

	zeta.begin()

To specify the packet length (fixed length) or the max packet length (variable length, VL):

	zeta.beginWithPacketLengthOf(<packet length>)
	zeta.beginWithMaxPacketLengthOf(<max packet length>) // For VL


### Event loop
You must poll the lib for new messages or any event that occured:

	ZetaRf::Events const ev = zeta.checkForEvent(); // Check default events

Or check for specific events:

	Events checkForAnyEventOf(Events filter) // e.g. checkForAnyEventOf(ZetaRf::Event::PacketReceived | ZetaRf::Event::PacketTransmitted)
	Events checkForAllEventsOf(Events filter) // Waits for all given events at once

`ZetaRf::Events` evaluates to true if any event occured, you can then check for any particular event, e.g.:

	if (ev & ZetaRf::Event::PacketReceived) { ... }

Events are cleared and do not persist unless they are not "checked" by the above methods.

You can get current events with `events()` and clear events using `clearEvents()` or `clearEvents(<events>)`.

> *Note: `PacketReceived` will be cleared even if data is left in the FIFO after read (if you don't read it all). Loop on `available()` or `hasDataAvailable()` to read everything.*


Events available (Default event = checked by `checkForEvent()`):

Event | Default event | Description
------|---------------|------------
PacketTransmitted            | x | Packet transmit completed
PacketReceived               | x | Valid packet received and waiting in FIFO. Don't forget to restart listening if you listened for a single packet!
CrcError                     | x | CRC Error detected on the received packet (FIFO is reset, packet lost)
TxFifoAlmostEmpty            | x | Tx FIFO almost empty
RxFifoAlmostFull             | x | Rx FIFO almost full
InvalidSync                  |   | Invalid Sync frame detected (RF modem)
InvalidPreamble              |   | Invalid Preamble detected (RF modem)
DetectedPreamble             |   | Preamble detected (RF modem)
DetectedSync                 |   | Sync frame detected (RF modem)
LatchedRssi                  | x | RSSI value latched on packet receive (read with latchedRssiValue(), value is reset when restarting RX )
FifoUnderflowOrOverflowError | x | Underflow or overflow on the FIFO (FIFO is reset)
StateChange                  |   | Radio state changed
ChipReady                    |   | Chip ready to accept commands (after power up)
DeviceBusy                   | x | The chip didn't respond to a command (sign of comm errors)

>*Notes: </br>
>- You should restart listening on any error if you listened for a single packet. </br>
>- DeviceBusy can be considered a "fatal error", it is recommended to call begin to reset the chip upon getting this error.*


### Send data

Transmit a packet, for both fixed or variable length configs:

	bool sendPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100)

For fixed length packets only:

	bool sendFixedLengthPacketOnChannel(uint8_t channel, uint8_t const* data, unsigned long timeoutForReady_ms = 100)

For variable length packets only:

	bool sendVariableLengthPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100)
	
Max data size is 64 bytes. Returns true if the transmission started (it doesn't wait for completion), false on error (timeout or comm error).

A `PacketTransmitted` event will be raised when the packet transmit completed. The chip will automatically return to the active mode before the transmit.



### Receive data

#### Switch to receive mode

Start listening packets on a channel, and automatically return to RX after packet reception:

	bool startListeningOnChannel(uint8_t newChannel)

To listen for only one packet (needed to read the packet's RSSI value):

	bool startListeningSinglePacketOnChannel(uint8_t newChannel)

To restart listening on the same channel as previously:

	bool restartListeningSinglePacket()




#### Read data

Use `available()` or `hasDataAvailable()` while data is available to read.

Common method to read either a fixed or variable length packet (max data size is the packet length, or for VL configs it is the max packet length set with `beginWithMaxPacketLengthOf` or `setMaxRxPacketLength`):

	ZetaRf::ReadPacketResult readPacketTo(uint8_t* data)

To specifically read a fixed length packet:

	ZetaRf::ReadPacketResult readFixedLengthPacketTo(uint8_t* data, uint8_t packetLength)
	
To specifically read a variable length packet (optional, use the `rxPacketDataLength` pointer to get the received packet length (which is also in ReadPacketResult) )

	ZetaRf::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t* rxPacketDataLength) // Max data size is the max packet length set via beginWithMaxPacketLengthOf or setMaxRxPacketLength.
	ZetaRf::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxDataSize, uint8_t* rxPacketDataLength)

*Notes: Reads directly from FIFO. FIFO has a capacity of 64 bytes.*


All methods returns an object that evaluates to true if no error occured, or false on error. It contains the error value if any, and the received packet size (`receivedPacketSize()`).

Simple way:

	if (zeta.readPacketTo(data)) { /* Valid packet */ }

Or to check error values:

	ZetaRf::ReadPacketResult const result = zeta.readPacketTo(data);
	if (result != ZetaRf::ReadPacketResult::Success) { /* Error */ }
	
	// or
	
	switch (result.value()) {
	case ZetaRf::ReadPacketResult::PacketSizeLargerThanBuffer:
		// do something?
		... etc
	}


Error Values | Description
-------------|------------
Success                    | Packet received and valid
PacketSizeLargerThanBuffer | VL mode, received packet is bigger than the given data buffer (use `receivedPacketSize()` to get the size)
InvalidPacketSize          | VL mode, the size byte of the received packet is invalid (either null or bigger than the FIFO capacity)
RequestFailed              | Comm error with the chip
InvalidArgument            | Invalid argument(s) passed to the method
NotEnoughDataInFifo        | The FIFO doesn't contain the amout of data you requested
Timeout                    | Comm timeout




#### Examples

See code examples for more details.
(TODO: add more examples)

To build the Raspberry Pi examples, run `cmake .` followed by `make` from the root directory. The binaries will be located in the new `build` folder.


## Installation

### Arduino

Copy the library folder to your Arduino library folder.

### Raspberry Pi

Copy the library folder to your project folder. Add ZetaRF to your `CMakeLists.txt` file with

```cmake
add_subdirectory(ZetaRF)

add_compile_definitions(WIRINGPI)
include_directories(ZetaRF/src)

add_executable(MyProject ${SOURCES})
target_link_libraries(MyProject zetarf)
```

## Pin connections

ZETA Pin #|ZETA Module|Arduino|Pi  |Description
----------|-----------|-------|----|-----------
1         |ANT        |-      |-   |Antenna (small wire for tests works great (86mm long for 868MHz))
2         |GND        |GND    |GND |Power Ground
3         |SDN        |GPIO   |GPIO|Shutdown (active high)
4         |VDD        |VDD    |3v3 |Power (1.8V to 3.6V)
5         |nIRQ       |GPIO   |GPIO|IRQ (active low)
6         |NC         |-      |-   |Not Connected
7         |GPIO1      |-      |-   |Not Used or Hardware CTS for DRF4463F modules (active low)
8         |GPIO2      |-      |-   |Not Used
9         |SCLK       |SCLK   |SCLK|SPI Clock
10        |SDI        |COPI   |COPI|Zeta SPI In to Arduino SPI Out (Controller Out, Peripheral In)
11        |SDO        |CIPO   |CIPO|Zeta SPI Out to Arduino SPI In (Controller In, Peripheral Out)
12        |nSEL       |CS     |GPIO|Chip Select (active low)



## Configuration files
The project contains WDS XML configuration files, used to generate configuration files, if you want to change some settings. Be careful, it might brake things if you don't know what you're doing.

Note: FRR are used and enforced by the library to read module state and interrupt registers. Changing the FRR configuration will break the library.


## Credits
All contributions are welcome! Open an issue or make a Pull Request to contribute.

@adamfowleruk - 433MHz configurations

@hallgchris - Raspberry Pi port


## Todo List
- More examples
- Frequency selection with enum instead of define
- Configurable auto-return to listening mode
- Sleep/Active status commands


## License
See LICENSE file.

[1]: https://www.silabs.com/wireless/proprietary/ezradio-sub-ghz-ics/device.si4455 "Wireless Development Suite"
[2]: https://www.rfsolutions.co.uk/radio-modules-c10/name-c49/zeta-c86 "RF Solutions"
[3]: https://www.silabs.com/wireless/proprietary/ezradio-sub-ghz-ics/device.si4455 "Si4455"

