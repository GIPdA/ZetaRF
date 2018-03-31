ZetaRF ![Version 1.1.11](https://img.shields.io/badge/Version-1.1.11-blue.svg)
======
**Written by**: *Benjamin Balga.*
**Copyright**: ***2018***, *Benjamin Balga*, released under BSD license.

**Supported (tested) Platforms**: *(should be compatible with any platform supporting SPI at 1MHz)*      
![Basic Arduino](https://img.shields.io/badge/Arduino-AVR-brightgreen.svg)
![Arduino Zero](https://img.shields.io/badge/Arduino_Zero-SAMD-yellowgreen.svg)
![Arduino Due](https://img.shields.io/badge/Arduino_Due-SAM-orange.svg)
![Teensy 2 upto 3.5 including LC](https://img.shields.io/badge/Teensy-2_to_3.5,_LC-brown.svg)


## About
ZetaRF is an Arduino library for ZETA modules from [RF Solutions][2], which implements a Silicon Labs [Si4455][3] Low Current Sub-GHz Transceiver

RF Solutions does not provide ready to use sample code in C/C++ to use their ZETA modules directly, but either PIC ASM code or Arduino library if you interface your module with their CODEC chip. This library enables you to communicate directly with the ZETA module without the CODEC chip.

ZETA modules communicates over high-speed SPI and 2 gpio.

This library is based on code examples from Silicon Labs [Wireless Development Suite][1], largely modified.

**ZetaRF library is optimized for very low latency (a few milliseconds). That costs a bit of reliability, packets may be dropped.** You can expect about 4ms of latency for a 8-byte packet.

--

**Beta release, everything has not been tested yet!**

> This library may not be directly compatible with any board implementing Si4455 chip! Currently it was only tested with the ZETA-868-SO and ZETA-433-SO modules from RF Solutions.

ZETA module datasheet: <https://www.rfsolutions.co.uk/downloads/1456219226DS-ZETA.pdf>



## Data Packets

The Si4455 chip has two options: variable length or fixed length packets.
Both cannot be used together, write

	#define VARIABLE_LENGTH_ON
before including ZetaRF.h to activate variable length packets. *See the warning below about variable length packets.*


### Fixed size packets
The library use fixed size packets by default.

Default packet size is 8 bytes, but you can specify the size you want by passing the value to `begin(channel, packet size)`.

Receive and sending methods use this packet length by default unless you specify the length to receive/send (thus you're not limited to the length specified in begin(), but you must tell the send and read methods what size you expect in that case).

The buffer pointed by data in `readPacket()` must be at least the size of the packet.


### Variable length packets

When variable length is activated, the first byte in the buffer is the packet length (size byte excluded), at sending and receiving.

To receive variable length packets, listening is done with a length of zero (handled automatically by `startListening()` and `startListening(channel)`).

At this time you cannot peek the packet size before read.

> Few tests have shown that variable packet may not be very reliable, extensive testings are needed.


## Usage

### Including the SPI header file 

It has been reported that on the latest Arduino IDE you have to explicitly add the following before you include the ZetaRF.h file, even though 
the SPI.h file is included by the ZetaRF.h file itself. Not doing this will generate many compiler warnings about a 'missing' SPI.h file.

	#include <SPI.h>


### 868 MHz or 433 MHz operation

By default this library configures the Zeta for 868 MHz operation. If you instead have a 433MHz module, then add this before including the ZetaRF.h file:

	#define ZETARF_FREQUENCY_433MHZ 1


### Power Supply
Zeta modules **only supports 1.8V to 3.6V operation**, and malfunction issues have been reported with 5V-level signals (e.g. 5V Arduino with 3.3V Zeta). Do not use voltage levels greater than VDD. If you can, power everything with 3.3V or use level shifters (voltage divider or else).


### Send data

	void sendPacket(const uint8_t *data);
	
Transmit a packet (channel and packet size set in begin()). FIFO size is 64 bytes (data is overwritten if you overshoot).

To transmit on a different channel, use:

	void sendPacket(uint8_t channel, const uint8_t *data);


#### Check if data was transmitted

	bool checkTransmitted();

Returns true when data was successfully sent.

The chip automatically goes back to listening when the transmit is done.



### Receive data

#### Switch to receive mode

	void startListening();

Chip must be in listening mode in order to receive data (listening channel set in `begin()`).

To receive on a different channel, use:

	void startListening(uint8_t channel);


#### Check if data is available

	bool checkReceived();

Returns true if data is available.


#### Read data

	uint8_t readPacket(uint8_t *data);

Read a packet from FIFO, returns the number of bytes read or -1 on error.  FIFO has a capacity of 64 bytes.


#### Errors

	bool systemError();

Returns true if a system error occured (auto clears). System error happens when the module is not responding correctly. In that case, reset the module by calling begin() again.


#### Examples

See code examples for more details.
(TODO: add more examples)


## Installation
Copy the library folder to your Arduino library folder.

## Pin connexions

ZETA Pin #|ZETA Module|Arduino|Description
----------|-----------|-------|-----------
1         |ANT        |-      |Antenna (small wire for tests works great (86mm long for 868MHz))
2         |GND        |GND    |Power Ground
3         |SDN        |GPIO   |Shutdown (active high)
4         |VDD        |VDD    |Power (1.8V to 3.6V)
5         |nIRQ       |GPIO   |IRQ (active low)
6         |NC         |-      |Not Connected
7         |GPIO1      |-      |Not Used
8         |GPIO2      |-      |Not Used
9         |SCLK       |SCLK   |SPI Clock
10        |SDI        |MOSI   |Zeta SPI In to Arduino SPI Out
11        |SDO        |MISO   |Zeta SPI Out to Arduino SPI In
12        |nSEL       |CS     |Chip Select (active low)

(**Note** - Possible improvement: a GPIO could be used for CTS instead of polling a register)


## Configuration files
The project contains WDS XML configuration files, used to generate `radio_config_xx.h` files, if you want to change some settings. Be careful, it might brake things if you don't know what you're doing.

Note: FRR are used and enforced by the library (on begin()) to read module state and interrupt registers. Changing the FRR configuration after begin() will break the library.


## Todo List
- More examples
- Frequency selection with enum instead of define
- Configurable auto-return to listening mode
- Sleep/Active status commands
- Add available() method and FIFO capacity getters


## License
See LICENSE file.

[1]: http://www.silabs.com/products/wireless/EZRadio/Pages/Si4455.aspx "Wireless Development Suite"
[2]: https://www.rfsolutions.co.uk/radio-modules-c10/name-c49/zeta-c86 "RF Solutions"
[3]: http://www.silabs.com/products/wireless/EZRadio/Pages/Si4455.aspx "Si4455"

