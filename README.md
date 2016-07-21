ZetaRF
======
**v0.0.1 !Beta!**

ZetaRF is an Arduino library for ZETA modules from [RF Solutions][2], which implements a Silicon Labs [Si4455][3] Low Current Sub-GHz Transceiver

RF Solutions does not provide ready to use sample code in C/C++ to use their ZETA modules directly, but either PIC ASM code or Arduino library if you interface your module with their CODEC chip. This library enables you to communicate directly with the ZETA module without the CODEC chip.

ZETA modules communicates over high-speed SPI and 2 gpio.

This library is based on code examples from Silicon Labs [Wireless Development Suite][1], largely modified.

**Beta release, everything has not been tested yet!**


## Data Packets
The Si4455 chip transmit and receive fixed size packets, and you have to set the packet size when initializing the library (see begin()).
Packet size cannot be changed without resetting the chip (well... maybe).

## Usage

### Send data

	void sendPacket(const uint8_t *data);
	
Transmit a packet (channel and packet size set in begin()). FIFO size is 64 bytes.

To transmit on a different channel, use:

	void sendPacket(uint8_t channel, const uint8_t *data);

#### Check if data was transmitted

	bool checkTransmitted();

Returns true when data was successfully sent.



### Receive data

#### Switch to receive mode

	void startReceiver();

Chip must be in receive mode in order to receive data (listening channel set in begin()). You will need to switch back to receive mode after sending a packet if you want to receive any more data.

To receive on a different channel, use:

	void startReceiver(uint8_t channel);


#### Check if data is available

	bool checkReceived();

Returns true if data is available.

#### Read data

	uint8_t readPacket(uint8_t *data);

Read a packet from FIFO, returns the number of bytes read.  FIFO has a capacity of 64 bytes.

See code examples for more details.


## Installation
Copy the library folder to your Arduino library folder.

## License
See LICENSE file.

[1]: http://www.silabs.com/products/wireless/EZRadio/Pages/Si4455.aspx "Wireless Development Suite"
[2]: https://www.rfsolutions.co.uk/radio-modules-c10/name-c49/zeta-c86 "RF Solutions"
[3]: http://www.silabs.com/products/wireless/EZRadio/Pages/Si4455.aspx "Si4455"

