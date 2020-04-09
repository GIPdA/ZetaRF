/*
 *
 */

#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "zetarf_hal.hpp"

#ifdef __AVR__
#include "avr_helpers/type_traits.hpp"
#endif

#define ZETARF_SPI_SETTINGS_DEFAULT SPISettings(1000000UL, MSBFIRST, SPI_MODE0)
#define ZETARF_SPI_SETTINGS         ZETARF_SPI_SETTINGS_DEFAULT


namespace ZetaRFHal {

template <class TChipSelectPin, class TShutdownPin, class TIrqPin>
class ArduinoSpiHal
{
	static_assert(std::is_base_of<ChipSelectPinSelector, TChipSelectPin>::value, "First parameter of ZetaRF must be the Chip Select pin. Use ChipSelectPin<10> to use pin 10.");
	static_assert(std::is_base_of<ShutdownPinSelector, TShutdownPin>::value, "Second parameter of ZetaRF must be the Shutdown pin. Use ShutdownPin<9> to use pin 9.");
	static_assert(std::is_base_of<IrqPinSelector, TIrqPin>::value, "Third parameter of ZetaRF must be the IRQ pin. Use IrqPin<8> to use pin 8.");
 
	static constexpr int ChipSelect_pin = TChipSelectPin::pin;
    static constexpr int ShutdownPin = TShutdownPin::pin;
    static constexpr int IrqPin = TIrqPin::pin;

    bool m_inSpiTransaction {false};

public:
    bool initialize()
	{
        pinMode(ChipSelect_pin, OUTPUT);
        pinMode(IrqPin, INPUT_PULLUP);
        pinMode(ShutdownPin, OUTPUT);

        digitalWrite(ChipSelect_pin, HIGH);

        SPI.begin();
        //putInShutdown();
        return true;
    }
    void deinitialize()
	{
		pinMode(ChipSelect_pin, INPUT);
        pinMode(IrqPin, INPUT);
        pinMode(ShutdownPin, INPUT);
		SPI.end();
	}


	bool isIrqAsserted() const {
        return (digitalRead(IrqPin) == LOW);
    }
	void waitIrq();
	bool waitIrqFor(unsigned long timeout_ms);

	void putInShutdown() {
        digitalWrite(ShutdownPin, HIGH);
    }
	void releaseFromShutdown() {
        digitalWrite(ShutdownPin, LOW);
    }

	bool waitForClearToSend(uint8_t readCommandId, uint16_t timeout_ms)
	{
		unsigned long const t { millis() };
		uint8_t ctsVal {0};
		do {
			restartOrBeginSpiTransaction();
			spiWriteByte(readCommandId);
			ctsVal = spiReadByte();

			if ((millis()-t) > timeout_ms) {
				endSpiTransaction();
				return false;
			}

			//delayMicroseconds(100); // MAYBE: needed?
		} while (ctsVal != 0xFF); // Clear to send when 0xFF

		// Hold SPI transaction
		return true;
	}

	void beginSpiTransaction()
	{
        m_inSpiTransaction = true;
        digitalWrite(ChipSelect_pin, LOW);
        SPI.beginTransaction(ZETARF_SPI_SETTINGS);
    }
	void resumeOrBeginSpiTransaction()
	{
        if (m_inSpiTransaction)
            return;
        beginSpiTransaction();
    }
	void restartOrBeginSpiTransaction()
	{
        if (m_inSpiTransaction) {
            endSpiTransaction();
            delayMicroseconds(100);
        }
        beginSpiTransaction();
    }
	void endSpiTransaction()
	{
        SPI.endTransaction();
        delayMicroseconds(1);
        digitalWrite(ChipSelect_pin, HIGH);
        m_inSpiTransaction = false;
    }

	uint8_t spiReadWriteByte(uint8_t value) {
        return SPI.transfer(value);
    }

	void spiWriteByte(uint8_t value) {
        spiReadWriteByte(value);
    }
	uint8_t spiReadByte() {
        return spiReadWriteByte(0xFF);
    }

	void spiReadWriteData(uint8_t* data, uint8_t count) {
        SPI.transfer(data, count);
    }
	void spiWriteData(uint8_t const* data, uint8_t count) {
        while (count--)
            SPI.transfer(*data++);
    }
	void spiReadData(uint8_t* data, uint8_t count) {
        while (count--)
            *data++ = SPI.transfer(0xFF);
    }
};


// HAL with GPIO 1 as CTS output
// WARNING: the radio config needs to support it!
template <class TChipSelectPin, class TShutdownPin, class TIrqPin, class TClearToSendPin>
class ArduinoSpiHal_Gpio1AsClearToSend : ArduinoSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>
{
	static_assert(std::is_base_of<ClearToSendPinSelector, TClearToSendPin>::value, "Fourth parameter of ZetaRF must be the Clear To Send pin. Use ClearToSendPin<11> to use pin 11.");

    static constexpr int ClearToSend_pin = TClearToSendPin::pin;

public:
    bool initialize() {
        pinMode(ClearToSend_pin, INPUT_PULLUP);
        // TODO: add GPIO config?
        return ArduinoSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>::initialize();;
    }
    bool deinitialize() {
        pinMode(ClearToSend_pin, INPUT);
        return ArduinoSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>::deinitialize();;
    }

    bool waitForClearToSend(uint8_t readCommandId, uint16_t timeout_ms)
	{
		(void)readCommandId;
		unsigned long const t { millis() };
		while (digitalRead(ClearToSend_pin) == LOW) {
			// Wait with timeout...
			if ((millis()-t) > timeout_ms) {
				return false;
			}
			delayMicroseconds(100);
		}
		return true;
    }
};

} // namespace ZetaRFHal
