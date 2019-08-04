/*!
 * @brief ZetaRF HAL file
 *
 * License: see LICENSE file
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

#define ZETARF_SPI_SETTINGS_DEFAULT SPISettings(1000000UL, MSBFIRST, SPI_MODE0)

#define ZETARF_SPI_SETTINGS         ZETARF_SPI_SETTINGS_DEFAULT


template<int PinNumber>
struct PinSelector
{
    static constexpr int Pin = PinNumber;
};

struct ChipSelectPinSelector
{
};
struct ShutdownPinSelector
{
};
struct IrqPinSelector
{
};

template<int PinNumber>
struct ChipSelectPin : public ChipSelectPinSelector, public PinSelector<PinNumber>
{
};
template<int PinNumber>
struct ShutdownPin : public ShutdownPinSelector, public PinSelector<PinNumber>
{
};
template<int PinNumber>
struct IrqPin : public IrqPinSelector, public PinSelector<PinNumber>
{
};


template <class TChipSelectPin, class TShutdownPin, class TIrqPin>
class ZetaRfHal
{
    static constexpr int ChipSelect_pin = TChipSelectPin::Pin;
    static constexpr int ShutdownPin = TChipSelectPin::Pin;
    static constexpr int IrqPin = TIrqPin::Pin;

    bool m_inSpiTransaction {false};

public:
    static constexpr bool HasHardwareClearToSend {false};

    ZetaRfHal() {
        // Check pins
        static_assert(std::is_base_of<ChipSelectPinSelector, TChipSelectPin>::value, "First parameter of ZetaRF must be the Chip Select pin. Use ChipSelectPin<10> to use pin 10.");
        static_assert(std::is_base_of<ShutdownPinSelector, TShutdownPin>::value, "Second parameter of ZetaRF must be the Shutdown pin. Use ShutdownPin<9> to use pin 9.");
        static_assert(std::is_base_of<IrqPinSelector, TIrqPin>::value, "Third parameter of ZetaRF must be the IRQ pin. Use IrqPin<8> to use pin 8.");
    }

    bool initialize() {
        pinMode(ChipSelect_pin, OUTPUT);
        pinMode(IrqPin, INPUT_PULLUP);
        pinMode(ShutdownPin, OUTPUT);
        return true;
    }

    /*bool isClearToSend() const
    {
        return true; // Not implemented here
    }//*/

    bool isIrqAsserted() const
    {
        return (digitalRead(IrqPin) == LOW);
    }

    void putInShutdown() 
    {
        // Put in shutdown
        digitalWrite(ShutdownPin, HIGH);
    }
    void releaseFromShutdown()
    {
        digitalWrite(ShutdownPin, LOW);
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
            delayMicroseconds(5);
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


    uint8_t spiReadWriteByte(uint8_t value)
    {
        return SPI.transfer(value);
    }

    void spiWriteByte(uint8_t value)
    {
        spiReadWriteByte(value);
    }
    uint8_t spiReadByte()
    {
        return spiReadWriteByte(0xFF);
    }


    void spiReadWriteData(uint8_t* data, uint8_t count)
    {
        SPI.transfer(data, count);
    }

    void spiWriteData(const uint8_t* data, uint8_t count)
    {
        while (count--)
            SPI.transfer(*data++);
    }
    void spiReadData(uint8_t* data, uint8_t count)
    {
        while (count--)
            *data++ = SPI.transfer(0xFF);
    }

};


// HAL with GPIO 1 as CTS output
struct ClearToSendPinSelector
{
};
template<int PinNumber>
struct ClearToSendPin : public ClearToSendPinSelector, public PinSelector<PinNumber>
{
};

template <class TChipSelectPin, class TShutdownPin, class TIrqPin, class TClearToSendPin>
class ZetaRfHal_Gpio1AsClearToSend : ZetaRfHal<TChipSelectPin, TShutdownPin, TIrqPin>
{
    static constexpr int ClearToSend_pin = TClearToSendPin::Pin;

public:
    static constexpr bool HasHardwareClearToSend = true;

    ZetaRfHal_Gpio1AsClearToSend() {
        // Check pins
        static_assert(std::is_base_of<ClearToSendPinSelector, TClearToSendPin>::value, "Fourth parameter of ZetaRF must be the Clear To Send pin. Use ClearToSendPin<11> to use pin 11.");
    }

    bool initialize() {
        pinMode(ClearToSend_pin, INPUT_PULLUP);
        return ZetaRfHal<TChipSelectPin, TShutdownPin, TIrqPin>::initialize();;
    }

    bool isClearToSend() const
    {
        return (digitalRead(ClearToSend_pin) != LOW);
    }
};

