/*
 * SPI HAL for Arduino boards
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


namespace ZetaRfHal {

namespace arduino {

//!! Timeout utility class
struct Timeout
{
    unsigned long const timeout;
    unsigned long timeoutValue;

    Timeout(unsigned long const timeout) noexcept :
        timeout(timeout), timeoutValue(millis())
    {}

    void restart() noexcept {
        timeoutValue = millis();
    }
    bool expired() const noexcept {
        return (millis() - timeoutValue) >= timeout;
    }


    static Timeout make(uint32_t timeout_ms) noexcept {
        return Timeout(timeout_ms);
    }
};

} // namespace arduino


template <class TChipSelectPin, class TShutdownPin, class TIrqPin>
class ArduinoSpiHal
{
    static_assert(std::is_base_of<ChipSelectPinSelector, TChipSelectPin>::value, "First parameter of ZetaRF must be the Chip Select pin. Use ChipSelectPin<10> to use pin 10.");
    static_assert(std::is_base_of<ShutdownPinSelector, TShutdownPin>::value, "Second parameter of ZetaRF must be the Shutdown pin. Use ShutdownPin<9> to use pin 9.");
    static_assert(std::is_base_of<IrqPinSelector, TIrqPin>::value, "Third parameter of ZetaRF must be the IRQ pin. Use IrqPin<8> to use pin 8.");

    static constexpr int ChipSelect_pin = TChipSelectPin::pin;
    static constexpr int ShutdownPin = TShutdownPin::pin;
    static constexpr int IrqPin = TIrqPin::pin;

protected:
    bool m_inSpiTransaction {false};

public:
    using Timeout = arduino::Timeout;

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
        auto const t = Timeout::make(timeout_ms);
        uint8_t ctsVal {0};
        do {
            restartOrBeginSpiTransaction();
            spiWriteByte(readCommandId);
            ctsVal = spiReadByte();

            if (t.expired()) {
                endSpiTransaction();
                return false;
            }
        } while (ctsVal != 0xFF); // Clear to send when 0xFF

        // Hold SPI transaction
        return true;
    }

    void beginSpiTransaction()
    {
        m_inSpiTransaction = true;
        digitalWrite(ChipSelect_pin, LOW);
        //delayMicroseconds(1);
        SPI.beginTransaction(ZETARF_SPI_SETTINGS);
    }
    void resumeOrBeginSpiTransaction(uint8_t readCommandId)
    {
        (void)readCommandId; // Needed for hardware CTS
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


// HAL with GPIO 1 as CTS output (inverted, low when CTS)
// WARNING: the radio config needs to support it!
// WARNING: Not yet compatible with Si4455 (Zeta modules), ezradio_si4455.hpp need added support.
template <class TChipSelectPin, class TShutdownPin, class TIrqPin, class TClearToSendPin>
class ArduinoSpiHal_Gpio1AsClearToSend : ArduinoSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>
{
    using Base = ArduinoSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>;

    static_assert(std::is_base_of<ClearToSendPinSelector, TClearToSendPin>::value, "Fourth parameter of ZetaRF must be the Clear To Send pin. Use ClearToSendPin<11> to use pin 11.");

    static constexpr int ClearToSend_pin = TClearToSendPin::pin;

public:
    using Timeout = arduino::Timeout;

    bool initialize() {
        pinMode(ClearToSend_pin, INPUT_PULLUP);
        // TODO: add GPIO config?
        return Base::initialize();
    }
    bool deinitialize() {
        pinMode(ClearToSend_pin, INPUT);
        return Base::deinitialize();
    }

    bool waitForClearToSend(uint8_t readCommandId, uint16_t timeout_ms)
    {
        // CTS line goes low when clear to send.
        (void)readCommandId; // Not needed when using CTS on GPIO

        if (digitalRead(ClearToSend_pin) == LOW) // Fast check to avoid timeout
            return true;

        auto const t = Timeout::make(timeout_ms);

        // Polling GPIO for CTS
        while (digitalRead(ClearToSend_pin) == HIGH) {
            // Wait with timeout...
            if (t.expired())
                return false;
            delayMicroseconds(100);
        }
        return true;
    }

    void resumeOrBeginSpiTransaction(uint8_t readCommandId)
    {
        if (this->m_inSpiTransaction)
            return;
        this->beginSpiTransaction();

        // Only when using hardware CTS, we need to send the "read command buffer" command
        // and clear the first byte (the CTS value). When polling the CTS via SPI,
        // this is done in waitForClearToSend(...).
        this->spiWriteByte(readCommandId);
        this->spiReadByte();
    }
};

} // namespace ZetaRfHal
