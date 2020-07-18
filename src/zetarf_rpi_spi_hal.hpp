/*
 * SPI HAL for RaspberryPi
 */

#pragma once

#include <algorithm>
#include <cstring>
#include <type_traits>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "zetarf_hal.hpp"

#define ZETARF_SPI_SETTINGS_CHANNEL 0
#define ZETARF_SPI_SETTINGS_FREQ 1000000UL

namespace ZetaRfHal {

namespace rpi {

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

} // namespace rpi


template <class TChipSelectPin, class TShutdownPin, class TIrqPin>
class RPiSpiHal
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
    using Timeout = rpi::Timeout;

    bool initialize()
    {
        wiringPiSetup();
        wiringPiSPISetup(ZETARF_SPI_SETTINGS_CHANNEL, ZETARF_SPI_SETTINGS_FREQ);

        pinMode(ChipSelect_pin, OUTPUT);
        pinMode(IrqPin, INPUT);
        pullUpDnControl(IrqPin, PUD_UP);
        pinMode(ShutdownPin, OUTPUT);

        digitalWrite(ChipSelect_pin, HIGH);
        //putInShutdown();
        return true;
    }
    void deinitialize()
    {
        pinMode(ChipSelect_pin, INPUT);
        pinMode(IrqPin, INPUT);
        pinMode(ShutdownPin, INPUT);
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
        delayMicroseconds(1);
        digitalWrite(ChipSelect_pin, HIGH);
        m_inSpiTransaction = false;
    }

    uint8_t spiReadWriteByte(uint8_t value) {
        wiringPiSPIDataRW(ZETARF_SPI_SETTINGS_CHANNEL, &value, 1);
        return value;
    }

    void spiWriteByte(uint8_t value) {
        spiReadWriteByte(value);
    }
    uint8_t spiReadByte() {
        return spiReadWriteByte(0xFF);
    }

    void spiReadWriteData(uint8_t* data, uint8_t count) {
        wiringPiSPIDataRW(ZETARF_SPI_SETTINGS_CHANNEL, data, count);
    }
    void spiWriteData(uint8_t const* data, uint8_t count) {
        // Make copy of data as wiringPi will overwrite
        uint8_t dataCopy[count];
        memcpy(dataCopy, data, sizeof(dataCopy));
        wiringPiSPIDataRW(ZETARF_SPI_SETTINGS_CHANNEL, dataCopy, count);
    }
    void spiReadData(uint8_t* data, uint8_t count) {
        std::fill_n(data, count, 0xFF);
        wiringPiSPIDataRW(ZETARF_SPI_SETTINGS_CHANNEL, data, count);
    }
};


// HAL with GPIO 1 as CTS output (inverted, low when CTS)
// WARNING: the radio config needs to support it!
template <class TChipSelectPin, class TShutdownPin, class TIrqPin, class TClearToSendPin>
class RPiSpiHal_Gpio1AsClearToSend : RPiSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>
{
    using Base = RPiSpiHal<TChipSelectPin, TShutdownPin, TIrqPin>;

    static_assert(std::is_base_of<ClearToSendPinSelector, TClearToSendPin>::value, "Fourth parameter of ZetaRF must be the Clear To Send pin. Use ClearToSendPin<11> to use pin 11.");

    static constexpr int ClearToSend_pin = TClearToSendPin::pin;

public:
    using Timeout = rpi::Timeout;

    bool initialize() {
        pinMode(ClearToSend_pin, INPUT);
        pullUpDnControl(ClearToSend_pin, PUD_UP);
        // TODO: add GPIO config?
        return Base::initialize();;
    }
    bool deinitialize() {
        pinMode(ClearToSend_pin, INPUT);
        return Base::deinitialize();;
    }

    bool waitForClearToSend(uint8_t readCommandId, uint16_t timeout_ms)
    {
        (void)readCommandId;

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
};

} // namespace ZetaRfHal
