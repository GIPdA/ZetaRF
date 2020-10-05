/*! @file ZetaRf.hpp
 *
 * @brief ZetaRf library main file
 * @version 2.0
 *
 * License: see LICENSE file
 */

/* Si4455 Known Issues - Rev B1B? (ROMID=3) - No errata sheet found for this rev.

Chip lock-up:
START_RX command with return state on timeout to RX may leave the chip unresponding (CTS not clearing). Return to NOCHANGE to fix.

*/

#pragma once

//#define ZETARF_DEBUG_ON

#if defined(ZETARF_DEBUG_ON)
    #if defined(WIRINGPI)
        #define debug(...)   std::cout << __VA_ARGS__
        #define debugln(...) std::cout << __VA_ARGS__ << std::endl
    #else
        #define debug(...)   Serial.print(__VA_ARGS__)
        #define debugln(...) Serial.println(__VA_ARGS__)
    #endif
#else
    #define debug(...) {}
    #define debugln(...) {}
#endif


#include "zetarf_hal.hpp"

#if defined(WIRINGPI)
    #include "zetarf_rpi_spi_hal.hpp"
#else
    #include "zetarf_arduino_spi_hal.hpp"
#endif

#include "zetarf_radio.hpp"
#include "ezradio_si4455.hpp"
#include "ezradiopro_si446x.hpp"

// Include other configs here
#include "configs/config868_fixedlength_crc_preamble10_sync4_payload8.hpp"
#include "configs/config433_fixedlength_crc_preamble10_sync4_payload8.hpp"

#include "configs/config868_variablelength_crc_preamble10_sync4_payload8.hpp"
#include "configs/config433_variablelength_crc_preamble10_sync4_payload8.hpp"

#include "configs/config433_4463_fixedlength_crc_preamble10_sync4_payload8.hpp"
#include "configs/config868_4463_fixedlength_crc_preamble10_sync4_payload8.hpp"
//#include "configs/config868_4463_variablelength_crc_preamble10_sync4_payload8.hpp"
#include "configs/config868_4463_variablelength_50k_crc_whitening_longrange.hpp"
#include "configs/config868_4463_variablelength_10k_crc_whitening_longrange.hpp"


#include <stdint.h>

#ifdef __AVR__
#define CONSTEXPR inline
#else
#define CONSTEXPR constexpr
#endif

namespace ZetaRf {

class ReadPacketResult {
public:
    enum Result
    {
        PacketSizeLargerThanBuffer = -5, // Variable length
        InvalidPacketSize = -4, // Variable length
        RequestFailed = -3,
        InvalidArgument = -2,
        NotEnoughDataInFifo = -1,
        Timeout = 0,
        Success
    };

    /* implicit */ ReadPacketResult(Result value, uint8_t packetSize = 0) noexcept : m_value(value), m_packetSize(packetSize) {}

    operator bool() const noexcept {
        return m_value == Result::Success;
    }
    operator Result() const noexcept {
        return m_value;
    }

    uint8_t receivedPacketSize() const noexcept {
        return m_packetSize;
    }

    Result value() const noexcept {
        return m_value;
    }

private:
    friend inline bool operator==(ReadPacketResult a, ReadPacketResult b) {
        return a.m_value == b.m_value;
    }
    friend inline bool operator==(ReadPacketResult a, Result b) {
        return a.m_value == b;
    }
    friend inline bool operator!=(ReadPacketResult a, ReadPacketResult b) {
        return !(a == b);
    }
    friend inline bool operator!=(ReadPacketResult a, Result b) {
        return !(a == b);
    }

    Result const m_value;
    uint8_t const m_packetSize;
};

} // namespace ZetaRf


template <typename EZRadio>
class ZetaRfBase
{
public:
    using RadioState = typename EZRadio::RadioState;
    using Event = ZetaRf::Event;
    using Events = ZetaRf::Events;
    using EZRadioReply = typename EZRadio::EZRadioReply;

    using PartInfo = typename EZRadioReply::PartInfo;
    using FuncInfo = typename EZRadioReply::FuncInfo;


    Events checkForEvent()
    {
        return checkForAnyEventOf(
              Event::CrcError //| Event::CommandError
            | Event::PacketTransmitted | Event::PacketReceived
            | Event::TxFifoAlmostEmpty | Event::RxFifoAlmostFull
            | Event::LatchedRssi
            | Event::FifoUnderflowOrOverflowError
            | Event::DeviceBusy
        );
    }

    Events checkForAnyEventOf(Events filter)
    {
        if (m_radio.isIrqAsserted()) {
            //Serial.println("irq");
            if (!readInterruptsToEvents())
                return Event::DeviceBusy;
        }

        if (!(m_events & filter))
            return Event::None;

        filter &= m_events; // Clear filter flags not in status
        m_events &= ~filter; // Clear status flags
        return filter;
    }

    Events checkForAllEventsOf(Events filter)
    {
        if (m_radio.isIrqAsserted()) {
            if (!readInterruptsToEvents())
                return Event::DeviceBusy;
        }

        if ((m_events & filter) != filter)
            return Event::None;

        m_events &= ~filter; // Clear status flags
        return filter;
    }

    Events const& events() const {
        return m_events;
    }
    void clearEvents(Events events) {
        m_events.clear(events);
    }
    void clearEvents() {
        m_events.clear();
    }

    bool hasDataAvailable() const {
        return m_dataAvailable;
    }
    bool hasDataAvailable() {
        return m_dataAvailable;
    }
    bool available() const {
        return hasDataAvailable();
    }
    bool available() {
        return hasDataAvailable();
    }

    //! Reset device and load up the config.
    bool beginWithConfigurationDataArray(uint8_t const* configArray)
    {
        initialize();

        // Power up
        hardwareReset();
        // Wait until reset timeout or Reset IT signal
        //delay(100);

        int retryCount = 2;
        // Load radio configuration and set FRRs
        while (!m_radio.loadConfigurationArray(configArray) && (retryCount--)) {
            // Reset and retry
            hardwareReset();
            // Wait until reset timeout or Reset IT signal
            delay(100);
        }

        if (retryCount <= 0) {
            debugln("Failed to initialize the radio module!");
            return false;
        }

        m_maxRxPacketLength = m_radio.readMaxRxPacketLength()&0xFF;
        m_radio.clearAllPendingInterrupts();

        _clearEvents();
        return m_radio.succeeded();
    }

    void end() {
        m_radio.deinitialize();
    }

    bool requestNop()
    {
        m_radio.noOperation();
        return m_radio.succeeded();
    }


    /*!
     * Send data to @a channel. Returns to previous radio state after TX complete.
     * Notes when using variable length packets:
     *  - First data byte must be the payload length field, and its value must be the data length (ignoring this extra byte).
     *  - e.g. To send 5 bytes of data, @a data buffer must contains: [5, data1, ... , data5] and @a length must be 6.
     * @sa sendVariableLengthPacket
     *
     * @param channel Channel to send data to.
     * @param data Pointer to data to send, first byte should be the payload length.
     * @param dataSize Data length (bytes). Includes payload length byte when using variable length packets. Fill with zeros if < packet length.
     * @param timeout_ms Max delay in ms to wait for the device to be ready to send a packet.
     */
    bool sendFixedLengthPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeout_ms)
    {
        // TODO: handle retransmit feature
        if (!data || dataSize == 0)
            return false;

        if (!waitUntilOutOfTx(timeout_ms))
            return false;

        // TxComplete value should not be 7 (TX state) as waitReadyToSendPacket checks for that.
        RadioState const txCompleteState = radioState();

        // Fill the TX fifo with data
        using namespace std; // In case min() is not a f* define as with Arduino
        m_radio.writeTxFifo(data, min(m_packetLength, dataSize));

        // Fill remaining with data
        if (m_packetLength > dataSize) {
            m_radio.writeTxFifoWithZeros(m_packetLength-dataSize);
        }

        // Start sending packet on channel, return to current state after transmit
        m_radio.startTx(channel,
                        m_packetLength,
                        txCompleteState);

        return m_radio.succeeded();
    }

    /*!
     * Convenience method to send variable length packets without the need to handle the extra payload field.
     * Do not include any extra payload byte or account for it in @a length, it is handled internally.
     *
     * @param channel Channel to send data to.
     * @param data Pointer to data to send, first byte should not be the payload length.
     * @param length Data length (bytes). Do not include the payload length byte.
     * @param timeout_ms Max delay in ms to wait for the device to be ready to send a packet.
     */
    bool sendVariableLengthPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataByteCount, unsigned long timeout_ms)
    {
        if (!data || dataByteCount == 0)
            return false;

        if (!waitUntilOutOfTx(timeout_ms))
            return false;

        // TxComplete value should not be 7 (TX state) as waitReadyToSendPacket checks for that.
        RadioState const txCompleteState = radioState();

        // Write the varible length field
        m_radio.writeTxFifo(&dataByteCount, 1);
        // Fill the TX fifo with data
        m_radio.writeTxFifo(data, dataByteCount);

        // Start sending packet on channel, return to current state after transmit
        m_radio.startVariableLengthTx(channel,
                                      dataByteCount,
                                      txCompleteState);

        return m_radio.succeeded();
    }

    //! Checks if there is enough space left in TX fifo for one packet
    /*bool canSendOnePacket() {
        //
    }//*/

    //! Read packet from Rx FIFO, if any.
    ZetaRf::ReadPacketResult readFixedLengthPacketTo(uint8_t* data, uint8_t dataByteCount)
    {
        if (!data)
            return ZetaRf::ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        auto const& fi = m_radio.readFifoInfo();
        if (m_radio.failed())
            return ZetaRf::ReadPacketResult::RequestFailed;

        return readPacket(fi, data, dataByteCount);
    }

    //! Read variable length packet from Rx FIFO, if any.
    //! packetDataLength is set to the received packet length, even in case of packet size error.
    ZetaRf::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxByteCount, uint8_t* packetDataLength)
    {
        if (!data)
            return ZetaRf::ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        auto fi = m_radio.readFifoInfo(); // Make a copy
        if (m_radio.failed())
            return ZetaRf::ReadPacketResult::RequestFailed;

        if (fi.RX_FIFO_COUNT < 1) {
            debugln("Read VL Packet: Not Enough Data In Fifo");
            return ZetaRf::ReadPacketResult::NotEnoughDataInFifo;
        }

        // Read size
        fi.RX_FIFO_COUNT--; // Remove variable length field
        uint8_t length {0};
        m_radio.readRxFifo(&length, 1);

        if (packetDataLength)
            *packetDataLength = length;

        //debug("Read VL Packet of size: ");
        //debugln(packetDataLength);

        if (length <= 1 || length > 63) {
            m_radio.resetRxFifo(); // MAYBE: do if RX auto-recovery option active
            if (m_radio.succeeded())
                m_dataAvailable = false;
            return {ZetaRf::ReadPacketResult::InvalidPacketSize, length};
        }

        if (length > maxByteCount)
            return {ZetaRf::ReadPacketResult::PacketSizeLargerThanBuffer, length};

        return readPacket(fi, data, length);
    }


    //! RSSI value of the last received packet (reset upon entering RX).
    //! Valid only when using single packet listening mode, if you read it before restarting RX.
    uint8_t latchedRssiValue() {
        return m_radio.readFrrB().FRR_B_VALUE;
    }

    //! Packet size in fixed length mode, must be zero in variable length mode
    uint8_t packetLength() const
    {
        return m_packetLength;
    }
    void setPacketLength(uint8_t newLength)
    {
        m_packetLength = newLength;
    }

    //! Max packet length accepted by the RX modem.
    uint8_t maxRxPacketLength() const
    {
        return m_maxRxPacketLength;
    }
    void setMaxRxPacketLength(uint8_t newLength)
    {
        m_radio.setMaxRxPacketLength(newLength);
        m_maxRxPacketLength = newLength;
    }

    uint8_t listeningChannel() const
    {
        return m_listeningChannel;
    }

    uint8_t requestCurrentChannel()
    {
        auto const& ds {m_radio.readDeviceState()};
        return m_radio.succeeded() ? ds.CURRENT_CHANNEL : 0;
    }


    bool startListeningOnChannel(uint8_t newChannel)
    {
        if (!waitUntilOutOfTx(20))
            return false;

        if (!startListening(newChannel, m_packetLength))
            return false;

        m_listeningChannel = requestCurrentChannel();
        return m_radio.succeeded() && (m_listeningChannel == newChannel);
    }

    bool startListeningSinglePacketOnChannel(uint8_t newChannel)
    {
        if (!waitUntilOutOfTx(20))
            return false;

        if (!startListeningSinglePacket(newChannel, m_packetLength))
            return false;

        m_listeningChannel = requestCurrentChannel();
        return m_radio.succeeded() && (m_listeningChannel == newChannel);
    }

    bool restartListeningSinglePacket()
    {
        if (!waitUntilOutOfTx(20))
            return false;

        return startListeningSinglePacket(m_listeningChannel, m_packetLength);
    }
    bool restartListening()
    {
        if (!waitUntilOutOfTx(20))
            return false;

        return startListening(m_listeningChannel, m_packetLength);
    }


    //! Space left in TX FIFO
    uint8_t requestBytesAvailableInTxFifo()
    {
        auto const& fi = m_radio.readFifoInfo();
        return m_radio.succeeded() ? fi.TX_FIFO_SPACE : 0;
    }

    //! Bytes stored in RX FIFO
    uint8_t requestBytesAvailableInRxFifo()
    {
        auto const& fi = m_radio.readFifoInfo();
        return m_radio.succeeded() ? fi.RX_FIFO_COUNT : 0;
    }

    //! Manually reset the RX Fifo and loose any pending packet.
    bool requestResetRxFifo()
    {
        m_radio.resetRxFifo();
        m_dataAvailable = false;
        return m_radio.succeeded();
    }

    bool requestResetTxFifo()
    {
        m_radio.resetTxFifo();
        return m_radio.succeeded();
    }

    bool requestResetRxAndTxFifo()
    {
        m_radio.resetRxAndTxFifo();
        m_dataAvailable = false;
        return m_radio.succeeded();
    }

    // Checks if the current state of the module seems correct.
    bool isAlive()
    {
        auto const& cs = m_radio.readDeviceState();
        return m_radio.succeeded() && (cs.CURR_STATE != 0) && (cs.CURR_STATE != 0xFF);// && cs.CURRENT_CHANNEL == m_listeningChannel);
    }

    //! Returns the current RSSI reading from the modem.
    uint8_t requestCurrentRssi()
    {
        // Clear RSSI_LATCH_PEND and RSSI_PEND status bits
        uint8_t const rssi { m_radio.readModemStatus(0x27).CURR_RSSI };
        return m_radio.succeeded() ? rssi : 0;
    }

    //! Current internal radio state via FRR
    RadioState radioState()
    {
        return static_cast<RadioState>(m_radio.readFrrA().FRR_A_VALUE & 0x0F);
    }

    uint8_t requestDeviceState()
    {
        auto const& ds {m_radio.readDeviceState()};
        return m_radio.succeeded() ? ds.CURR_STATE : 0;
    }

    /*static CONSTEXPR const char* radioStateText(RadioState state) {
        switch (state) {
            case RadioState::Sleep: return "Sleep";
            case RadioState::SpiActive: return "SpiActive";
            case RadioState::Ready: return "Ready";
            case RadioState::Ready2: return "Ready2";
            case RadioState::TxTune: return "TxTune";
            case RadioState::RxTune: return "RxTune";
            case RadioState::Tx: return "Tx";
            case RadioState::Rx: return "Rx";

            default: return "Invalid";
        }
    }//*/

    //! Hardware reset the chip using the shutdown pin
    void hardwareReset()
    {
        // Put radio in shutdown, wait then release
        m_radio.holdInReset();
        //delayMicroseconds(20);
        delay(1);
        _clearEvents();
        m_radio.releaseFromReset();
        delay(5);
    }

    typename EZRadioReply::PartInfo const& readPartInformation() {
        return m_radio.readPartInformation();
    }

    typename EZRadioReply::FuncInfo const& readFunctionRevisionInformation() {
        return m_radio.readFunctionRevisionInformation();
    }

private:
    bool initialize()
    {
        if (!m_radio.initialize()) {
            debugln("Failed to initialize the radio module!");
            return false;
        }
        return true;
    }

    bool waitUntilOutOfTx(unsigned long timeout_ms)
    {
        if (!m_radio.waitForClearToSend(timeout_ms)) {
            debugln("Wait CTS failed");
            return false;
        }

        // Wait for the device to be ready to send a packet

        auto const t = EZRadio::Timeout::make(timeout_ms);

        RadioState s { radioState() };
        // Goes out of Tx/TxTune when packet is sent
        while ((s == RadioState::Tx) || (s == RadioState::TxTune) || (s == RadioState::Invalid)) {
            if (t.expired() || s >= RadioState::Unknown) {
                //debug("Timeout! "); debugln(radioStateText(s));
                return false;
            }

            //delayMicroseconds(100);

            s = radioState();
        }

        return true;//m_radio.succeeded();
    }

    //! Read a packet from RX FIFO, with size checks
    ZetaRf::ReadPacketResult readPacket(typename EZRadioReply::FifoInfo const& fifoInfo, uint8_t* data, uint8_t bytesToRead)
    {
        bool const dataRemaining { (fifoInfo.RX_FIFO_COUNT > bytesToRead) };

        debug("Read Packet: ");
        debug(bytesToRead); debug('/'); debugln(fifoInfo.RX_FIFO_COUNT);

        if (bytesToRead > fifoInfo.RX_FIFO_COUNT) {
            if (bytesToRead == m_packetLength) {
                // We shouldn't have not enough data when using the packet length.
                // Reset RX FIFO to avoid looping and stale data.
                m_radio.resetRxFifo();
                m_dataAvailable = false;
            }
            debugln("Read Packet: Not Enough Data In Fifo");
            return {ZetaRf::ReadPacketResult::NotEnoughDataInFifo, fifoInfo.RX_FIFO_COUNT};
        }

        m_radio.readRxFifo(data, bytesToRead);

        m_dataAvailable = dataRemaining;

        if (m_radio.succeeded())
            return {ZetaRf::ReadPacketResult::Success, bytesToRead};

        return {ZetaRf::ReadPacketResult::RequestFailed, bytesToRead};
    }


    //! Ask to start listening on given channel with given packet length.
    //! Auto-returns to RX mode after receiving a valid packet.
    //! For variable length packet configuration, set packetLength to zero.
    bool startListening(uint8_t channel, uint8_t packetLength)
    {
        //debug("Listening on channel ");
        //debug(channel);
        //debug(" with packet size ");
        //debugln(packetLength);

        m_radio.startRx(channel, packetLength);
        return m_radio.succeeded();
    }

    //! Ask to start listening on given channel with given packet length.
    //! Goes to Ready mode after receiving a valid packet. Call startListening again to listen for new packets.
    //! For variable length packet configuration, set packetLength to zero.
    bool startListeningSinglePacket(uint8_t channel, uint8_t packetLength)
    {
        //debug("Listening single packet on channel ");
        //debug(channel);
        //debug(" with packet size ");
        //debugln(packetLength);

        m_radio.startRxForSinglePacket(channel, packetLength);
        return m_radio.succeeded();
    }


    //! Process and clear pending interrupts
    bool readInterruptsToEvents()
    {
        Events const ev { m_radio.readInterrupts() };
        if (m_radio.failed()) //it & EZRadio::Interrupt::DeviceBusy)
            return false;

#if defined(ZETARF_DEBUG_ON)
        debugln("Events:");
        printEvents(ev);
#endif

        m_events &= ~Event::CommandError;
        m_events |= ev;

        if (ev & Event::PacketReceived) {
            m_dataAvailable = true;
        }

        if (ev & (Event::CrcError | Event::FifoUnderflowOrOverflowError)) {
            m_radio.resetRxFifo(); // MAYBE: leave that choice to the user? Add an option for RX auto-recovery
            m_dataAvailable = false;
            m_events &= ~Event::PacketReceived;
        }

        return true;
    }

    void printEvents(Events const& events)
    {
        if (events & Event::DeviceBusy) debugln("ERROR: Device Busy");

        if (events & Event::PacketTransmitted) debugln("Packet IT: Packet Transmitted");
        if (events & Event::PacketReceived) debugln("Packet IT: Packet Received");
        if (events & Event::CrcError) debugln("Packet IT: CRC Error");
        if (events & Event::TxFifoAlmostEmpty) debugln("Packet IT: TX FIFO Almost Empty");
        if (events & Event::RxFifoAlmostFull) debugln("Packet IT: RX FIFO Almost Full");

        if (events & Event::FilterMatch) debugln("Packet IT: FilterMatch");
        if (events & Event::FilterMiss) debugln("Packet IT: FilterMiss");
        if (events & Event::AlternateCrcError) debugln("Packet IT: AlternateCrcError");

         if (events & Event::InvalidSync) debugln("Modem IT: Invalid Sync");
         if (events & Event::InvalidPreamble) debugln("Modem IT: Invalid Preamble");
         if (events & Event::DetectedPreamble) debugln("Modem IT: Detected Preamble");
         if (events & Event::DetectedSync) debugln("Modem IT: Detected Sync");
        if (events & Event::LatchedRssi) debugln("Modem IT: RSSI Latched");

        if (events & Event::DetectedPostamble) debugln("Modem IT: DetectedPostamble");
        if (events & Event::RssiJump) debugln("Modem IT: RssiJump");
        if (events & Event::RssiThreshold) debugln("Modem IT: RssiThreshold");

        if (events & Event::FifoUnderflowOrOverflowError) debugln("Chip IT: FIFO Underflow/Overflow");
        if (events & Event::CommandError) debugln("Chip IT: Command Error");
         if (events & Event::StateChange) debugln("Chip IT: State Change");
        if (events & Event::ChipReady) debugln("Chip IT: Chip Ready");

        if (events & Event::Calibration) debugln("Chip IT: Calibration");
        if (events & Event::LowBattery) debugln("Chip IT: LowBattery");
        if (events & Event::WakeUpTimerExpired) debugln("Chip IT: WakeUpTimerExpired");
    }

    inline void _clearEvents() {
        m_events = Event::None;
        m_dataAvailable = false;
    }

protected:
    EZRadio m_radio;

    Events m_events;
    uint8_t m_listeningChannel {0};
    //uint8_t m_transmittingChannel {0};
    uint8_t m_packetLength {0};
    uint8_t m_maxRxPacketLength {0};
    bool m_dataAvailable {false};
};


//! Base class to use one of the provided radio configurations.
//! Implements common methods for both fixed and variable length packets.
template <typename Config, typename EZRadio>
class ZetaRfConfig : public ZetaRfBase<EZRadio>
{
    using Base = ZetaRfBase<EZRadio>;
public:
    using RadioState = typename EZRadio::RadioState;
    using Event = typename Base::Event;
    using Events = typename Base::Events;

    //!! Load radio config with default packet length
    bool begin()
    {
        if (!Base::beginWithConfigurationDataArray(Config::RadioConfigurationDataArray))
            return false;

        if (Config::VariableLengthPacketConfiguration)
            Base::setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            Base::setPacketLength(Config::PacketLength);

        Base::m_maxRxPacketLength = Config::PacketLength;
        return true;
    }

    //! Load radio config with the specified packet length.
    //! In variable length mode, sets the max length for RX packets (only valid for Si446x).
    bool beginWithPacketLengthOf(uint8_t packetLength)
    {
        if (!Base::beginWithConfigurationDataArray(Config::RadioConfigurationDataArray))
            return false;

        if (Config::VariableLengthPacketConfiguration)
            Base::setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            Base::setPacketLength(packetLength);

        // If the packet length is the default length, avoid the transaction
        if (Config::PacketLength != packetLength)
            Base::setMaxRxPacketLength(packetLength);
        else
            Base::m_maxRxPacketLength = packetLength;

        return Base::m_radio.succeeded(); // setMaxRxPacketLength may fail
    }

    //! Load a variable length radio config with the specified max packet length.
    //! Convenience method for variable length mode, it just calls beginWithPacketLengthOf(packetLength).
    bool beginWithMaxPacketLengthOf(uint8_t packetLength)
    {
        return beginWithPacketLengthOf(packetLength);
    }


    // ### PACKET SENDING METHODS ###

    //! Send either fixed or variable length packet depending on radio config.
    //! For fixed length packet mode, if @a dataSize is less than the packet size, zeros are automatically appended.
    bool sendPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        if (Config::VariableLengthPacketConfiguration)
            return Base::sendVariableLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
        else
            return Base::sendFixedLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
    }

    //! Send fixed length packet, config length or user-set length is used. RX must be waiting for that length of packet.
    //! @a data must point to at least 'packet length' bytes.
    bool sendFixedLengthPacketOnChannel(uint8_t channel, uint8_t const* data, unsigned long timeoutForReady_ms = 100) {
        return Base::sendFixedLengthPacketOnChannel(channel, data, Base::m_packetLength, timeoutForReady_ms);
    }


    //! Send variable length packet. Data is put in FIFO and set to send.
    bool sendVariableLengthPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        //static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return Base::sendVariableLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
    }


    // ### PACKET RECEIVING METHODS ###

    //! Read a packet with size set via beginWithPacketLengthOf().
    //! Data ptr must point to a buffer large enough (at least packetLength() or maxRxPacketLength()).
    ZetaRf::ReadPacketResult readPacketTo(uint8_t* data)
    {
        if (Config::VariableLengthPacketConfiguration)
            return Base::readVariableLengthPacketTo(data, Base::m_maxRxPacketLength, nullptr);
        else
            return Base::readFixedLengthPacketTo(data, Base::m_packetLength);
    }

    ZetaRf::ReadPacketResult readFixedLengthPacketTo(uint8_t* data, uint8_t packetLength)
    {
        return Base::readFixedLengthPacketTo(data, packetLength);
    }

    //! Read a variable length packet, max data size set by setMaxRxPacketLength() or beginWithMaxPacketLengthOf().
    ZetaRf::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t* rxPacketDataLength)
    {
        return Base::readVariableLengthPacketTo(data, Base::m_maxRxPacketLength, rxPacketDataLength);
    }

    //! Read a variable length packet with at most 'maxDataSize' bytes.
    ZetaRf::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxDataSize, uint8_t* rxPacketDataLength)
    {
        return Base::readVariableLengthPacketTo(data, maxDataSize, rxPacketDataLength);
    }

    //! Packet length defined in the radio config file.
    constexpr uint8_t defaultPacketLength() const
    {
        return Config::PacketLength;
    }
};


// Remove debug macros
#undef debug
#undef debugln


// ########################################
// #### Supported Radio configurations ####
// ########################################

// NOTE: Hardware CTS (ZetaRfHal::ArduinoSpiHal_Gpio1AsClearToSend) is not yet compatible with Si4455 (Zeta modules), ezradio_si4455.hpp need added support.

#if defined(WIRINGPI)
    template<class ...Ts>
    using SpiHal = ZetaRfHal::RPiSpiHal<Ts...>;
#else
    template<class ...Ts>
    using SpiHal = ZetaRfHal::ArduinoSpiHal<Ts...>;
#endif

// Default Zeta configs
template<class ...Ts>
using ZetaRf868 = ZetaRfConfig<ZetaRfConfigs::Config868_FixedLength_CRC_Preamble10_Sync4_Payload8,
                               ZetaRfEZRadio::EZRadioSi4455<SpiHal<Ts...>> >;

template<class ...Ts>
using ZetaRf433 = ZetaRfConfig<ZetaRfConfigs::Config433_FixedLength_CRC_Preamble10_Sync4_Payload8,
                               ZetaRfEZRadio::EZRadioSi4455<SpiHal<Ts...>> >;


// Variable Length
template<class ...Ts>
using ZetaRf868_VL = ZetaRfConfig<ZetaRfConfigs::Config868_VariableLength_CRC_Preamble10_Sync4_Payload8,
                                  ZetaRfEZRadio::EZRadioSi4455<SpiHal<Ts...>> >;

template<class ...Ts>
using ZetaRf433_VL = ZetaRfConfig<ZetaRfConfigs::Config433_VariableLength_CRC_Preamble10_Sync4_Payload8,
                                  ZetaRfEZRadio::EZRadioSi4455<SpiHal<Ts...>> >;



// DRF4463F configs
template<class ...Ts>
using ZetaRf_DRF4463F_868 = ZetaRfConfig<ZetaRfConfigs::Config868_Si4463_FixedLength_CRC_Preamble10_Sync4_Payload8,
                                         ZetaRfEZRadioPro::EZRadioProSi446x<SpiHal<Ts...>> >;

template<class ...Ts>
using ZetaRf_DRF4463F_433 = ZetaRfConfig<ZetaRfConfigs::Config433_Si4463_FixedLength_CRC_Preamble10_Sync4_Payload8,
                                         ZetaRfEZRadioPro::EZRadioProSi446x<SpiHal<Ts...>> >;


/* FIXME, check config
template<class ...Ts>
using ZetaRf_DRF4463F_868_VL = ZetaRfConfig<ZetaRfConfigs::Config868_Si4463_VariableLength_CRC_Preamble10_Sync4_Payload8,
                                            ZetaRfEZRadioPro::EZRadioProSi446x<SpiHal<Ts...>> >;//*/


// Long range config, not compatible with others or Zeta modules.
template<class ...Ts>
using ZetaRf_DRF4463F_868_VL_LongRange = ZetaRfConfig<ZetaRfConfigs::Config868_Si4463_VariableLength_50kbps_CRC_Whitening_LongRange,
                                                      ZetaRfEZRadioPro::EZRadioProSi446x<SpiHal<Ts...>> >;

template<class ...Ts>
using ZetaRf_DRF4463F_868_VL_10kLongRange = ZetaRfConfig<ZetaRfConfigs::Config868_Si4463_VariableLength_10kbps_CRC_Whitening_LongRange,
                                                      ZetaRfEZRadioPro::EZRadioProSi446x<SpiHal<Ts...>> >;

