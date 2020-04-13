/*! @file ZetaRF.h
 *
 * @brief ZetaRF library main file
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
    #define debug(...)   Serial.print(__VA_ARGS__)
    #define debugln(...) Serial.println(__VA_ARGS__)
#else
    #define debug(...)
    #define debugln(...)
#endif


#include "zetarf_hal.hpp"
#include "zetarf_arduino_spi_hal.h"

#include "zetarf_radio.hpp"
#include "ezradio_si4455.h"

// Include other configs here
#include "configs/config868_fixedlength_crc_preamble10_sync4_payload8.h"
#include "configs/config433_fixedlength_crc_preamble10_sync4_payload8.h"

#include "configs/config868_variablelength_crc_preamble10_sync4_payload8.h"
#include "configs/config433_variablelength_crc_preamble10_sync4_payload8.h"

#include <stdint.h>

#ifdef __AVR__
#define CONSTEXPR inline
#else
#define CONSTEXPR constexpr
#endif

namespace ZetaRF {

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

    /* implicit */ ReadPacketResult(Result value) : m_value(value) {}

    operator bool() const {
        return m_value == Result::Success;
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

    Result m_value;
};

} // namespace ZetaRF


template <typename EZRadio>
class ZetaRFBase
{
public:
    using RadioState = typename EZRadio::RadioState;
    using Event = ZetaRF::Event;
    using Events = ZetaRF::Events;

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

    bool hasDataAvailble() const {
        return m_dataAvailable;
    }
    bool hasDataAvailble() {
        return m_dataAvailable;
    }
    bool available() const {
        return hasDataAvailble();
    }
    bool available() {
        return hasDataAvailble();
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
        // Load radio configuration
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

        m_radio.clearAllPendingInterrupts();

        // Configure FRR
        // ! DO NOT CHANGE ! FRRs are used by the library.
        m_radio.setProperties(0x02, // Group ID
                              0x00, // Start at index 0 (FRR A)
                              SI4455_PROP_FRR_CTL_A_MODE_FRR_A_MODE_ENUM_CURRENT_STATE,
                              SI4455_PROP_FRR_CTL_B_MODE_FRR_B_MODE_ENUM_LATCHED_RSSI,
                              SI4455_PROP_FRR_CTL_C_MODE_FRR_C_MODE_ENUM_INT_PH_PEND,
                              SI4455_PROP_FRR_CTL_D_MODE_FRR_D_MODE_ENUM_INT_CHIP_PEND);

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
                        static_cast<uint8_t>(txCompleteState),
                        false, // retransmit
                        m_packetLength);

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
        m_radio.startTx(channel,
                        static_cast<uint8_t>(txCompleteState),
                        false, // retransmit
                        dataByteCount+1);

        return m_radio.succeeded();
    }

    //! Checks if there is enough space left in TX fifo for one packet
    /*bool canSendOnePacket() {
        //
    }//*/

    //! Read packet from Rx FIFO, if any.
    ZetaRF::ReadPacketResult readFixedLengthPacketTo(uint8_t* data, uint8_t dataByteCount)
    {
        if (!data)
            return ZetaRF::ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        EZRadioReply::FifoInfo const& fi = m_radio.readFifoInfo();
        if (m_radio.failed())
            return ZetaRF::ReadPacketResult::RequestFailed;

        return readPacket(fi, data, dataByteCount);
    }

    //! Read variable length packet from Rx FIFO, if any.
    ZetaRF::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxByteCount, uint8_t& packetDataLength)
    {
        if (!data)
            return ZetaRF::ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        EZRadioReply::FifoInfo fi = m_radio.readFifoInfo(); // Make a copy
        if (m_radio.failed())
            return ZetaRF::ReadPacketResult::RequestFailed;

        if (fi.RX_FIFO_COUNT < 1) {
            debugln("Read VL Packet: Not Enough Data In Fifo");
            return ZetaRF::ReadPacketResult::NotEnoughDataInFifo;
        }

        // Read size
        fi.RX_FIFO_COUNT--; // Remove variable length field
        m_radio.readRxFifo(&packetDataLength, 1);

        //debug("Read VL Packet of size: ");
        //debugln(packetDataLength);

        if (packetDataLength <= 1) {
            m_radio.resetRxFifo(); // MAYBE: do if RX auto-recovery option active
            if (m_radio.succeeded())
                m_dataAvailable = false;
            return ZetaRF::ReadPacketResult::InvalidPacketSize;
        }

        if (packetDataLength > maxByteCount)
            return ZetaRF::ReadPacketResult::PacketSizeLargerThanBuffer;

        return readPacket(fi, data, packetDataLength);
    }


    //! RSSI value of the last received packet (reset upon entering RX).
    //! Valid only when using single packet listening mode, if you read it before restarting RX.
    uint8_t latchedRssiValue() {
        return m_radio.readFrrB().FRR_B_VALUE;
    }

    uint8_t packetLength() const
    {
        return m_packetLength;
    }
    void setPacketLength(uint8_t newLength)
    {
        m_packetLength = newLength;
    }

    uint8_t listeningChannel() const
    {
        return m_listeningChannel;
    }

    uint8_t requestCurrentChannel()
    {
        EZRadioReply::DeviceState const& ds {m_radio.readDeviceState()};
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

    bool restartListeningSinglePacket() {
        return startListeningSinglePacket(m_listeningChannel, m_packetLength);
    }

    // Space left in TX FIFO
    uint8_t requestBytesAvailableInTxFifo()
    {
        EZRadioReply::FifoInfo const& fi = m_radio.readFifoInfo();
        return m_radio.succeeded() ? fi.TX_FIFO_SPACE : 0;
    }

    // Bytes stored in RX FIFO
    uint8_t requestBytesAvailableInRxFifo()
    {
        EZRadioReply::FifoInfo const& fi = m_radio.readFifoInfo();
        return m_radio.succeeded() ? fi.RX_FIFO_COUNT : 0;
    }

    //! Manually reset the RX Fifo and loose any pending packet.
    bool requestResetRxFifo() {
        m_radio.resetRxFifo();
        m_dataAvailable = false;
        return m_radio.succeeded();
    }

    bool requestResetTxFifo() {
        m_radio.resetTxFifo();
        return m_radio.succeeded();
    }

    bool requestResetRxAndTxFifo() {
        m_radio.resetRxAndTxFifo();
        m_dataAvailable = false;
        return m_radio.succeeded();
    }

    // Checks if the current state of the module seems correct.
    bool isAlive()
    {
        EZRadioReply::DeviceState const& cs = m_radio.readDeviceState();
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
        delayMicroseconds(20);
        _clearEvents();
        m_radio.releaseFromReset();
        delay(5);
    }

    EZRadioReply::PartInfo const& readPartInformation() {
        return m_radio.readPartInformation();
    }

    EZRadioReply::FuncInfo const& readFunctionRevisionInformation() {
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
        unsigned long const t { millis() };
        RadioState s { radioState() };
        // Goes out of Tx/TxTune when packet is sent
        while ((s == RadioState::Tx) || (s == RadioState::TxTune) || (s == RadioState::Invalid)) {
            if ((millis()-t) > timeout_ms || s >= RadioState::Unknown) {
                //debug("Timeout! "); debugln(radioStateText(s));
                return false;
            }

            //delay(1);
            delayMicroseconds(100);

            s = radioState();
        }

        return true;//m_radio.succeeded();
    }

    //! Read a packet from RX FIFO, with size checks
    ZetaRF::ReadPacketResult readPacket(EZRadioReply::FifoInfo const& fifoInfo, uint8_t* data, uint8_t byteCount)
    {
        bool const dataRemaining { (fifoInfo.RX_FIFO_COUNT > byteCount) };

        debug("Read Packet: ");
        debug(byteCount); debug('/'); debugln(fifoInfo.RX_FIFO_COUNT);

        if (byteCount > fifoInfo.RX_FIFO_COUNT) {
            if (byteCount == m_packetLength) {
                // We shouldn't have not enough data when using the packet length.
                // Reset RX FIFO to avoid looping and stale data.
                m_radio.resetRxFifo();
                m_dataAvailable = false;
            }
            debugln("Read Packet: Not Enough Data In Fifo");
            return ZetaRF::ReadPacketResult::NotEnoughDataInFifo;
        }

        m_radio.readRxFifo(data, byteCount);

        m_dataAvailable = dataRemaining;

        return m_radio.succeeded() ? ZetaRF::ReadPacketResult::Success : ZetaRF::ReadPacketResult::RequestFailed;
    }


    //! Ask to start listening on given channel with given packet length.
    //! Auto-returns to RX mode after receiving a valid packet.
    //! For variable length packet configuration, set packetLegth to zero.
    bool startListening(uint8_t channel, uint8_t packetLength)
    {
        //debug("Listening on channel ");
        //debug(channel);
        //debug(" with packet size ");
        //debugln(packetLength);

        // Start Receiving packet on channel, START immediately, Packet n bytes long
        m_radio.startRx(channel, 0, packetLength,
                        SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                        SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_RX,
                        SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX);

        return m_radio.succeeded();
    }

    //! Ask to start listening on given channel with given packet length.
    //! Goes to Ready mode after receiving a valid packet. Call startListening again to listen for new packets.
    //! For variable length packet configuration, set packetLegth to zero.
    bool startListeningSinglePacket(uint8_t channel, uint8_t packetLength)
    {
        //debug("Listening single packet on channel ");
        //debug(channel);
        //debug(" with packet size ");
        //debugln(packetLength);

        // Start Receiving packet on channel, START immediately, Packet n bytes long
        m_radio.startRx(channel, 0, packetLength,
                        SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                        SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_READY,
                        SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX);

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

        //if (events & Event::InvalidSync) debugln("Modem IT: Invalid Sync");
        //if (events & Event::InvalidPreamble) debugln("Modem IT: Invalid Preamble");
        //if (events & Event::DetectedPreamble) debugln("Modem IT: Detected Preamble");
        //if (events & Event::DetectedSync) debugln("Modem IT: Detected Sync");
        if (events & Event::LatchedRssi) debugln("Modem IT: RSSI Latched");

        if (events & Event::FifoUnderflowOrOverflowError) debugln("Chip IT: FIFO Underflow/Overflow");
        if (events & Event::CommandError) debugln("Chip IT: Command Error");
        //if (events & Event::StateChange) debugln("Chip IT: State Change");
        if (events & Event::ChipReady) debugln("Chip IT: Chip Ready");
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
    bool m_dataAvailable {false};
};


template <typename Config, typename EZRadio>
class ZetaRFConfig : public ZetaRFBase<EZRadio>
{
    using Base = ZetaRFBase<EZRadio>;
public:
    using RadioState = typename EZRadio::RadioState;
    using Event = typename Base::Event;
    using Events = typename Base::Events;

    //!! Load radio config
    bool begin() {
        if (Config::VariableLengthPacketConfiguration)
            Base::setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            Base::setPacketLength(Config::PacketLength);

        return Base::beginWithConfigurationDataArray(Config::RadioConfigurationDataArray);
    }

    //! Load radio config with the specified packet length (no effect in variable length packet mode).
    bool beginWithPacketLengthOf(uint8_t packetLength) {
        if (Config::VariableLengthPacketConfiguration)
            Base::setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            Base::setPacketLength(packetLength);

        return Base::beginWithConfigurationDataArray(Config::RadioConfigurationDataArray);
    }


    // ### PACKET SENDING METHODS ###

    //! Send either fixed or variable length packet depending on radio config. Data is put in FIFO and set to send.
    //! For fixed length packet mode, if @a dataSize is less than the packet size, zeros are automatically appended.
    bool sendPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        if (Config::VariableLengthPacketConfiguration)
            return Base::sendVariableLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
        else
            return Base::sendFixedLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
    }

    //! Send fixed length packet, config length or user-set length is used. RX must be waiting for that length of packet. Data is put in FIFO and set to send.
    //! @a data must point to at least 'packet length' bytes.
    bool sendFixedLengthPacketOnChannel(uint8_t channel, uint8_t const* data, unsigned long timeoutForReady_ms = 100) {
        return Base::sendFixedLengthPacketOnChannel(channel, data, Base::m_packetLength, timeoutForReady_ms);
    }

    //! Send fixed length packet with specified length. RX must be waiting for that length of packet. Data is put in FIFO and set to send.
    //! If @a dataSize is less than the packet size, zeros are automatically appended.
    //bool sendFixedLengthPacket(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100)

    //! Send variable length packet. Data is put in FIFO and set to send.
    bool sendVariableLengthPacketOnChannel(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        //static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return Base::sendVariableLengthPacketOnChannel(channel, data, dataSize, timeoutForReady_ms);
    }


    // ### PACKET RECEIVING METHODS ###

    //! Read either fixed or variable length packet depending on radio config
    ZetaRF::ReadPacketResult readPacketTo(uint8_t* data, uint8_t byteCount) {
        if (Config::VariableLengthPacketConfiguration) {
            uint8_t packetDataLength {0};
            return readVariableLengthPacketTo(data, byteCount, packetDataLength);
        }
        else
            return Base::readFixedLengthPacketTo(data, byteCount);
    }

    //! Read a packet with size set via beginWithPacketLengthOf(). @a data must point to a buffer large enough!
    ZetaRF::ReadPacketResult readPacketTo(uint8_t* data) {
        return readPacketTo(data, Base::m_packetLength);
    }

    ZetaRF::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxByteCount, uint8_t& packetDataLength) {
        //static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return Base::readVariableLengthPacketTo(data, maxByteCount, packetDataLength);
    }

    constexpr uint8_t defaultPacketLength() const {
        return Config::PacketLength;
    }
};


// Remove debug macros
#undef debug
#undef debugln



// Default Arduino configs
template<class ...Ts>
using ZetaRF868 = ZetaRFConfig<ZetaRFConfigs::Config868_FixedLength_CRC_Preamble10_Sync4_Payload8, ZetaRFEZRadio::EZRadioSi4455<ZetaRFHal::ArduinoSpiHal<Ts...>> >;

// Variable Length
template<class ...Ts>
using ZetaRF868_VL = ZetaRFConfig<ZetaRFConfigs::Config868_VariableLength_CRC_Preamble10_Sync4_Payload8, ZetaRFEZRadio::EZRadioSi4455<ZetaRFHal::ArduinoSpiHal<Ts...>> >;


/*
template<class ...Ts>
using ZetaRF433 = ZetaRFImpl<ZetaRFConfigs::Config433_FixedLength_CRC_Preamble10_Sync4_Payload8, Ts...>;

// Configs using variable length packets
template<class ...Ts>
using ZetaRF868_VL = ZetaRFImpl<ZetaRFConfigs::Config868_VariableLength_CRC_Preamble10_Sync4_Payload8, Ts...>;

template<class ...Ts>
using ZetaRF433_VL = ZetaRFImpl<ZetaRFConfigs::Config433_VariableLength_CRC_Preamble10_Sync4_Payload8, Ts...>;
//*/