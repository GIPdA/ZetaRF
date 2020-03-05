/*! @file ZetaRF.h
 *
 * @brief ZetaRF library main file
 *
 * License: see LICENSE file
 */

#pragma once

#include "si4455_defs.h"

#include "zetarf_hal.h"
#include "zetarf_radio.h"

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

namespace ZetaRF
{
    using RadioState = ZetaRFRadio::RadioState;
    using Status = ZetaRFRadio::Status;
    using StatusFlags = ZetaRFRadio::StatusFlags;
    using ReadResult = ZetaRFRadio::ReadPacketResult;

    using onEventCallback = ZetaRFRadio::onEventCallback; // -> void onZetaEvents(StatusFlags status) {}
    using onDataReceivedCallback = ZetaRFRadio::onDataReceivedCallback; // -> void onZetaDataReceived() {}

    class ReadPacketResult {
        friend inline bool operator==(ReadPacketResult a, ReadPacketResult b);
        friend inline bool operator==(ReadPacketResult a, ReadResult b);
        ZetaRFRadio::ReadPacketResult m_value;

    public:
        /* implicit */ ReadPacketResult(decltype(m_value) value) : m_value(value) {}

        operator bool() const {
            return m_value == ZetaRFRadio::ReadPacketResult::Success;
        }
    };

    inline bool operator==(ReadPacketResult a, ReadPacketResult b) {
        return a.m_value == b.m_value;
    }
    inline bool operator==(ReadPacketResult a, ReadResult b) {
        return a.m_value == b;
    }

    inline bool operator!=(ReadPacketResult a, ReadPacketResult b) {
        return !(a == b);
    }
    inline bool operator!=(ReadPacketResult a, ReadResult b) {
        return !(a == b);
    }

    CONSTEXPR ZetaRF::RadioState radioStateFromValue(uint8_t state) {
        switch (state) {
            case 1: return ZetaRF::RadioState::Sleep;
            case 2: return ZetaRF::RadioState::SpiActive;
            case 3: return ZetaRF::RadioState::Ready;
            case 4: return ZetaRF::RadioState::Ready2;
            case 5: return ZetaRF::RadioState::TxTune;
            case 6: return ZetaRF::RadioState::RxTune;
            case 7: return ZetaRF::RadioState::Tx;
            case 8: return ZetaRF::RadioState::Rx;

            default: return ZetaRF::RadioState::Invalid;
        }
    }
    CONSTEXPR const char* radioStateText(ZetaRF::RadioState state) {
        switch (state) {
            case ZetaRF::RadioState::Sleep: return "Sleep";
            case ZetaRF::RadioState::SpiActive: return "SpiActive";
            case ZetaRF::RadioState::Ready: return "Ready";
            case ZetaRF::RadioState::Ready2: return "Ready2";
            case ZetaRF::RadioState::TxTune: return "TxTune";
            case ZetaRF::RadioState::RxTune: return "RxTune";
            case ZetaRF::RadioState::Tx: return "Tx";
            case ZetaRF::RadioState::Rx: return "Rx";

            default: return "Invalid";
        }
    }
}


template <typename Config, class ...HalTypes>
class ZetaRFImpl
{
public:
    //! Current internal radio state
    ZetaRF::RadioState radioState() {
        return m_radio.radioStateViaFrr();
    }

    bool hasDataAvailble() const {
        return m_radio.hasDataAvailable();
    }
    bool hasDataAvailble() {
        return m_radio.hasDataAvailable();
    }
    bool available() const {
        return hasDataAvailble();
    }
    bool available() {
        return hasDataAvailble();
    }


    bool update() {
        return m_radio.update();
    }


    bool isAlive() {
        return m_radio.isAlive();
    }

    //!! Load radio config
    bool begin() {
        if (Config::VariableLengthPacketConfiguration)
            m_radio.setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            m_radio.setPacketLength(Config::PacketLength);

        return m_radio.beginWithConfigurationDataArray(Config::RadioConfigurationDataArray);
    }

    //! Load radio config with the specified packet length (no effect in variable length packet mode).
    bool beginWithPacketLengthOf(uint8_t packetLength) {
        if (Config::VariableLengthPacketConfiguration)
            m_radio.setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            m_radio.setPacketLength(packetLength);

        return m_radio.beginWithConfigurationDataArray(Config::RadioConfigurationDataArray);
    }


    ZetaRF::StatusFlags events() const {
        return m_radio.eventFlagsMask();
    }
    void setEvents(ZetaRF::StatusFlags events) {
        m_radio.setEventFlagsMask(events);
    }
    void onEvent(ZetaRF::onEventCallback callback) {
        m_radio.setEventCallback(callback);
    }
    void onDataReceivedCallback(ZetaRF::onDataReceivedCallback callback) {
        m_radio.setDataReceivedCallback(callback);
    }

    //bool isNotResponding() const;

    //! Hold the chip in reset. begin must be called to restart it.
    void shutdown() {
        m_radio.holdInReset();
    }


    // ### PACKET SENDING METHODS ###

    //! Send either fixed or variable length packet depending on radio config. Data is put in FIFO and set to send.
    //! For fixed length packet mode, if @a dataSize is less than the packet size, zeros are automatically appended.
    bool sendPacket(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        if (Config::VariableLengthPacketConfiguration)
            return m_radio.sendVariableLengthPacket(channel, data, dataSize, timeoutForReady_ms);
        else
            return m_radio.sendFixedLengthPacket(channel, data, dataSize, timeoutForReady_ms);
    }

    //! Send fixed length packet, config length or user-set length is used. RX must be waiting for that length of packet. Data is put in FIFO and set to send.
    //! @a data must point to at least 'packet length' bytes.
    bool sendFixedLengthPacket(uint8_t channel, uint8_t const* data, unsigned long timeoutForReady_ms = 100) {
        return m_radio.sendFixedLengthPacket(channel, data, timeoutForReady_ms);
    }

    //! Send fixed length packet with specified length. RX must be waiting for that length of packet. Data is put in FIFO and set to send.
    //! If @a dataSize is less than the packet size, zeros are automatically appended.
    bool sendFixedLengthPacket(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeoutForReady_ms = 100) {
        return m_radio.sendFixedLengthPacket(channel, data, dataSize, timeoutForReady_ms);
    }

    //! Send variable length packet. Data is put in FIFO and set to send.
    bool sendVariableLengthPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeoutForReady_ms = 100) {
        static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return m_radio.sendVariableLengthPacket(channel, data, length, timeoutForReady_ms);
    }


    // ### PACKET RECEIVING METHODS ###

    //! Read either fixed or variable length packet depending on radio config
    ZetaRF::ReadPacketResult readPacketTo(uint8_t* data, uint8_t byteCount) {
        if (Config::VariableLengthPacketConfiguration) {
            uint8_t packetDataLength {0};
            return m_radio.readVariableLengthPacket(data, byteCount, packetDataLength);
        }
        else
            return m_radio.readFixedLengthPacket(data, byteCount);
    }

    //! Read a packet with size set via beginWithPacketLengthOf(). @a data must point to a buffer large enough!
    ZetaRF::ReadPacketResult readPacketTo(uint8_t* data) {
        return readPacketTo(data, m_radio.packetLength());
    }

    ZetaRF::ReadPacketResult readFixedLengthPacketTo(uint8_t* data, uint8_t byteCount) {
        return m_radio.readFixedLengthPacket(data, byteCount);
    }

    ZetaRF::ReadPacketResult readVariableLengthPacketTo(uint8_t* data, uint8_t maxByteCount, uint8_t& packetDataLength) {
        static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return m_radio.readVariableLengthPacket(data, maxByteCount, packetDataLength);
    }


    uint8_t packetLength() const {
        return m_radio.packetLength();
    }

    bool startListeningOnChannel(uint8_t newChannel) {
        return m_radio.startListeningOnChannel(newChannel);
    }
    bool startListeningSinglePacketOnChannel(uint8_t newChannel) {
        return m_radio.startListeningSinglePacketOnChannel(newChannel);
    }


    //! Checks if there is enough space left in TX fifo for one packet
    /*bool canSendOnePacket() {
        //
    }//*/

    // Space left in TX FIFO
    uint8_t bytesAvailableInTxFifo()
    {
        return m_radio.bytesAvailableInTxFifo();
    }
    // Bytes stored in RX FIFO
    uint8_t bytesAvailableInRxFifo()
    {
        return m_radio.bytesAvailableInRxFifo();
    }

    void resetRxFifo() {
        m_radio.cmd_resetRxFifo();
    }


    uint8_t readCurrentRssi()
    {
        uint8_t const rssi = m_radio.cmd_readModemStatus().CURR_RSSI;
        return m_radio.statusHasError() ? 0 : rssi;
    }

    //! Rssi value of the last received packet (reset upon entering RX).
    //! Valid only when using single packet listening mode, if you read it before restarting RX.
    uint8_t readLatchedRssi()
    {
        return m_radio.readLatchedRssiViaFrr();
    }


    Si4455_PartInfo const& readPartInformation() {
        return m_radio.cmd_readPartInformation();
    }

    Si4455_FuncInfo const& readFunctionRevisionInformation() {
        return m_radio.cmd_readFunctionRevisionInformation();
    }

private:
    ZetaRFRadioImpl< ZetaRfHal<HalTypes...> > m_radio;
};


// Default configs
template<class ...Ts>
using ZetaRF868 = ZetaRFImpl<ZetaRFConfigs::Config868_FixedLength_CRC_Preamble10_Sync4_Payload8, Ts...>;

template<class ...Ts>
using ZetaRF433 = ZetaRFImpl<ZetaRFConfigs::Config433_FixedLength_CRC_Preamble10_Sync4_Payload8, Ts...>;

// Configs using variable length packets
template<class ...Ts>
using ZetaRF868_VL = ZetaRFImpl<ZetaRFConfigs::Config868_VariableLength_CRC_Preamble10_Sync4_Payload8, Ts...>;

template<class ...Ts>
using ZetaRF433_VL = ZetaRFImpl<ZetaRFConfigs::Config433_VariableLength_CRC_Preamble10_Sync4_Payload8, Ts...>;
