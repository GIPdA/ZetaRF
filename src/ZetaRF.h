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


#include <cstdint>

namespace ZetaRF
{
    using RadioState = ZetaRFRadio::RadioState;
    using Status = ZetaRFRadio::Status;
    using ReadResult = ZetaRFRadio::ReadPacketResult;

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

    constexpr ZetaRF::RadioState radioStateFromValue(uint8_t state) {
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
    constexpr const char* radioStateText(ZetaRF::RadioState state) {
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
class ZetaRFImpl// : public ZetaRF
{
public:
    //! Current internal radio state
    ZetaRF::RadioState radioState() {
        return m_radio.radioState();
    }

    flagset::bitflag<ZetaRF::Status> status() const {
        return m_radio.status();
    }

    bool checkFor(ZetaRF::Status status) {
        return m_radio.testAndClearStatus(status);
    }

    bool isAlive() {
        return m_radio.isAlive();
    }

    bool controlError() {
        return m_radio.statusHasError();
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

    bool update() {
        return m_radio.update();
    }

    //bool isNotResponding() const;

    //! Hold the chip in reset. begin must be called to restart it.
    void shutdown() {
        m_radio.holdInReset();
    }


    // ### PACKET SENDING METHODS ###

    //! Send either fixed or variable length packet depending on radio config
    bool sendPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100) {
        if (Config::VariableLengthPacketConfiguration)
            return m_radio.sendVariableLengthPacket(channel, data, length, timeout_ms);
        else
            return m_radio.sendFixedLengthPacket(channel, data, length, timeout_ms);
    }

    bool sendFixedLengthPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100) {
        return m_radio.sendFixedLengthPacket(channel, data, length, timeout_ms);
    }

    bool sendVariableLengthPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100) {
        static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return m_radio.sendVariableLengthPacket(channel, data, length, timeout_ms);
    }


    // ### PACKET RECEIVING METHODS ###

    //! Read either fixed or variable length packet depending on radio config
    ZetaRF::ReadPacketResult readPacket(uint8_t* data, uint8_t byteCount) {
        if (Config::VariableLengthPacketConfiguration) {
            uint8_t packetDataLength {0};
            return m_radio.readVariableLengthPacket(data, byteCount, packetDataLength);
        }
        else
            return m_radio.readFixedLengthPacket(data, byteCount);
    }

    ZetaRF::ReadPacketResult readFixedLengthPacket(uint8_t* data, uint8_t byteCount) {
        return m_radio.readFixedLengthPacket(data, byteCount);
    }

    ZetaRF::ReadPacketResult readVariableLengthPacket(uint8_t* data, uint8_t maxByteCount, uint8_t& packetDataLength) {
        static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return m_radio.readVariableLengthPacket(data, maxByteCount, packetDataLength);
    }


    bool startListeningOnChannel(uint8_t newChannel) {
        return m_radio.startListeningOnChannel(newChannel);
    }
    bool startListeningSinglePacketOnChannel(uint8_t newChannel) {
        return m_radio.startListeningSinglePacketOnChannel(newChannel);
    }

    bool checkReceived() {
        return m_radio.checkReceived();
    }

    void resetRxFifo() {
        m_radio.cmd_resetRxFifo();
    }


    unsigned int readCurrentRssi()
    {
        auto rssi = m_radio.cmd_readModemStatus().CURR_RSSI;
        return m_radio.statusHasError() ? 0 : rssi;
    }

    //! Rssi value of the last received packet (reset upon entering RX).
    //! Valid only when using single packet listening mode, if you read it before restarting RX.
    unsigned int readLatchedRssi()
    {
        auto rssi = m_radio.cmd_readModemStatus().LATCH_RSSI;
        return m_radio.statusHasError() ? 0 : rssi;
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
