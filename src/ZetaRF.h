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
#include "configs/config868_fixedsize_crc_preamble10_sync4_payload8.h"
#include "configs/config433_fixedsize_crc_preamble10_sync4_payload8.h"

#include <cstdint>

namespace ZetaRF
{
    using RadioState = ZetaRFRadio::RadioState;
    using Status = ZetaRFRadio::Status;
    using ReadPacketResult = ZetaRFRadio::ReadPacketResult;
}


template <typename Config, class ...HalTypes>
class ZetaRFImpl// : public ZetaRF
{
public:
    //! Current internal radio state
    ZetaRF::RadioState radioState();

    bitflag<ZetaRF::Status> status() const {
        return m_radio.status();
    }


    //!! Load radio config
    bool begin() {
        if (Config::VariableLengthPacketConfiguration)
            m_radio.setPacketLength(0); // Variable length packets listen with packet size of zero
        else
            m_radio.setPacketLength(Config::PacketLength);

        return m_radio.beginWithConfigurationDataArray(Config::RadioConfigurationDataArray);
    }

    bool update() {
        return m_radio.update();
    }

    //bool isNotResponding() const;

    // ### PACKET SENDING METHODS ###
    bool sendPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100) {
        return m_radio.sendPacket(channel, data, length, timeout_ms);
    }

    bool sendVariableLengthPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100) {
        static_assert(Config::VariableLengthPacketConfiguration, "Radio configuration does not support variable length packets.");
        return m_radio.sendVariableLengthPacket(channel, data, length, timeout_ms);
    }


    // ### PACKET RECEIVING METHODS ###
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

    bool checkReceived() {
        return m_radio.checkReceived();
    }

    Si4455_PartInfo const& readPartInformation() {
        return m_radio.cmd_readPartInformation();
    }

    Si4455_FuncInfo const& readFunctionRevisionInformation() {
        return m_radio.cmd_readFunctionRevisionInformation();
    }


protected:
    ZetaRF::RadioState radioStateFromValue(uint8_t state) const {
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
    const char* radioStateText(ZetaRF::RadioState state) const {
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

private:
    ZetaRFRadioImpl< ZetaRfHal<HalTypes...> > m_radio;
};


// Default configs
template<class ...Ts>
using ZetaRF868 = ZetaRFImpl<ZetaRFConfigs::Config868_FixedSize_CRC_Preamble10_Sync4_Payload8, Ts...>;

template<class ...Ts>
using ZetaRF433 = ZetaRFImpl<ZetaRFConfigs::Config433_FixedSize_CRC_Preamble10_Sync4_Payload8, Ts...>;

