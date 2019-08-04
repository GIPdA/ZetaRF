/*! @file ZetaRF.h
 *
 * @brief This file contains the public functions controlling the Si4455 radio chip.
 *
 * License: see LICENSE file
 */

#pragma once

#include "si4455_defs.h"

#include "zetarf_hal.h"
#include "zetarf_radio.h"

#include "configs/config868_fixedsize_crc_preamble10_sync4_payload8.h"

//#define ZETARF_DEBUG_VERBOSE_ON


class ZetaRF
{
public:
    using RadioState = ZetaRFRadio::RadioState;
    using Status = ZetaRFRadio::Status;

    //bool beginWithConfigurationArray(ZetaRFConfigs::RadioConnfiguration const& config);

    //! Current internal radio state
    ZetaRF::RadioState radioState();


    //bool isNotResponding() const;



protected:
    RadioState radioStateFromValue(uint8_t state) const {
        switch (state) {
            case 1: return RadioState::Sleep;
            case 2: return RadioState::SpiActive;
            case 3: return RadioState::Ready;
            case 4: return RadioState::Ready2;
            case 5: return RadioState::TxTune;
            case 6: return RadioState::RxTune;
            case 7: return RadioState::Tx;
            case 8: return RadioState::Rx;

            default: return RadioState::Invalid;
        }
    }
    const char* radioStateText(RadioState state) const {
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
    }

    bool begin(uint8_t const* configArray);

private:
    ZetaRFRadioImpl< ZetaRfHal< ChipSelectPin<10>, ShutdownPin<9>, IrqPin<8> > > m_radio;
};


template<typename Config>
class ZetaRF_C : public ZetaRF
{
public:
    bool begin() {
        return ZetaRF::begin(Config::RadioConfigurationDataArray);
    }
};


//CREATE_FLAGSET(ZetaRF::Status)


using ZetaRF868 = ZetaRF_C<ZetaRFConfigs::Config868_FixedSize_CRC_Preamble10_Sync4_Payload8>;

