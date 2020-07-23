/*!
 * @brief Radio configuration data adapted from SiLabs's WDS software.
 *
 * Configuration for 868 MHz DORJI DRF4463F module.
 */

#pragma once

#include <stdint.h>

// INPUT DATA
/*
// Crys_freq(Hz): 30000000    Crys_tol(ppm): 20    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 1    ANT_DIV: 0    PM_pattern: 0    
// MOD_type: 3    Rsymb(sps): 10000    Fdev(Hz): 20000    RXBW(Hz): 150000    Manchester: 1    AFC_en: 1    Rsymb_error: 0.0    Chip-Version: 2    
// RF Freq.(MHz): 868    API_TC: 29    fhst: 250000    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 0    Hi_pfm_div: 1    
// 
// # RX IF frequency is  -468750 Hz
// # WB filter 1 (BW = 114.46 kHz);  NB-filter 8 (BW = 54.51 kHz)
// 
// Modulation index: 4
*/

namespace ZetaRfConfigs {

struct Config868_Si4463_VariableLength_10kbps_CRC_Whitening_LongRange
{
    static const uint8_t RadioConfigurationDataArray[];

    constexpr static uint8_t DefaultChannel = 0;
    constexpr static uint8_t PacketLength = 32;
    constexpr static bool VariableLengthPacketConfiguration = true;
};

} // namespace ZetaRfConfigs
