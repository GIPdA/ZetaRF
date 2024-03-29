
#include "config915_variablelength_crc_preamble10_sync4_payload8.hpp"

// INPUT DATA
/*
// Crys_freq(Hz): 30000000    Crys_tol(ppm): 30    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 0
// MOD_type: 2    Rsymb(sps): 128000    Fdev(Hz): 70000    RXBW(Hz): 305000    Manchester: 1    AFC_en: 1    Rsymb_error: 0.0    Chip-Version: 2
// RF Freq.(MHz): 915    API_TC: 28    fhst: 250000    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 0    Hi_pfm_div: 0
//
// # WB filter 2 (BW = 274.83 kHz);  NB-filter 2 (BW = 274.83 kHz)
//
// Modulation index: 1
*/



// CONFIGURATION COMMANDS

/*
// Command:                  RF_POWER_UP
// Description:              Command to power-up the device and select the operational mode and functionality.
*/
#define RF_POWER_UP 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80

/*
// Set properties:           RF_INT_CTL_ENABLE_2
// Number of properties:     2
// Group ID:                 0x01
// Start ID:                 0x00
// Default values:           0x04, 0x00,
// Descriptions:
//   INT_CTL_ENABLE - This property provides for global enabling of the three interrupt groups (Chip, Modem and Packet Handler) in order to generate HW interrupts at the NIRQ pin.
//   INT_CTL_PH_ENABLE - Enable individual interrupt sources within the Packet Handler Interrupt Group to generate a HW interrupt on the NIRQ output pin.
*/
#define RF_INT_CTL_ENABLE_2 0x11, 0x01, 0x02, 0x00, 0x05, 0x38

/*
// Set properties:           RF_INT_CTL_CHIP_ENABLE_1
// Number of properties:     1
// Group ID:                 0x01
// Start ID:                 0x03
// Default values:           0x04,
// Descriptions:
//   INT_CTL_CHIP_ENABLE - Enable individual interrupt sources within the Chip Interrupt Group to generate a HW interrupt on the NIRQ output pin.
*/
#define RF_INT_CTL_CHIP_ENABLE_1 0x11, 0x01, 0x01, 0x03, 0x6C

/*
// Set properties:           RF_FRR_CTL_A_MODE_4
// Number of properties:     4
// Group ID:                 0x02
// Start ID:                 0x00
// Default values:           0x01, 0x02, 0x09, 0x00,
// Descriptions:
//   FRR_CTL_A_MODE - Fast Response Register A Configuration.
//   FRR_CTL_B_MODE - Fast Response Register B Configuration.
//   FRR_CTL_C_MODE - Fast Response Register C Configuration.
//   FRR_CTL_D_MODE - Fast Response Register D Configuration.
*/
#define RF_FRR_CTL_A_MODE_4 0x11, 0x02, 0x04, 0x00, 0x09, 0x04, 0x06, 0x08

/*
// Set properties:           RF_EZCONFIG_XO_TUNE_1
// Number of properties:     1
// Group ID:                 0x24
// Start ID:                 0x03
// Default values:           0x40,
// Descriptions:
//   EZCONFIG_XO_TUNE - Configure the internal capacitor frequency tuning bank for the crystal oscillator.
*/
#define RF_EZCONFIG_XO_TUNE_1 0x11, 0x24, 0x01, 0x03, 0x52

/*
// Command:                  RF_WRITE_TX_FIFO
// Description:              Writes data byte(s) to the TX FIFO.
*/
#define RF_WRITE_TX_FIFO 0x66, 0xEC, 0x6D, 0xE4, 0xAC, 0x30, 0x78, 0xB0, 0x9E, 0x16, 0x3B, 0xC1, 0x0D, 0x63, 0x9A, 0x73, 0x64, 0xA3, 0x9B, 0xA7, \
0xB4, 0xFD, 0x8C, 0xC8, 0xFD, 0x7A, 0x9C, 0x74, 0x1D, 0x9F, 0x90, 0x0D, 0xE2, 0x9B, 0x11, 0xC2, 0x46, 0xA9, 0x7C, 0xD9, \
0x1E, 0x6C, 0x66, 0x1E, 0x96, 0x63, 0x1D, 0xB6, 0x45, 0xDE, 0x58, 0x3C, 0xDD, 0x3C, 0xA4, 0x72, 0x6F, 0x5F, 0x69, 0xD0, \
0xDD, 0xD9, 0x55, 0x1D, 0x0E, 0xE5, 0x1D, 0x5A, 0x7E, 0x2F, 0x7A, 0xFD, 0xB7, 0x35, 0x20, 0x91, 0x03, 0x64, 0xE9, 0x04, \
0xC9, 0x16, 0xC3, 0x70, 0x2D, 0x76, 0xDE, 0x57, 0x96, 0x78, 0x48, 0x71, 0x0D, 0xC7, 0x50, 0x0C, 0xD0, 0xB9, 0x17, 0x55, \
0xFA, 0x1E, 0x2D, 0xF3, 0x49, 0x71, 0x67, 0x2B, 0x05, 0xE5, 0x86, 0x56, 0xA2, 0x90

/*
// Command:                  RF_NOP
// Description:              No Operation command.
*/
#define RF_NOP 0x00

/*
// Command:                  RF_WRITE_TX_FIFO_1
// Description:              Writes data byte(s) to the TX FIFO.
*/
#define RF_WRITE_TX_FIFO_1 0x66, 0x16, 0xBC, 0x3F, 0x62, 0x1C, 0x2E, 0x71, 0x48, 0xB9, 0x12, 0xAF, 0x0E, 0xC6, 0x8F, 0x59, 0x95, 0x12, 0x33, 0x07, \
0x4F, 0x42, 0x04, 0xAF, 0x1F, 0x9F, 0x34, 0x4D, 0xE8, 0xC4, 0xDB, 0xA9, 0xF0, 0xC3, 0x9F, 0x21, 0xE6, 0xDE, 0xB4, 0xCF, \
0x1B, 0x30, 0xD5, 0x74, 0x32, 0xB0, 0x23, 0x30, 0xAB, 0x61, 0x3E, 0xF6, 0x4A, 0xA9, 0x6E, 0xF4, 0xA4, 0xE1, 0xBA, 0x79, \
0x71, 0x3E, 0x8A, 0x12, 0xA4, 0xB6, 0x47, 0x89, 0x5C, 0xF3, 0xCF, 0x25, 0x89, 0x6B, 0x61, 0xA4, 0x61, 0x17, 0x1F, 0xAC, \
0x5E, 0x2E, 0x0D, 0x60, 0x9A, 0x12, 0x1E, 0x8D, 0x94, 0x5E, 0x9D, 0x31, 0x72, 0xD1, 0xA2, 0x82, 0x58, 0xDB, 0x64, 0x2B, \
0x55, 0x03, 0xD9, 0x7C, 0x26, 0xDD, 0x3E, 0xE3, 0x3F, 0xC6, 0x32, 0x9B

/*
// Command:                  RF_EZCONFIG_CHECK
// Description:              Validates the EZConfig array was written correctly.
*/
#define RF_EZCONFIG_CHECK 0x19, 0x1A, 0xB2

/*
// Command:                  RF_GPIO_PIN_CFG
// Description:              Configures the GPIO pins.
*/
#define RF_GPIO_PIN_CFG 0x13, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00


namespace ZetaRfConfigs {

const uint8_t Config915_VariableLength_CRC_Preamble10_Sync4_Payload8::RadioConfigurationDataArray[]
{
    0x07, RF_POWER_UP, \
    0x06, RF_INT_CTL_ENABLE_2, \
    0x05, RF_INT_CTL_CHIP_ENABLE_1, \
    0x08, RF_FRR_CTL_A_MODE_4, \
    0x05, RF_EZCONFIG_XO_TUNE_1, \
    0x72, RF_WRITE_TX_FIFO, \
    0x01, RF_NOP, \
    0x70, RF_WRITE_TX_FIFO_1, \
    0x03, RF_EZCONFIG_CHECK, \
    0x08, RF_GPIO_PIN_CFG, \
    0x00
};

} // namespace ZetaRfConfigs
