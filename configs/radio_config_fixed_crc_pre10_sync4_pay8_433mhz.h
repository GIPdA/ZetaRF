/*! @file radio_config.h
 * @brief This file contains the automatically generated
 * configurations.
 *
 * @n WDS GUI Version: 3.2.11.0
 * @n Device: Si4455 Rev.: B1
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2017 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef RADIO_CONFIG_H_
#define RADIO_CONFIG_H_

// USER DEFINED PARAMETERS
// Define your own parameters here

// INPUT DATA
/*
 // Crys_freq(Hz): 30000000    Crys_tol(ppm): 30    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 0
 // MOD_type: 2    Rsymb(sps): 2400    Fdev(Hz): 30000    RXBW(Hz): 114000    Manchester: 1    AFC_en: 1    Rsymb_error: 0.0    Chip-Version: 2
 // RF Freq.(MHz): 433    API_TC: 28    fhst: 250000    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 0    Hi_pfm_div: 0
 //
 // # WB filter 1 (BW = 114.46 kHz);  NB-filter 1 (BW = 114.46 kHz)
 
 //
 // Modulation index: 25
 */


// CONFIGURATION PARAMETERS
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ                     30000000L
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER                    0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH               0x08
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP        0x03
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET       0xF000
#define RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD					   {0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5}


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
#define RF_WRITE_TX_FIFO 0x66, 0x82, 0x0A, 0xB3, 0x00, 0x23, 0xF5, 0xF4, 0x28, 0x36, 0x40, 0x11, 0xFB, 0xBE, 0x63, 0xA2, 0x9D, 0x5A, 0x92, 0x9D, \
0xCD, 0xD7, 0x9E, 0xD1, 0xC8, 0x98, 0x64, 0x24, 0x30, 0x98, 0xA7, 0xAB, 0x0D, 0x27, 0xF7, 0x31, 0x3D, 0x73, 0x89, 0xF2, \
0x73, 0xFB, 0x23, 0xE5, 0x3E, 0xC2, 0xEF, 0xAF, 0x28, 0xDD, 0x33, 0xC8, 0x8A, 0xE5, 0xD8, 0xEE, 0x45, 0x60, 0x28, 0x14, \
0xCF, 0x24, 0x38, 0x7B, 0xD9, 0xAE, 0x32, 0x69, 0x8F, 0x55, 0xD9, 0x20, 0xB9, 0xD3, 0x5E, 0xCE, 0x43, 0x2F, 0x90, 0xA8, \
0xBD, 0xCC, 0x2B, 0xD5, 0x80, 0x49, 0xB8, 0x81, 0x32, 0x5E, 0x4E, 0xB8, 0xD2, 0x89, 0x16, 0x82, 0x9B, 0xC6, 0xC0, 0x9D, \
0x7B, 0xF6, 0x66, 0x9C, 0x4B, 0x1D, 0x0D, 0x35, 0x32, 0x15, 0x4B, 0xE8, 0xA8, 0x9A

/*
 // Command:                  RF_NOP
 // Description:              No Operation command.
 */
#define RF_NOP 0x00

/*
 // Command:                  RF_WRITE_TX_FIFO_1
 // Description:              Writes data byte(s) to the TX FIFO.
 */
#define RF_WRITE_TX_FIFO_1 0x66, 0xE4, 0x42, 0x72, 0xFA, 0x21, 0xA2, 0xE5, 0xB1, 0x0E, 0xB1, 0xD3, 0xCA, 0x56, 0x7D, 0x3B, 0xF0, 0x0A, 0x37, 0x49, \
0xF0, 0x74, 0x3D, 0xF3, 0x76, 0x2C, 0x9F, 0x28, 0x6A, 0x90, 0x21, 0xBB, 0x6B, 0xAD, 0xFA, 0xB8, 0x46, 0x4E, 0x1B, 0x4E, \
0xF6, 0x51, 0xF2, 0xE0, 0x94, 0x3F, 0xD4, 0xD5, 0xC4, 0x57, 0xE8, 0x86, 0x91, 0xED, 0x2C, 0xEA, 0x6C, 0x13, 0x29, 0x3F, \
0x83, 0x89, 0x1E, 0xD4, 0x4B, 0x45, 0x05, 0x25, 0x32, 0x73, 0xA4, 0x5A, 0xF4, 0x59, 0xCA, 0xFA, 0x5D, 0xD9, 0x3A, 0xE6, \
0x39, 0xC6, 0x26, 0xE3, 0x6F, 0x26, 0x94, 0x06, 0x98, 0x46, 0xEF, 0x4D, 0x50, 0x16, 0xF9, 0xCC, 0x25, 0x72, 0x95, 0xFE, \
0x81, 0x0C, 0x00, 0x8D, 0xD0, 0xC6, 0x53, 0xD2, 0xFB, 0xF7, 0x00, 0x94

/*
 // Command:                  RF_EZCONFIG_CHECK
 // Description:              Validates the EZConfig array was written correctly.
 */
#define RF_EZCONFIG_CHECK 0x19, 0x16, 0x3C

/*
 // Command:                  RF_GPIO_PIN_CFG
 // Description:              Configures the GPIO pins.
 */
#define RF_GPIO_PIN_CFG 0x13, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00


// AUTOMATICALLY GENERATED CODE!
// DO NOT EDIT/MODIFY BELOW THIS LINE!
// --------------------------------------------

#ifndef FIRMWARE_LOAD_COMPILE
#define RADIO_CONFIGURATION_DATA_ARRAY { \
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
0x00 \
}
#else
#define RADIO_CONFIGURATION_DATA_ARRAY { 0 }
#endif

// DEFAULT VALUES FOR CONFIGURATION PARAMETERS
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ_DEFAULT                     30000000L
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER_DEFAULT                    0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH_DEFAULT               0x10
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP_DEFAULT        0x01
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET_DEFAULT       0x1000
#define RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD_DEFAULT  		           {0x42, 0x55, 0x54, 0x54, 0x4F, 0x4E, 0x31} // BUTTON1

#define RADIO_CONFIGURATION_DATA_RADIO_PATCH_INCLUDED                      0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PATCH_SIZE                          0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PATCH                               {  }

#ifndef RADIO_CONFIGURATION_DATA_ARRAY
#error "This property must be defined!"
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ          RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ_DEFAULT
#endif

#ifndef RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER         RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER_DEFAULT
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH    RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH_DEFAULT
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP   RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP_DEFAULT
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET  RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET_DEFAULT
#endif

#ifndef RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD
#define RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD         RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD_DEFAULT
#endif

#define RADIO_CONFIGURATION_DATA { \
Radio_Configuration_Data_Array,                            \
RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,                   \
RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH,              \
RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP,       \
RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET,      \
RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD                    \
}

#endif /* RADIO_CONFIG_H_ */
