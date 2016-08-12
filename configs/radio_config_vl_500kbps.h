/*! @file radio_config.h
 * @brief This file contains the automatically generated
 * configurations.
 *
 * @n WDS GUI Version: 3.2.10.0
 * @n Device: Si4455 Rev.: B1                                 
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2015 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef RADIO_CONFIG_H_
#define RADIO_CONFIG_H_

// USER DEFINED PARAMETERS
// Define your own parameters here

// INPUT DATA
/*
// Crys_freq(Hz): 30000000    Crys_tol(ppm): 30    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 0    
// MOD_type: 2    Rsymb(sps): 500000    Fdev(Hz): 70000    RXBW(Hz): 305000    Manchester: 1    AFC_en: 1    Rsymb_error: 0.0    Chip-Version: 2    
// RF Freq.(MHz): 868    API_TC: 28    fhst: 250000    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 0    Hi_pfm_div: 0    
// 
// # WB filter 5 (BW = 593.60 kHz);  NB-filter 5 (BW = 593.60 kHz) 
// 
// Modulation index: 0
*/


// CONFIGURATION PARAMETERS
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ                     30000000L
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER                    0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH               0x09
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP        0x03
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET       0xF000
#define RADIO_CONFIGURATION_DATA_CUSTOM_PAYLOAD					   {0x08, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5}


// CONFIGURATION COMMANDS

/*
// Command:                  RF_POWER_UP
// Description:              Command to power-up the device and select the operational mode and functionality.
*/
#define RF_POWER_UP 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80

/*
// Set properties:           RF_INT_CTL_ENABLE_1
// Number of properties:     1
// Group ID:                 0x01
// Start ID:                 0x00
// Default values:           0x04, 
// Descriptions:
//   INT_CTL_ENABLE - This property provides for global enabling of the three interrupt groups (Chip, Modem and Packet Handler) in order to generate HW interrupts at the NIRQ pin.
*/
#define RF_INT_CTL_ENABLE_1 0x11, 0x01, 0x01, 0x00, 0x04

/*
// Set properties:           RF_INT_CTL_CHIP_ENABLE_1
// Number of properties:     1
// Group ID:                 0x01
// Start ID:                 0x03
// Default values:           0x04, 
// Descriptions:
//   INT_CTL_CHIP_ENABLE - Enable individual interrupt sources within the Chip Interrupt Group to generate a HW interrupt on the NIRQ output pin.
*/
#define RF_INT_CTL_CHIP_ENABLE_1 0x11, 0x01, 0x01, 0x03, 0x74

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
#define RF_FRR_CTL_A_MODE_4 0x11, 0x02, 0x04, 0x00, 0x02, 0x04, 0x09, 0x0A

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
#define RF_WRITE_TX_FIFO 0x66, 0xA3, 0xC0, 0x0F, 0x10, 0xCE, 0xF1, 0x84, 0x1F, 0xA2, 0x85, 0xFF, 0x0E, 0xE8, 0xE8, 0xD4, 0x25, 0x5B, 0x01, 0xBD, \
0x72, 0xB7, 0x2D, 0x37, 0x39, 0xEC, 0x73, 0x67, 0x19, 0xE8, 0xBA, 0xCF, 0x97, 0x25, 0xA8, 0xDF, 0x94, 0x0B, 0x70, 0x3B, \
0xB3, 0xC1, 0x18, 0xF3, 0xD9, 0xBE, 0x64, 0x32, 0x44, 0xC7, 0x04, 0xAB, 0x62, 0xC0, 0x81, 0x9D, 0x5E, 0x94, 0xBB, 0x85, \
0xCC, 0x42, 0x66, 0xDE, 0x38, 0x68, 0xFC, 0x46, 0x2B, 0x79, 0x84, 0x6F, 0xD9, 0x10, 0x96, 0xF4, 0xE3, 0x7D, 0xDF, 0xEF, \
0xFB, 0xC6, 0x80, 0x16, 0xF0, 0xEF, 0x45, 0x54, 0xCC, 0xAE, 0x66, 0xE9, 0x16, 0xE0, 0xD7, 0x91, 0xA5, 0x4D, 0x89, 0xAE, \
0x36, 0xA0, 0x2D, 0x3E, 0xC9, 0xD9, 0x9D, 0xF8, 0xDD, 0xDF, 0x69, 0x80, 0x3D, 0x72

/*
// Command:                  RF_NOP
// Description:              No Operation command.
*/
#define RF_NOP 0x00

/*
// Command:                  RF_WRITE_TX_FIFO_1
// Description:              Writes data byte(s) to the TX FIFO.
*/
#define RF_WRITE_TX_FIFO_1 0x66, 0x3E, 0x88, 0xE6, 0x82, 0x1B, 0x7F, 0xAF, 0x2A, 0x1C, 0x89, 0x21, 0x45, 0x70, 0x0F, 0xE2, 0x0F, 0x9E, 0x44, 0x9E, \
0x28, 0x7B, 0xC4, 0x26, 0xA9, 0x14, 0xBB, 0x48, 0xD1, 0x49, 0x69, 0x4B, 0x64, 0xCA, 0xE9, 0x25, 0xCF, 0x36, 0x3B, 0x92, \
0xFB, 0xBB, 0xA5, 0x62, 0x69, 0x18, 0x06, 0x12, 0xBA, 0xC8, 0xA7, 0x62, 0x6E, 0xB1, 0xEC, 0xA8, 0x6A, 0x81, 0x57, 0x5E, \
0xCE, 0x6B, 0xF6, 0xE2, 0x8F, 0x65, 0x52, 0x1B, 0x4B, 0xE7, 0xC5, 0x61, 0x2D, 0x2D, 0x22, 0x83, 0x30, 0x98, 0xD7, 0x3B, \
0x9A, 0xCB, 0xCD, 0xB7, 0xEB, 0xCF, 0xF5, 0x3A, 0x55, 0x65, 0x8D, 0xCE, 0x10, 0x99, 0x57, 0x0F, 0x4B, 0x90, 0xB4, 0x7D, \
0x48, 0xB0, 0x32, 0xAC, 0xB0, 0x4A, 0x3D, 0x91, 0x19, 0x38, 0xE7, 0x6D

/*
// Command:                  RF_EZCONFIG_CHECK
// Description:              Validates the EZConfig array was written correctly.
*/
#define RF_EZCONFIG_CHECK 0x19, 0x6C, 0x71

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
        0x05, RF_INT_CTL_ENABLE_1, \
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
