/*! @file ZetaRF_config.h
 *
 * @brief Configuration file for ZetaRF library.
 */

#ifndef ZETARF_CONFIG_H_
#define ZETARF_CONFIG_H_

#define ZETARF_SPI_SETTINGS SPISettings(1000000UL, MSBFIRST, SPI_MODE0)

// Radio defines
#define ZETARF_CHANNEL_NUMBER   0
#define ZETARF_PACKET_LENGTH    10

#endif /* ZETARF_CONFIG_H_ */
