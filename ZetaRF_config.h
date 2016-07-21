/*! @file ZetaRF_config.h
 *
 * @brief Configuration file for ZetaRF library.
 */

#ifndef ZETARF_CONFIG_H_
#define ZETARF_CONFIG_H_

#define ZETARF_SPI_SETTINGS SPISettings(1000000UL, MSBFIRST, SPI_MODE0)

// Radio defines
#ifndef ZETARF_CHANNEL_NUMBER
#define ZETARF_CHANNEL_NUMBER   0
#endif

#ifndef ZETARF_PACKET_LENGTH
#define ZETARF_PACKET_LENGTH    8
#endif

#endif /* ZETARF_CONFIG_H_ */
