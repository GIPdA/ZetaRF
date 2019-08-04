/*! @file ZetaRF.cpp
 *
 * @brief This file contains the public functions controlling the Si4455 radio chip.
 *
 * http://www.silabs.com/products/wireless/EZRadio/Pages/Si4455.aspx
 *
 * License: see LICENSE file
 */

#include <Arduino.h>

#include "ZetaRF.h"


constexpr uint16_t milliseconds(uint16_t value)
{
    return value;
}


bool ZetaRF::begin(uint8_t const* configArray)
{
    return m_radio.beginWithConfigurationDataArray(configArray);
}
