#pragma once

#include <stdint.h>

namespace ZetaRFHal {

namespace Pin {

    template<int PinNumber>
    struct PinSelector {
        static constexpr int pin = PinNumber;
    };

    struct PinBase {};

    template<int PinNumber>
    struct IO : public PinBase, public PinSelector<PinNumber> {};

} // namespace Pin

struct ChipSelectPinSelector {};
struct ShutdownPinSelector {};
struct IrqPinSelector {};
struct ClearToSendPinSelector {};

template<int PinNumber>
struct ChipSelectPin : public ChipSelectPinSelector, public Pin::PinSelector<PinNumber> {};
template<int PinNumber>
struct ShutdownPin : public ShutdownPinSelector, public Pin::PinSelector<PinNumber> {};
template<int PinNumber>
struct IrqPin : public IrqPinSelector, public Pin::PinSelector<PinNumber> {};
template<int PinNumber>
struct ClearToSendPin : public ClearToSendPinSelector, public Pin::PinSelector<PinNumber> {};

} // namespace ZetaRFHal

namespace ZetaRF {

template<int PinNumber>
using CS = ZetaRFHal::ChipSelectPin<PinNumber>;
template<int PinNumber>
using nSEL = ZetaRFHal::ChipSelectPin<PinNumber>;
template<int PinNumber>
using SDN = ZetaRFHal::ShutdownPin<PinNumber>;
template<int PinNumber>
using IRQ = ZetaRFHal::IrqPin<PinNumber>;
template<int PinNumber>
using nIRQ = ZetaRFHal::IrqPin<PinNumber>;
template<int PinNumber>
using CTS = ZetaRFHal::ClearToSendPin<PinNumber>;

} // namespace ZetaRF
