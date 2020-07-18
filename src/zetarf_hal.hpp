#pragma once

#include <stdint.h>

namespace ZetaRfHal {

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

} // namespace ZetaRfHal

namespace ZetaRf {

template<int PinNumber>
using CS = ZetaRfHal::ChipSelectPin<PinNumber>;
template<int PinNumber>
using nSEL = ZetaRfHal::ChipSelectPin<PinNumber>;
template<int PinNumber>
using SDN = ZetaRfHal::ShutdownPin<PinNumber>;
template<int PinNumber>
using IRQ = ZetaRfHal::IrqPin<PinNumber>;
template<int PinNumber>
using nIRQ = ZetaRfHal::IrqPin<PinNumber>;
template<int PinNumber>
using CTS = ZetaRfHal::ClearToSendPin<PinNumber>;

} // namespace ZetaRf
