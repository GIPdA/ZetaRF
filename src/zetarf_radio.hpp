#pragma once

#include "flags/flags.hpp"

#include <stdint.h>

namespace ZetaRF {

enum class Event : uint32_t
{
    None                            = uint32_t(0),

    // Packet Handler
    PacketTransmitted               = uint32_t(1 << 1),
    PacketReceived                  = uint32_t(1 << 2),
    CrcError                        = uint32_t(1 << 3),
    TxFifoAlmostEmpty               = uint32_t(1 << 4),
    RxFifoAlmostFull                = uint32_t(1 << 5),

    // Modem Interrupt
    InvalidSync                     = uint32_t(1 << 6),
    InvalidPreamble                 = uint32_t(1 << 7),
    DetectedPreamble                = uint32_t(1 << 8),
    DetectedSync                    = uint32_t(1 << 9),
    LatchedRssi                     = uint32_t(1 << 10),

    // Chip Interrupt
    FifoUnderflowOrOverflowError    = uint32_t(1 << 11),
    CommandError                    = uint32_t(1 << 12),
    StateChange                     = uint32_t(1 << 13),
    ChipReady                       = uint32_t(1 << 14),

    DeviceBusy                      = uint32_t(1 << 15) // Communication error (CTS timeout)
};
using Events = flags::flags<Event>;

} // namespace ZetaRF

ALLOW_FLAGS_FOR_ENUM(ZetaRF::Event)

namespace ZetaRF {

constexpr Events const AllEvents {
              Event::PacketTransmitted | Event::PacketReceived
            | Event::CrcError
            | Event::TxFifoAlmostEmpty | Event::RxFifoAlmostFull
            | Event::InvalidSync | Event::InvalidPreamble | Event::DetectedPreamble | Event::DetectedSync
            | Event::LatchedRssi
            | Event::FifoUnderflowOrOverflowError
            | Event::CommandError
            | Event::StateChange | Event::ChipReady
            | Event::DeviceBusy
            };

} // namespace ZetaRF
