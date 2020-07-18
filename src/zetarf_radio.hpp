#pragma once

#include "flags/flags.hpp"

#include <stdint.h>

namespace ZetaRf {

enum class Event : uint32_t
{
    None                            = uint32_t(0),

    // Packet Handler
    PacketTransmitted               = uint32_t(1UL << 1),
    PacketReceived                  = uint32_t(1UL << 2),
    CrcError                        = uint32_t(1UL << 3),
    TxFifoAlmostEmpty               = uint32_t(1UL << 4),
    RxFifoAlmostFull                = uint32_t(1UL << 5),
    FilterMatch                     = uint32_t(1UL << 6), // EZRadioPro
    FilterMiss                      = uint32_t(1UL << 7), // EZRadioPro
    AlternateCrcError               = uint32_t(1UL << 8), // EZRadioPro

    // Modem Interrupt
    InvalidSync                     = uint32_t(1UL << 9),
    InvalidPreamble                 = uint32_t(1UL << 10),
    DetectedPreamble                = uint32_t(1UL << 11),
    DetectedSync                    = uint32_t(1UL << 12),
    LatchedRssi                     = uint32_t(1UL << 13),
    DetectedPostamble               = uint32_t(1UL << 14), // EZRadioPro
    RssiJump                        = uint32_t(1UL << 15), // EZRadioPro
    RssiThreshold                   = uint32_t(1UL << 16), // EZRadioPro

    // Chip Interrupt
    FifoUnderflowOrOverflowError    = uint32_t(1UL << 17),
    CommandError                    = uint32_t(1UL << 18),
    StateChange                     = uint32_t(1UL << 19),
    ChipReady                       = uint32_t(1UL << 20),
    Calibration                     = uint32_t(1UL << 21), // EZRadioPro
    LowBattery                      = uint32_t(1UL << 22), // EZRadioPro
    WakeUpTimerExpired              = uint32_t(1UL << 23), // EZRadioPro

    DeviceBusy                      = uint32_t(1UL << 31) // Communication error (CTS timeout)
};
using Events = flags::flags<Event>;

} // namespace ZetaRf

ALLOW_FLAGS_FOR_ENUM(ZetaRf::Event)

namespace ZetaRf {

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

} // namespace ZetaRf
