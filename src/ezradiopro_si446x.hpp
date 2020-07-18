/*
 * EZRadio Commands
 */

#pragma once

#include "ezradiopro_si446x_defs.h"
#include "zetarf_radio.hpp"
#include "flags/flags.hpp"

#include <string.h> // memset

namespace ZetaRfEZRadioPro {

template<class Hal>
class EZRadioProSi446x
{
public:
    using Timeout = typename Hal::Timeout;

    using Event = ZetaRf::Event;
    using Events = ZetaRf::Events;
    using EZRadioReply = EZRadioReply_Si446x;

    enum class RadioState : uint8_t
    { // From EZRadio API
        Invalid     = 0,
        Sleep       = 1,
        SpiActive   = 2,
        Ready       = 3,
        Ready2      = 4,
        TxTune      = 5,
        RxTune      = 6,
        Tx          = 7,
        Rx          = 8,
        Unknown     = 9
    };

    /*static const char* radioStateText(RadioState state) {
        switch (state) {
            case RadioState::Sleep: return "Sleep";
            case RadioState::SpiActive: return "SpiActive";
            case RadioState::Ready: return "Ready";
            case RadioState::Ready2: return "Ready2";
            case RadioState::TxTune: return "TxTune";
            case RadioState::RxTune: return "RxTune";
            case RadioState::Tx: return "Tx";
            case RadioState::Rx: return "Rx";

            default: return "Invalid";
        }
    }//*/
    /*static constexpr RadioState radioStateFromValue(uint8_t state) {
        switch (state) {
            case 1: return RadioState::Sleep;
            case 2: return RadioState::SpiActive;
            case 3: return RadioState::Ready;
            case 4: return RadioState::Ready2;
            case 5: return RadioState::TxTune;
            case 6: return RadioState::RxTune;
            case 7: return RadioState::Tx;
            case 8: return RadioState::Rx;

            default: return RadioState::Invalid;
        }
    }//*/

    enum class StartTxOption_Update : uint8_t {
        UseTxParametersNow     = 0, // Use TX parameters to enter TX mode.
        UpdateTxParametersOnly = 1  // Update TX parameters (to be used by a subsequent packet) but do not enter TX mode.
    };
    enum class StartTxOption_ReTransmit : uint8_t {
        SendDataFromTxFifo = 0, // Send data that has been written to the TX FIFO. If the TX FIFO is empty, a FIFO underflow interrupt will occur.
        SendLastPacket     = 1  // Send last packet again (do not write FIFO).
    };
    enum class StartTxOption_Start : uint8_t {
        Immediate               = 0, // Start TX immediately.
        OnWakeUpTimerExpiration = 1  // Start TX upon expiration of the Wake-Up Timer.
    };

    enum class StartRxOption_Start : uint8_t {
        Immediate               = 0, // Start TX immediately.
        OnWakeUpTimerExpiration = 1  // Start TX upon expiration of the Wake-Up Timer.
    };

    uint8_t _cast(RadioState state) {
        return static_cast<uint8_t>(state);
    }
    uint8_t _cast(StartTxOption_Update update) {
        return static_cast<uint8_t>(update);
    }
    uint8_t _cast(StartTxOption_ReTransmit retransmit) {
        return static_cast<uint8_t>(retransmit);
    }
    uint8_t _cast(StartTxOption_Start start) {
        return static_cast<uint8_t>(start);
    }
    uint8_t _cast(StartRxOption_Start start) {
        return static_cast<uint8_t>(start);
    }

    bool failed() const {
        return m_deviceBusy;
    }
    bool succeeded() const {
        return !failed();
    }

    bool isDeviceBusy() const {
        return m_deviceBusy;
    }

    void holdInReset() {
        m_hal.putInShutdown();
    }
    void releaseFromReset() {
        m_hal.releaseFromShutdown();
    }

    bool isIrqAsserted() {
        return m_hal.isIrqAsserted();
    }

    bool initialize() {
        return m_hal.initialize();
    }
    void deinitialize() {
        m_hal.deinitialize();
    }

    //! Load all properties and commands with a list of NULL terminated commands.
    //! Call @reset before.
    bool loadConfigurationArray(uint8_t const* configArray)
    {
        // Local comm functions to avoid config-specific CTS settings like
        // hardware CTS (via GPIO) that will not be set until this config is done.
        auto _waitForCTS = [&]() {
            auto const t = Hal::Timeout::make(300);
            uint8_t ctsVal {0};
            do {
                m_hal.restartOrBeginSpiTransaction();
                m_hal.spiWriteByte(SI446X_CMD_ID_READ_CMD_BUFF);
                ctsVal = m_hal.spiReadByte();

                if (t.expired()) {
                    m_hal.endSpiTransaction();
                    return false;
                }
            } while (ctsVal != 0xFF); // Clear to send when 0xFF

            // Hold SPI transaction
            return true;
        };

        auto _sendCommand = [&](uint8_t const* data, uint8_t dataByteCount)
        {
            if (!_waitForCTS())
                return false;
            m_hal.restartOrBeginSpiTransaction();
            m_hal.spiWriteData(data, dataByteCount);
            m_hal.endSpiTransaction();
            return true;
        };

        auto commandErrorOccurred = [&]() {
            uint8_t const buffer[] = {
                SI446X_CMD_ID_GET_INT_STATUS, 0, 0, 0
            };

            if (!_sendCommand(buffer, SI446X_CMD_ARG_COUNT_GET_INT_STATUS))
                return true;

            // Wait until radio IC is ready with the data
            if (!_waitForCTS())
                return true;

            m_hal.resumeOrBeginSpiTransaction(SI446X_CMD_ID_READ_CMD_BUFF);
            m_hal.spiReadData(m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_INT_STATUS);
            m_hal.endSpiTransaction();

            EZRadioReply::InterruptStatus const& it = m_commandReply.GET_INT_STATUS;
            if (it.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_MASK)
                return true;

            return false;
        };


        //debugln("Initializing...");
        // While cycle as far as the pointer points to a command
        while (*configArray != 0x00) {
            // Commands structure in the array:
            // --------------------------------
            // LEN | <LEN length of data>

            uint8_t cmdBytesCount = *configArray++;

            if (cmdBytesCount > 16u) {
                // Number of command bytes exceeds maximal allowable length
                //debugln("Init failed: Command bytes exceeds max");
                return false;
            }

            // Non-EZConfig command
            uint8_t radioCmd[16];
            for (uint8_t col = 0; col < cmdBytesCount; col++) {
                radioCmd[col] = *configArray;
                configArray++;
            }

            if (!_sendCommand(radioCmd, cmdBytesCount)) {
                // Timeout occured
                //debugln("Init failed: Command timeout");
                return false;
            }

            if (m_hal.isIrqAsserted()) {
                // Get and clear all interrupts. Maybe an error has occured...
                if (commandErrorOccurred()) {
                    //debugln("Init failed: Command Error");
                    return false;
                }
            }
        }

        // Extra configurations
        {
            // The EzRadioPro API use packet fields to configure packets, with fields being
            //  able to be the size of a following variable length field (FVL).
            //  e.g. Field 1 is variable length, Field 2 is varying in length from Field 1.
            // But to send a variable packet, we need to set the FVL length property
            //  to the size we want to send (e.g. PKT_FIELD_2_LENGTH).
            // To receive a variable packet, the FVL length property need to be set to
            //  the maximum packet size we can expect.

            // WDS configures the fields only in "shared mode", TX and RX use the same fields config.
            // That would mean having to reconfigure the field length at every TX/RX switch.
            // To avoid that, we enable the "field split mode" option and copy shared field options to RX.
            // That way RX can always receive the max packet length configured in the config array
            //  without doing anything.

            // First we detect the variable length option, then copy packet fields 1 and 2 to
            // the RX packet field properties. We only use 2 fields.

            // Read PKT_CONFIG1 (0x06) to PKT_LEN (0x08) property (one empty byte in the middle)
            auto const& p = readProperties(0x12, 0x06, 3);
            uint8_t PKT_CONFIG1 = p.DATA[0]; // 0x06
            uint8_t const PKT_LEN = p.DATA[2]; // 0x08

            // Safety check
            if (failed() || PKT_CONFIG1 == 0xff || PKT_LEN == 0xff)
                return false;

            // PKT_LEN.DST_FIELD sets which packet field is a variable length field.
            // Valid values are 2, 3, 4 and 5, otherwise it uses fixed length packets.
            uint8_t const PKT_LEN_DST_FIELD = PKT_LEN&0b111;
            if (PKT_LEN_DST_FIELD >= 2 && PKT_LEN_DST_FIELD <= 5) {
                // Variable length mode is enabled
                //debugln("Enabling Split mode");
                // Copy PKT_FIELD 1 and 2 to PKT_RX_FIELD 1 and 2
                auto const& p = readProperties(0x12, 0x0d, 8); // PKT_FIELD_1_LENGTH to PKT_FIELD_2_CRC_CONFIG
                setProperties(0x12, 0x21, // PKT_RX_FIELD_1_LENGTH to PKT_RX_FIELD_2_CRC_CONFIG
                              p.DATA[0], p.DATA[1], p.DATA[2], p.DATA[3],
                              p.DATA[4], p.DATA[5], p.DATA[6], p.DATA[7]);

                // Enable PH_FIELD_SPLIT to split TX and RX fields config
                PKT_CONFIG1 |= 0x80; // Set PH_FIELD_SPLIT
                setProperties(0x12, 0x06, PKT_CONFIG1); // PKT_CONFIG1

                if (failed())
                    return false;
            }

            // Be sure no command error happened
            if (m_hal.isIrqAsserted()) {
                // Get and clear all interrupts. Maybe an error has occured...
                if (commandErrorOccurred()) {
                    //debugln("Init failed: Command Error");
                    return false;
                }
            }
        }


        // Configure FRR
        // ! DO NOT CHANGE ! FRRs are used by the library.
        setProperties(0x02, // Group ID
                      0x00, // Start at index 0 (FRR A)
                      SI446X_PROP_FRR_CTL_A_MODE_FRR_A_MODE_ENUM_CURRENT_STATE,
                      SI446X_PROP_FRR_CTL_B_MODE_FRR_B_MODE_ENUM_LATCHED_RSSI,
                      SI446X_PROP_FRR_CTL_C_MODE_FRR_C_MODE_ENUM_INT_PH_PEND,
                      SI446X_PROP_FRR_CTL_D_MODE_FRR_D_MODE_ENUM_INT_CHIP_PEND);

        _cleanCommandBuffer();
        return true;
    }


    uint16_t readMaxRxPacketLength()
    {
        auto const& p = readProperties(0x12, 0x25, 2); // PKT_RX_FIELD_2_LENGTH
        return succeeded() ? (uint16_t(p.DATA[0]<<8) | uint16_t(p.DATA[1]))&0x1FFF : 0;
    }

    void setMaxRxPacketLength(uint16_t packetLength)
    {
        setProperties(0x12, 0x25, // PKT_RX_FIELD_2_LENGTH
                      (packetLength>>8)&0x1F, packetLength&0xFF);
    }

    //! This function is used to initialize after power-up the radio chip.
    //! Before this function reset should be called.
    void powerUp(uint8_t bootOptions, uint8_t xtalOptions, uint32_t xOFreq)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_POWER_UP,
            bootOptions,
            xtalOptions,
            (uint8_t)(xOFreq >> 24),
            (uint8_t)(xOFreq >> 16),
            (uint8_t)(xOFreq >> 8),
            (uint8_t)(xOFreq >> 0),
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_POWER_UP);
    }

    //! Reads data byte(s) from the current position in RX FIFO. Returns false on error. Doesn't need CTS
    void readRxFifo(uint8_t* data, uint8_t length) {
        readCommandWithoutClearToSend(SI446X_CMD_ID_READ_RX_FIFO, data, length);
    }
    //! Writes data byte(s) to the TX FIFO. Doesn't need CTS
    void writeTxFifo(const uint8_t* data, uint8_t length) {
        sendCommandWithoutClearToSend(SI446X_CMD_ID_WRITE_TX_FIFO, data, length);
    }
    //! Writes data byte(s) to the TX FIFO. Doesn't need CTS
    void writeTxFifoWithZeros(uint8_t length) {
        sendZeroedCommandWithoutClearToSend(SI446X_CMD_ID_WRITE_TX_FIFO, length);
    }


    //! Switches to RX state and starts reception of a packet.
    void startRx(uint8_t channel, uint16_t length, uint8_t condition, uint8_t nextState1, uint8_t nextState2, uint8_t nextState3)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_START_RX,
            channel,
            condition,
            (uint8_t)(length >> 8),
            (uint8_t)(length),
            nextState1,
            nextState2,
            nextState3
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_START_RX);
    }

    void startRx(uint8_t channel, uint16_t length, uint8_t condition)
    {
        // No changes to next states, avoid sending the extra 3 bytes
        uint8_t const buffer[] = {
            SI446X_CMD_ID_START_RX,
            channel,
            condition,
            (uint8_t)(length >> 8),
            (uint8_t)(length)
        };

        sendCommand(buffer, 5);//SI446X_CMD_ARG_COUNT_START_RX);
    }

    void startRxWithPreviousParameters()
    {
        // Use the last settings
        uint8_t const buffer[] = {
            SI446X_CMD_ID_START_RX
        };

        sendCommand(buffer, 1);
    }

    void startRx(uint8_t channel, uint16_t length, StartRxOption_Start start = StartRxOption_Start::Immediate)
    {
        startRx(channel,
                length,
                (_cast(start) << SI446X_CMD_START_RX_ARG_CONDITION_START_ENUM_IMMEDIATE),
                SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_RX,
                SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX
                );
    }
    void startRxForSinglePacket(uint8_t channel, uint16_t length, StartRxOption_Start start = StartRxOption_Start::Immediate)
    {
        startRx(channel,
                length,
                (_cast(start) << SI446X_CMD_START_RX_ARG_CONDITION_START_ENUM_IMMEDIATE),
                SI446X_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
                SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX
                );
    }


    //! Switches to TX state and starts transmission of a packet.
    void startTx(uint8_t channel, uint16_t length,
                 RadioState stateWhenTxComplete,
                 StartTxOption_Update update,// = StartTxOption_Update::UseTxParametersNow,
                 StartTxOption_Start start,// = StartTxOption_Start::Immediate,
                 StartTxOption_ReTransmit retransmit,// = StartTxOption_ReTransmit::SendDataFromTxFifo,
                 uint8_t delayBetweenRetransmitPackets_us = 0,
                 uint8_t retransmitRepeatCount = 0)
    {
        const uint8_t buffer[] = {
            SI446X_CMD_ID_START_TX,
            channel,
            uint8_t(( (_cast(stateWhenTxComplete)&0x0F) << SI446X_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_LSB)
                    | (_cast(update) << SI446X_CMD_START_TX_ARG_CONDITION_UPDATE_LSB)
                    | (_cast(retransmit) << SI446X_CMD_START_TX_ARG_CONDITION_RETRANSMIT_LSB)
                    | (_cast(start) << SI446X_CMD_START_TX_ARG_CONDITION_START_ENUM_IMMEDIATE)
                    ),
            uint8_t((length&0x1FFF) >> 8),
            uint8_t(length&0xFF),
            // last options not in B1 revision
            delayBetweenRetransmitPackets_us, // Delay (in usec) between packet retransmissions
            retransmitRepeatCount  // The number of times to repeat the packet.
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_START_TX);
    }

    void startTx(uint8_t channel, uint16_t length,
                 RadioState stateWhenTxComplete)
    {
        startTx(channel, length,
                stateWhenTxComplete,
                StartTxOption_Update::UseTxParametersNow,
                StartTxOption_Start::Immediate,
                StartTxOption_ReTransmit::SendDataFromTxFifo
                );
    }

    //! Start TX for packet retransmission
    void startTxRepeat(uint8_t channel, uint16_t length,
                       RadioState stateWhenTxComplete,
                       StartTxOption_ReTransmit retransmit,
                       uint8_t delayBetweenPacket_us,
                       uint8_t repeatCount)
    {
        startTx(channel, length,
                stateWhenTxComplete,
                StartTxOption_Update::UseTxParametersNow,
                StartTxOption_Start::Immediate,
                retransmit,
                delayBetweenPacket_us,
                repeatCount
                );
    }

    void startVariableLengthTx(uint8_t channel, uint16_t length, RadioState stateWhenTxComplete)
    {
        setProperties(0x12, SI446X_PROP_PKT_FIELD_2_LENGTH_FIELD_2_LENGTH_12_8_INDEX,
                      (length>>8)&0x1F, length&0xFF); // PKT_FIELD_2_LENGTH
        startTx(channel, 0, stateWhenTxComplete);
    }


    EZRadioReply::DeviceState const& readDeviceState()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_REQUEST_DEVICE_STATE
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_REQUEST_DEVICE_STATE,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_REQUEST_DEVICE_STATE);
        return m_commandReply.REQUEST_DEVICE_STATE;

        // Si446xCmd.REQUEST_DEVICE_STATE.CURR_STATE       = radioCmd[0];
        // Si446xCmd.REQUEST_DEVICE_STATE.CURRENT_CHANNEL  = radioCmd[1];
    }

    //! Manually switch the chip to a desired operating state.
    void changeState(uint8_t nextState)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_CHANGE_STATE,
            nextState
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_CHANGE_STATE);
    }


    //! While in RX state this will hop to the frequency specified by the parameters and start searching for a preamble.
    void rxHop(uint8_t inte, uint8_t frac2, uint8_t frac1, uint8_t frac0, uint8_t vc0_cnt1, uint8_t vc0_cnt0)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_RX_HOP,
            inte,
            frac2,
            frac1,
            frac0,
            vc0_cnt1,
            vc0_cnt0
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_RX_HOP);
    }

    //! While in TX state this will hop to the frequency specified by the parameters.
    void txHop(uint8_t inte, uint8_t frac2, uint8_t frac1, uint8_t frac0, uint8_t vc0_cnt1, uint8_t vc0_cnt0,
               uint8_t pll_settle_time1, uint8_t pll_settle_time0)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_TX_HOP,
            inte,
            frac2,
            frac1,
            frac0,
            vc0_cnt1,
            vc0_cnt0,
            pll_settle_time1,
            pll_settle_time0
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_TX_HOP);
    }


    //! Returns the interrupt status of ALL the possible interrupt events (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the corresponding clear mask argument).
    EZRadioReply::InterruptStatus const& readInterruptStatus(uint8_t pendingPacketHandlerIntsClearMask,
                                                             uint8_t pendingModemIntsClearMask,
                                                             uint8_t pendingChipIntsClearMask)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_INT_STATUS,
            pendingPacketHandlerIntsClearMask,
            pendingModemIntsClearMask,
            pendingChipIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_INT_STATUS,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_INT_STATUS);

        return m_commandReply.GET_INT_STATUS;
    }

    EZRadioReply::InterruptStatus const& readAndClearInterruptStatus() {
        return readInterruptStatus(0,0,0);
    }

    void clearAllPendingInterrupts()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_INT_STATUS,
            /*0x00, // Don't send parameters = IT clear
            0x00,
            0x00//*/
        };

        sendCommand(buffer, 1);
    }

    void clearPendingInterrupts(uint8_t pendingPacketHandlerIntsClearMask, uint8_t pendingModemIntsClearMask, uint8_t pendingChipIntsClearMask)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_INT_STATUS,
            pendingPacketHandlerIntsClearMask,
            pendingModemIntsClearMask,
            pendingChipIntsClearMask
        };

        sendCommand(buffer, SI446X_CMD_REPLY_COUNT_GET_INT_STATUS);
    }

    //!  Reads the fast response registers (FRR) starting with FRR_A.
    EZRadioReply::FrrA const& readFrrA(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI446X_CMD_ID_FRR_A_READ,
                                      m_commandReply.RAW,
                                      registersToRead);
        return m_commandReply.FRR_A_READ;
    }
    //! Reads the fast response registers (FRR) starting with FRR_B.
    EZRadioReply::FrrB const& readFrrB(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI446X_CMD_ID_FRR_B_READ,
                                      m_commandReply.RAW,
                                      registersToRead);
        return m_commandReply.FRR_B_READ;
    }
    //! Reads the fast response registers (FRR) starting with FRR_C.
    EZRadioReply::FrrC const& readFrrC(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI446X_CMD_ID_FRR_C_READ,
                                      m_commandReply.RAW,
                                      registersToRead);
        return m_commandReply.FRR_C_READ;
    }
    //! Reads the fast response registers (FRR) starting with FRR_D.
    EZRadioReply::FrrD const& readFrrD(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI446X_CMD_ID_FRR_D_READ,
                                      m_commandReply.RAW,
                                      registersToRead);
        return m_commandReply.FRR_D_READ;
    }

    //! Sends NOP command. Can be used to maintain SPI communication.
    void noOperation()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_NOP
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_NOP);
    }

    void resetRxAndTxFifo()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FIFO_INFO,
            0x03
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_FIFO_INFO);
    }

    void resetRxFifo()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FIFO_INFO,
            0x02
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_FIFO_INFO);
    }

    void resetTxFifo()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FIFO_INFO,
            0x01
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_FIFO_INFO);
    }

    //! Access the current byte counts in the TX and RX FIFOs, and provide for resetting the FIFOs (see doc).
    EZRadioReply::FifoInfo const& readFifoInfo()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FIFO_INFO
        };

        sendCommandAndReadResponse(buffer, 1,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_FIFO_INFO);
        return m_commandReply.FIFO_INFO;

        // m_commandReply.FIFO_INFO.RX_FIFO_COUNT   = radioCmd[0];
        // m_commandReply.FIFO_INFO.TX_FIFO_SPACE   = radioCmd[1];
    }
    EZRadioReply::FifoInfo const& readFifoInfo(uint8_t fifoResetMask)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FIFO_INFO,
            fifoResetMask
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_FIFO_INFO,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_FIFO_INFO);
        return m_commandReply.FIFO_INFO;

        // m_commandReply.FIFO_INFO.RX_FIFO_COUNT   = radioCmd[0];
        // m_commandReply.FIFO_INFO.TX_FIFO_SPACE   = radioCmd[1];
    }

    //! Returns information about the length of the variable field in the last packet received.
    EZRadioReply::PacketInfo const& readPacketInfo()
    {
        return readPacketInfo(0, 0, 0);
    }

    //! Returns information about the length of the variable field in the last packet received, and (optionally) overrides field length.
    EZRadioReply::PacketInfo const& readPacketInfo(uint8_t fieldNum, uint16_t length, int16_t lenDiff)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_PACKET_INFO,
            (uint8_t)(fieldNum & 0x1F),
            (uint8_t)(length >> 8),
            (uint8_t)(length),
            (uint8_t)(uint16_t(lenDiff) >> 8),
            (uint8_t)(lenDiff)
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_PACKET_INFO,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_PACKET_INFO);
        return m_commandReply.PACKET_INFO;
    }


    //! Used to read CTS and the command response.
    EZRadioReply::CommandBuffer const& readCommandBuffer()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_READ_CMD_BUFF
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_READ_CMD_BUFF,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_READ_CMD_BUFF);
        return m_commandReply.READ_CMD_BUFF;
    }


    //! Retrieves the value of one or more properties.
    EZRadioReply::Properties const& readProperties(uint8_t group, uint8_t startPropertyIndex, uint8_t propertyCount)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_PROPERTY,
            group,
            propertyCount,
            startPropertyIndex
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_PROPERTY,
                                   m_commandReply.RAW, propertyCount);
        return m_commandReply.GET_PROPERTY;
    }

    //! Sets the value of one or more properties. Max property count is 12.
    void setProperties_n(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t propertyCount,
                                uint8_t data1, uint8_t data2=0, uint8_t data3=0, uint8_t data4=0, uint8_t data5=0, uint8_t data6=0,
                                uint8_t data7=0, uint8_t data8=0, uint8_t data9=0, uint8_t data10=0, uint8_t data11=0, uint8_t data12=0)
    {
        const uint8_t buffer[] = {    // No more than 12 properties allowed
                SI446X_CMD_ID_SET_PROPERTY,
                propertyGroup,
                propertyCount,
                startPropertyIndex,
                data1, data2, data3, data4, data5, data6,
                data7, data8, data9, data10, data11, data12
            };

        sendCommand(buffer, 4+propertyCount);
    }

    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1) {
        setProperties_n(propertyGroup, startPropertyIndex, 1, data1);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2) {
        setProperties_n(propertyGroup, startPropertyIndex, 2, data1, data2);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3) {
        setProperties_n(propertyGroup, startPropertyIndex, 3, data1, data2, data3);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4) {
        setProperties_n(propertyGroup, startPropertyIndex, 4, data1, data2, data3, data4);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5) {
        setProperties_n(propertyGroup, startPropertyIndex, 5, data1, data2, data3, data4, data5);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6) {
        setProperties_n(propertyGroup, startPropertyIndex, 6, data1, data2, data3, data4, data5, data6);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7) {
        setProperties_n(propertyGroup, startPropertyIndex, 7, data1, data2, data3, data4, data5, data6, data7);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8) {
        setProperties_n(propertyGroup, startPropertyIndex, 8, data1, data2, data3, data4, data5, data6, data7, data8);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9) {
        setProperties_n(propertyGroup, startPropertyIndex, 9, data1, data2, data3, data4, data5, data6, data7, data8, data9);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10) {
        setProperties_n(propertyGroup, startPropertyIndex, 10, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11) {
        setProperties_n(propertyGroup, startPropertyIndex, 11, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11);
    }
    void setProperties(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11, uint8_t data12) {
        setProperties_n(propertyGroup, startPropertyIndex, 12, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12);
    }

    /*void setProperties(uint8_t group, uint8_t count, uint8_t propertyIndex, uint8_t data, ...)
    {
        va_list argList;

        uint8_t buffer[16] = {    // No more than 12 properties allowed
            SI446X_CMD_ID_SET_PROPERTY,
            group,
            count,
            propertyIndex,
            data
        };

        va_start(argList, data);
        uint8_t cmdIndex = 5;
        while (count--) {
            buffer[cmdIndex] = (uint8_t)(va_arg(argList, uint8_t));
            cmdIndex++;
            if (cmdIndex == 16)
                break;
        }
        va_end(argList);

        sendCommand(buffer, cmdIndex);
    }//*/


    //! Returns the interrupt status of the Packet Handler Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    EZRadioReply::PhStatus const& readPacketHandlerStatus(uint8_t pendingPacketHandlerIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_PH_STATUS,
            pendingPacketHandlerIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_PH_STATUS,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_PH_STATUS);
        return m_commandReply.GET_PH_STATUS;
    }
    EZRadioReply::PhStatus const& readAndClearPacketHandlerStatus()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_PH_STATUS
        };

        sendCommandAndReadResponse(buffer, 1,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_PH_STATUS);
        return m_commandReply.GET_PH_STATUS;
    }
    void clearPacketHandlerStatus()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_PH_STATUS
        };
        sendCommand(buffer, 1);
    }

    //! Returns the interrupt status of the Modem Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    EZRadioReply::ModemStatus const& readModemStatus(uint8_t pendingModemIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_MODEM_STATUS,
            pendingModemIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_MODEM_STATUS,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_MODEM_STATUS);
        return m_commandReply.GET_MODEM_STATUS;
    }
    EZRadioReply::ModemStatus const& readAndClearModemStatus()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_MODEM_STATUS
        };

        sendCommandAndReadResponse(buffer, 1,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_MODEM_STATUS);
        return m_commandReply.GET_MODEM_STATUS;
    }
    void clearModemStatus()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_MODEM_STATUS
        };
        sendCommand(buffer, 1);
    }

    //! Returns the interrupt status of the Chip Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    EZRadioReply::ChipStatus const& readChipStatus(uint8_t pendingChipIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_CHIP_STATUS,
            pendingChipIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_CHIP_STATUS,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_CHIP_STATUS);
        return m_commandReply.GET_CHIP_STATUS;
    }
    EZRadioReply::ChipStatus const& readAndClearChipStatus()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_CHIP_STATUS
        };

        sendCommandAndReadResponse(buffer, 1,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_CHIP_STATUS);
        return m_commandReply.GET_CHIP_STATUS;
    }
    void clearChipStatus(uint8_t pendingChipIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_CHIP_STATUS,
            pendingChipIntsClearMask
        };
        sendCommand(buffer, SI446X_CMD_ARG_COUNT_GET_CHIP_STATUS);
    }


    EZRadioReply::GpioPinConfig const& readGpioPinsConfiguration()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GPIO_PIN_CFG
        };

        sendCommandAndReadResponse(buffer, 1,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GPIO_PIN_CFG);
        return m_commandReply.GPIO_PIN_CFG;
    }

    EZRadioReply::GpioPinConfig const& configureGpioPins(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3,
                                                         uint8_t nirq, uint8_t sdo,
                                                         uint8_t genConfig)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GPIO_PIN_CFG,
            gpio0,
            gpio1,
            gpio2,
            gpio3,
            nirq,
            sdo,
            genConfig
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GPIO_PIN_CFG,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GPIO_PIN_CFG);
        return m_commandReply.GPIO_PIN_CFG;
    }

    //! Performs conversions using the Auxiliary ADC and returns the results of those conversions.
    EZRadioReply::AdcReadings const& readAdc(uint8_t adcEnable, uint8_t adcConfig)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_GET_ADC_READING,
            adcEnable,
            adcConfig
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_GET_ADC_READING,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_GET_ADC_READING);
        return m_commandReply.GET_ADC_READING;
    }


    EZRadioReply::PartInfo const& readPartInformation()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_PART_INFO
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_PART_INFO,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_PART_INFO);
        return m_commandReply.PART_INFO;
    }

    EZRadioReply::FuncInfo const& readFunctionRevisionInformation()
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_FUNC_INFO
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_FUNC_INFO,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_FUNC_INFO);
        return m_commandReply.FUNC_INFO;
    }


    //! Image rejection calibration. Forces a specific value for IR calibration, and reads back calibration values from previous calibrations.
    EZRadioReply::IRCal const& imageRejectionCalibration(uint8_t ircal_amp, uint8_t ircal_ph)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_IRCAL_MANUAL,
            ircal_amp,
            ircal_ph
        };

        sendCommandAndReadResponse(buffer, SI446X_CMD_ARG_COUNT_IRCAL_MANUAL,
                                   m_commandReply.RAW, SI446X_CMD_REPLY_COUNT_IRCAL_MANUAL);
        return m_commandReply.IRCAL_MANUAL;
    }

    //! Performs image rejection calibration. Completion can be monitored by polling CTS or waiting for CHIP_READY interrupt source.
    void performImageRejectionCalibration(uint8_t searching_step_size, uint8_t searching_rssi_avg,
                                          uint8_t rx_chain_setting1, uint8_t rx_chain_setting2)
    {
        uint8_t const buffer[] = {
            SI446X_CMD_ID_IRCAL,
            searching_step_size,
            searching_rssi_avg,
            rx_chain_setting1,
            rx_chain_setting2
        };

        sendCommand(buffer, SI446X_CMD_ARG_COUNT_IRCAL);
    }



    //! Waits for CTS to be high. Returns false if timeout occured.
    bool waitForClearToSend(unsigned long timeout_ms = 300)
    {
        if (m_hal.waitForClearToSend(SI446X_CMD_ID_READ_CMD_BUFF, timeout_ms)) {
            m_deviceBusy = false;
            return true;
        }
        m_deviceBusy = true;
        return false;
    }


    static Events processPacketHandlerInterruptPending(uint8_t phPend)
    {
        Events it;

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
            it |= (Event::PacketTransmitted);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_RX_PEND_BIT)
            it |= Event::PacketReceived;

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_CRC_ERROR_PEND_BIT)
            it |= (Event::CrcError);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_ALT_CRC_ERROR_PEND_BIT)
            it |= (Event::AlternateCrcError);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT)
            it |= (Event::TxFifoAlmostEmpty);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_RX_FIFO_ALMOST_FULL_PEND_BIT)
            it |= (Event::RxFifoAlmostFull);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_FILTER_MATCH_PEND_BIT)
            it |= (Event::FilterMatch);

        if (phPend & SI446X_CMD_GET_INT_STATUS_REP_PH_PEND_FILTER_MISS_PEND_BIT)
            it |= (Event::FilterMiss);

        return it;
    }

    static Events processModemInterruptPending(uint8_t modemPend)
    {
        Events it;

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_INVALID_SYNC_PEND_BIT)
            it |= (Event::InvalidSync);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_INVALID_PREAMBLE_PEND_BIT)
            it |= (Event::InvalidPreamble);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_PREAMBLE_DETECT_PEND_BIT)
            it |= (Event::DetectedPreamble);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_SYNC_DETECT_PEND_BIT)
            it |= (Event::DetectedSync);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_RSSI_LATCH_PEND_BIT)
            it |= (Event::LatchedRssi);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_POSTAMBLE_DETECT_PEND_BIT)
            it |= (Event::DetectedPostamble);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_RSSI_JUMP_PEND_BIT)
            it |= (Event::RssiJump);

        if (modemPend & SI446X_CMD_GET_INT_STATUS_REP_MODEM_PEND_RSSI_PEND_BIT)
            it |= (Event::RssiThreshold);

        return it;
    }

    static Events processChipInterruptPending(uint8_t chipPend)
    {
        Events it;

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT)
            it |= (Event::FifoUnderflowOrOverflowError);

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_BIT)
            it |= (Event::CommandError);

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_STATE_CHANGE_PEND_BIT)
            it |= (Event::StateChange);

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_CHIP_READY_PEND_BIT)
            it |= Event::ChipReady;

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_CAL_PEND_BIT)
            it |= Event::Calibration;

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_LOW_BATT_PEND_BIT)
            it |= Event::LowBattery;

        if (chipPend & SI446X_CMD_GET_INT_STATUS_REP_CHIP_PEND_WUT_PEND_BIT)
            it |= Event::WakeUpTimerExpired;

        return it;
    }

    Events readInterrupts()
    {
        EZRadioReply::InterruptStatus const& is { readAndClearInterruptStatus() };

        //Serial.print("> PH_PEND    "); Serial.println(is.PH_PEND);
        //Serial.print("> MODEM_PEND "); Serial.println(is.MODEM_PEND);
        //Serial.print("> CHIP_PEND  "); Serial.println(is.CHIP_PEND);

        if (failed())
            return Event::DeviceBusy;

        Events ev;
        ev |= processPacketHandlerInterruptPending(is.PH_PEND);
        ev |= processModemInterruptPending(is.MODEM_PEND);
        ev |= processChipInterruptPending(is.CHIP_PEND);

        return ev;
    }

private:
    // Returns false if CTS wait timeout'd
    bool sendCommand(uint8_t const* data, uint8_t dataByteCount)
    {
        if (!waitForClearToSend())
            return false;

        m_hal.restartOrBeginSpiTransaction();
        m_hal.spiWriteData(data, dataByteCount);
        m_hal.endSpiTransaction();
        return true;
    }

    // Returns false if CTS wait timeout'd
    bool sendCommand(uint8_t command, uint8_t const* data, uint8_t dataByteCount)
    {
        if (!waitForClearToSend())
            return false;

        return sendCommandWithoutClearToSend(command, data, dataByteCount);
    }

    bool sendCommandWithoutClearToSend(uint8_t command, uint8_t const* data, uint8_t dataByteCount)
    {
        m_hal.restartOrBeginSpiTransaction();
        m_hal.spiWriteByte(command);
        m_hal.spiWriteData(data, dataByteCount);
        m_hal.endSpiTransaction();
        return true;
    }

    bool sendZeroedCommandWithoutClearToSend(uint8_t command, uint8_t zeroBytesCount)
    {
        m_hal.restartOrBeginSpiTransaction();
        m_hal.spiWriteByte(command);
        while (zeroBytesCount--)
            m_hal.spiWriteByte(0);
        m_hal.endSpiTransaction();
        return true;
    }

    // Returns false if CTS wait timeout'd
    bool sendCommandAndReadResponse(uint8_t const* commandData, uint8_t commandDataByteCount, uint8_t* responseData, uint8_t responseDataByteCount)
    {
        if (!sendCommand(commandData, commandDataByteCount))
            return false;

        // Wait until radio IC is ready with the data
        if (!waitForClearToSend())
            return false;

        m_hal.resumeOrBeginSpiTransaction(SI446X_CMD_ID_READ_CMD_BUFF);
        m_hal.spiReadData(responseData, responseDataByteCount);
        m_hal.endSpiTransaction();
        return true;
    }

    bool readCommandWithoutClearToSend(uint8_t command, uint8_t* data, uint8_t dataByteCount)
    {
        m_hal.restartOrBeginSpiTransaction();
        m_hal.spiWriteByte(command);
        m_hal.spiReadData(data, dataByteCount);
        m_hal.endSpiTransaction();
        return true;
    }

    // Returns false if CTS wait timeout'd
    bool readCommand(uint8_t command, uint8_t* data, uint8_t dataByteCount)
    {
        if (!waitForClearToSend())
            return false;

        return readCommandWithoutClearToSend(command, data, dataByteCount);
    }

    // Write zeros in command buffer
    void _cleanCommandBuffer() {
        memset(m_commandReply.RAW, 42, 16);
    }

    Hal m_hal;
    EZRadioReply::CommandReplyUnion m_commandReply;
    bool m_deviceBusy {false};
};

} // namespace ZetaRfEZRadioPro
