/*
 * EZRadio Commands
 */

#pragma once

#include "ezradio_si4455_defs.h"
#include "zetarf_radio.hpp"
#include "flags/flags.hpp"


namespace ZetaRfEZRadio {

template<class Hal>
class EZRadioSi4455
{
public:
    using Timeout = typename Hal::Timeout;

    using Event = ZetaRf::Event;
    using Events = ZetaRf::Events;
    using EZRadioReply = EZRadioReply_Si4455;

    enum class RadioState
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

    enum class StartTxOption_ReTransmit : uint8_t {
        SendDataFromTxFifo = 0, // Send data that has been written to the TX FIFO. If the TX FIFO is empty, a FIFO underflow interrupt will occur.
        SendLastPacket     = 1  // Send last packet again (do not write FIFO).
    };

    uint8_t _cast(RadioState state) {
        return static_cast<uint8_t>(state);
    }
    uint8_t _cast(StartTxOption_ReTransmit retransmit) {
        return static_cast<uint8_t>(retransmit);
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
        // TODO: add support for hardware CTS (see ezradiopro).
        // -> Need to add local comm functions to not depend on the Hal before
        // the chip is configured.

        //debugln("Initializing...");
        // While cycle as far as the pointer points to a command
        while (*configArray != 0x00) {
            // Commands structure in the array:
            // --------------------------------
            // LEN | <LEN length of data>

            uint8_t cmdBytesCount = *configArray++;

            if (cmdBytesCount > 16u) { // EZConfig
                // Initial configuration of Si4x55
                if (*configArray == SI4455_CMD_ID_WRITE_TX_FIFO) {
                    if (cmdBytesCount > 128u) {
                        // Number of command bytes exceeds maximal allowable length
                        // @todo May need to send NOP to send more than 128 bytes (check documentation)
                        //debugln("Init failed: Command bytes exceeds 128 bytes");
                        return false;
                    }

                    // Load array to the device
                    configArray++;
                    writeEZConfigArray(configArray, cmdBytesCount - 1);

                    // Point to the next command
                    configArray += cmdBytesCount - 1;

                    // Continue command interpreter
                    continue;
                } else {
                    // Number of command bytes exceeds maximal allowable length
                    //debugln("Init failed: Command bytes exceeds max");
                    return false;
                }
            }

            // Non-EZConfig command
            uint8_t radioCmd[16];
            for (uint8_t col = 0; col < cmdBytesCount; col++) {
                radioCmd[col] = *configArray;
                configArray++;
            }

            uint8_t response {0};
            if (!sendCommandAndReadResponse(radioCmd, cmdBytesCount, &response, 1)) {
                // Timeout occured
                //debugln("Init failed: Command timeout");
                return false;
            }

            // Check response byte for EZCONFIG_CHECK command
            if (radioCmd[0] == SI4455_CMD_ID_EZCONFIG_CHECK) {
                if (response != SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_VALID) {
                    // EZConfig failed, either SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_BAD_CHECKSUM or SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_INVALID_STATE
                    //debugln("Init failed: EZConfig Check error");
                    return false;
                }
            }

            if (m_hal.isIrqAsserted()) {
                // Get and clear all interrupts. An error has occured...
                _cleanCommandBuffer();
                EZRadioReply::InterruptStatus const& it = readAndClearInterruptStatus();
                if (it.CHIP_PEND & SI4455_CMD_GET_CHIP_STATUS_REP_CMD_ERROR_PEND_MASK) {
                    // Command error
                    //debugln("Init failed: Command Error");
                    return false;
                }
            }
        }

        // Extra configurations

        // Configure FRR
        // ! DO NOT CHANGE ! FRRs are used by the library.
        setProperties(0x02, // Group ID
                      0x00, // Start at index 0 (FRR A)
                      SI4455_PROP_FRR_CTL_A_MODE_FRR_A_MODE_ENUM_CURRENT_STATE,
                      SI4455_PROP_FRR_CTL_B_MODE_FRR_B_MODE_ENUM_LATCHED_RSSI,
                      SI4455_PROP_FRR_CTL_C_MODE_FRR_C_MODE_ENUM_INT_PH_PEND,
                      SI4455_PROP_FRR_CTL_D_MODE_FRR_D_MODE_ENUM_INT_CHIP_PEND);

        _cleanCommandBuffer();
        return true;
    }

    //! Writes data byte(s) to the EZConfig array (array generated from EZConfig tool).
    bool writeEZConfigArray(uint8_t const* ezConfigArray, uint8_t count) {
        return sendCommand(SI4455_CMD_ID_WRITE_TX_FIFO, ezConfigArray, count);
    }

    uint16_t readMaxRxPacketLength()
    {
        // Mainly a placeholder, not relevant for the Si4455.
        // Returns the max capacity for a variable length packet.
        return 63;
    }

    void setMaxRxPacketLength(uint16_t packetLength)
    {
        // Not needed for the Si4455.
        (void)packetLength;
    }

    //! Reads data byte(s) from the current position in RX FIFO. Returns false on error. Doesn't need CTS
    void readRxFifo(uint8_t* data, uint8_t length) {
        readCommandWithoutClearToSend(SI4455_CMD_ID_READ_RX_FIFO, data, length);
    }
    //! Writes data byte(s) to the TX FIFO. Doesn't need CTS
    void writeTxFifo(const uint8_t* data, uint8_t length) {
        sendCommandWithoutClearToSend(SI4455_CMD_ID_WRITE_TX_FIFO, data, length);
    }
    //! Writes data byte(s) to the TX FIFO. Doesn't need CTS
    void writeTxFifoWithZeros(uint8_t length) {
        sendZeroedCommandWithoutClearToSend(SI4455_CMD_ID_WRITE_TX_FIFO, length);
    }

    //! Switches to RX state and starts reception of a packet.
    void startRx(uint8_t channel, uint16_t length, uint8_t condition, uint8_t nextState1, uint8_t nextState2, uint8_t nextState3)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_START_RX,
            channel,
            condition,
            (uint8_t)(length >> 8),
            (uint8_t)(length),
            nextState1,
            nextState2,
            nextState3
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_START_RX);
    }

    void startRx(uint8_t channel, uint16_t length, uint8_t condition)
    {
        // No changes to next states, avoid sending the extra 3 bytes
        uint8_t const buffer[] = {
            SI4455_CMD_ID_START_RX,
            channel,
            condition,
            (uint8_t)(length >> 8),
            (uint8_t)(length)
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_START_RX-3);
    }

    void startRx(uint8_t channel, uint16_t length)
    {
        startRx(channel,
                length,
                0,
                SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_RX,
                SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX
                );
    }
    void startRxForSinglePacket(uint8_t channel, uint16_t length)
    {
        startRx(channel,
                length,
                0,
                SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_READY,
                SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX
                );
    }


    //! Switches to TX state and starts transmission of a packet.
    void startTx(uint8_t channel, uint16_t length,
                 RadioState stateWhenTxComplete,
                 StartTxOption_ReTransmit retransmit// = StartTxOption_ReTransmit::SendDataFromTxFifo,
                 )
    {
        const uint8_t buffer[] = {
            SI4455_CMD_ID_START_TX,
            channel,
            uint8_t(( (_cast(stateWhenTxComplete)&0x0F) << SI4455_CMD_START_TX_ARG_TXCOMPLETE_STATE_LSB)
                    | (_cast(retransmit) << SI4455_CMD_START_TX_ARG_RETRANSMIT_LSB)),
            uint8_t((length&0x1FFF) >> 8),
            uint8_t(length)
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_START_TX);
    }

    void startTx(uint8_t channel, uint16_t length,
                 RadioState stateWhenTxComplete)
    {
        startTx(channel, length,
                stateWhenTxComplete,
                StartTxOption_ReTransmit::SendDataFromTxFifo
                );
    }

    //! Start TX for packet retransmission
    void startTxRepeat(uint8_t channel, uint16_t length,
                       RadioState stateWhenTxComplete,
                       StartTxOption_ReTransmit retransmit)
    {
        startTx(channel, length,
                stateWhenTxComplete,
                retransmit);
    }

    void startVariableLengthTx(uint8_t channel, uint16_t length, RadioState stateWhenTxComplete)
    {
        (void)length; // Zero length RX for variable length
        startTx(channel, 0, stateWhenTxComplete);
    }


    EZRadioReply::DeviceState const& readDeviceState()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_REQUEST_DEVICE_STATE
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_REQUEST_DEVICE_STATE,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_REQUEST_DEVICE_STATE);
        return m_commandReply.REQUEST_DEVICE_STATE;

        // Si4455Cmd.REQUEST_DEVICE_STATE.CURR_STATE       = radioCmd[0];
        // Si4455Cmd.REQUEST_DEVICE_STATE.CURRENT_CHANNEL  = radioCmd[1];
    }

    //! Manually switch the chip to a desired operating state.
    void changeState(uint8_t nextState)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_CHANGE_STATE,
            nextState
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_CHANGE_STATE);
    }

    //! Returns the interrupt status of ALL the possible interrupt events (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the corresponding clear mask argument).
    EZRadioReply::InterruptStatus const& readInterruptStatus(uint8_t pendingPacketHandlerIntsClearMask, uint8_t pendingModemIntsClearMask, uint8_t pendingChipIntsClearMask)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_INT_STATUS,
            pendingPacketHandlerIntsClearMask,
            pendingModemIntsClearMask,
            pendingChipIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_INT_STATUS,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_INT_STATUS);

        return m_commandReply.GET_INT_STATUS;

        // m_commandReply.GET_INT_STATUS.INT_PEND       = radioCmd[0];
        // m_commandReply.GET_INT_STATUS.INT_STATUS     = radioCmd[1];
        // m_commandReply.GET_INT_STATUS.PH_PEND        = radioCmd[2];
        // m_commandReply.GET_INT_STATUS.PH_STATUS      = radioCmd[3];
        // m_commandReply.GET_INT_STATUS.MODEM_PEND     = radioCmd[4];
        // m_commandReply.GET_INT_STATUS.MODEM_STATUS   = radioCmd[5];
        // m_commandReply.GET_INT_STATUS.CHIP_PEND      = radioCmd[6];
        // m_commandReply.GET_INT_STATUS.CHIP_STATUS    = radioCmd[7];
    }

    EZRadioReply::InterruptStatus const& readAndClearInterruptStatus() {
        return readInterruptStatus(0,0,0);
    }

    void clearAllPendingInterrupts()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_INT_STATUS,
            /*0x00, // Don't send parameters = IT clear
            0x00,
            0x00//*/
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_GET_INT_STATUS-3);
    }

    void clearPendingInterrupts(uint8_t pendingPacketHandlerIntsClearMask, uint8_t pendingModemIntsClearMask, uint8_t pendingChipIntsClearMask)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_INT_STATUS,
            pendingPacketHandlerIntsClearMask,
            pendingModemIntsClearMask,
            pendingChipIntsClearMask
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_GET_INT_STATUS);
    }

    //!  Reads the fast response registers (FRR) starting with FRR_A.
    EZRadioReply::FrrA const& readFrrA(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI4455_CMD_ID_FRR_A_READ,
                                    m_commandReply.RAW,
                                    registersToRead);
        return m_commandReply.FRR_A_READ;

        // Si4455Cmd.FRR_A_READ.FRR_A_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_A_READ.FRR_B_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_A_READ.FRR_C_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_A_READ.FRR_D_VALUE = radioCmd[3];
    }

    //! Reads the fast response registers (FRR) starting with FRR_B.
    EZRadioReply::FrrB const& readFrrB(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI4455_CMD_ID_FRR_B_READ,
                                    m_commandReply.RAW,
                                    registersToRead);
        return m_commandReply.FRR_B_READ;

        // Si4455Cmd.FRR_B_READ.FRR_B_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_B_READ.FRR_C_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_B_READ.FRR_D_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_B_READ.FRR_A_VALUE = radioCmd[3];
    }
    //! Reads the fast response registers (FRR) starting with FRR_C.
    EZRadioReply::FrrC const& readFrrC(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI4455_CMD_ID_FRR_C_READ,
                                    m_commandReply.RAW,
                                    registersToRead);
        return m_commandReply.FRR_C_READ;

        // Si4455Cmd.FRR_C_READ.FRR_C_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_C_READ.FRR_D_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_C_READ.FRR_A_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_C_READ.FRR_B_VALUE = radioCmd[3];
    }

    //! Reads the fast response registers (FRR) starting with FRR_D.
    EZRadioReply::FrrD const& readFrrD(uint8_t registersToRead = 1)
    {
        readCommandWithoutClearToSend(SI4455_CMD_ID_FRR_D_READ,
                                    m_commandReply.RAW,
                                    registersToRead);
        return m_commandReply.FRR_D_READ;

        // Si4455Cmd.FRR_D_READ.FRR_D_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_D_READ.FRR_A_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_D_READ.FRR_B_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_D_READ.FRR_C_VALUE = radioCmd[3];
    }

    void noOperation()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_NOP
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_NOP);
    }

    void resetRxAndTxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x03
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    void resetRxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x02
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    void resetTxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x01
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    //! Access the current byte counts in the TX and RX FIFOs, and provide for resetting the FIFOs (see doc).
    EZRadioReply::FifoInfo const& readFifoInfo(uint8_t fifoResetMask = 0)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            fifoResetMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_FIFO_INFO);
        return m_commandReply.FIFO_INFO;

        // Si4455Cmd.FIFO_INFO.RX_FIFO_COUNT   = radioCmd[0];
        // Si4455Cmd.FIFO_INFO.TX_FIFO_SPACE   = radioCmd[1];
    }

    //! Returns information about the length of the variable field in the last packet received.
    EZRadioReply::PacketInfo const& readPacketInfo()
    {
        return readPacketInfo(0, 0, 0);
    }

    //! Returns information about the length of the variable field in the last packet received, and (optionally) overrides field length.
    EZRadioReply::PacketInfo const& readPacketInfo(uint8_t fieldNum, uint16_t length, uint16_t lenDiff)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_PACKET_INFO,
            (uint8_t)(fieldNum & 0x1F),
            (uint8_t)(length >> 8),
            (uint8_t)(length),
            (uint8_t)(lenDiff >> 8),
            (uint8_t)(lenDiff)
        };

        sendCommandAndReadResponse(buffer, (fieldNum == 0 ? 1 : 6),
                                m_commandReply.RAW, 2);
        return m_commandReply.PACKET_INFO;
    }


    //! Used to read CTS and the command response.
    EZRadioReply::CommandBuffer const& readCommandBuffer()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_READ_CMD_BUFF
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_READ_CMD_BUFF,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_READ_CMD_BUFF);
        return m_commandReply.READ_CMD_BUFF;

        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF0   = radioCmd[0];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF1   = radioCmd[1];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF2   = radioCmd[2];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF3   = radioCmd[3];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF4   = radioCmd[4];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF5   = radioCmd[5];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF6   = radioCmd[6];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF7   = radioCmd[7];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF8   = radioCmd[8];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF9   = radioCmd[9];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF10  = radioCmd[10];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF11  = radioCmd[11];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF12  = radioCmd[12];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF13  = radioCmd[13];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF14  = radioCmd[14];
        // Si4455Cmd.READ_CMD_BUFF.CMD_BUFF15  = radioCmd[15];
    }


    //! Retrieves the value of one or more properties.
    EZRadioReply::Properties const& readProperties(uint8_t group, uint8_t startPropertyIndex, uint8_t propertyCount)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_PROPERTY,
            group,
            propertyCount,
            startPropertyIndex
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_PROPERTY,
                                   m_commandReply.RAW, propertyCount);
        return m_commandReply.GET_PROPERTY;

        // Si4455Cmd.GET_PROPERTY.DATA0    = radioCmd[0];
        // Si4455Cmd.GET_PROPERTY.DATA1    = radioCmd[1];
        // Si4455Cmd.GET_PROPERTY.DATA2    = radioCmd[2];
        // Si4455Cmd.GET_PROPERTY.DATA3    = radioCmd[3];
        // Si4455Cmd.GET_PROPERTY.DATA4    = radioCmd[4];
        // Si4455Cmd.GET_PROPERTY.DATA5    = radioCmd[5];
        // Si4455Cmd.GET_PROPERTY.DATA6    = radioCmd[6];
        // Si4455Cmd.GET_PROPERTY.DATA7    = radioCmd[7];
        // Si4455Cmd.GET_PROPERTY.DATA8    = radioCmd[8];
        // Si4455Cmd.GET_PROPERTY.DATA9    = radioCmd[9];
        // Si4455Cmd.GET_PROPERTY.DATA10   = radioCmd[10];
        // Si4455Cmd.GET_PROPERTY.DATA11   = radioCmd[11];
        // Si4455Cmd.GET_PROPERTY.DATA12   = radioCmd[12];
        // Si4455Cmd.GET_PROPERTY.DATA13   = radioCmd[13];
        // Si4455Cmd.GET_PROPERTY.DATA14   = radioCmd[14];
        // Si4455Cmd.GET_PROPERTY.DATA15   = radioCmd[15];
    }

    //! Sets the value of one or more properties. Max property count is 12.
    void setProperties_n(uint8_t propertyGroup, uint8_t startPropertyIndex, uint8_t propertyCount,
                                uint8_t data1, uint8_t data2=0, uint8_t data3=0, uint8_t data4=0, uint8_t data5=0, uint8_t data6=0,
                                uint8_t data7=0, uint8_t data8=0, uint8_t data9=0, uint8_t data10=0, uint8_t data11=0, uint8_t data12=0)
    {
        const uint8_t buffer[] = {    // No more than 12 properties allowed
                SI4455_CMD_ID_SET_PROPERTY,
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
            SI4455_CMD_ID_SET_PROPERTY,
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
            SI4455_CMD_ID_GET_PH_STATUS,
            pendingPacketHandlerIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_PH_STATUS,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_PH_STATUS);
        return m_commandReply.GET_PH_STATUS;

        // Si4455Cmd.GET_PH_STATUS.PH_PEND        = radioCmd[0];
        // Si4455Cmd.GET_PH_STATUS.PH_STATUS      = radioCmd[1];
    }

    //! Returns the interrupt status of the Modem Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    EZRadioReply::ModemStatus const& readModemStatus(uint8_t pendingModemIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_MODEM_STATUS,
            pendingModemIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_MODEM_STATUS,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_MODEM_STATUS);
        return m_commandReply.GET_MODEM_STATUS;

        // Si4455Cmd.GET_MODEM_STATUS.MODEM_PEND   = radioCmd[0];
        // Si4455Cmd.GET_MODEM_STATUS.MODEM_STATUS = radioCmd[1];
        // Si4455Cmd.GET_MODEM_STATUS.CURR_RSSI    = radioCmd[2];
        // Si4455Cmd.GET_MODEM_STATUS.LATCH_RSSI   = radioCmd[3];
        // Si4455Cmd.GET_MODEM_STATUS.ANT1_RSSI    = radioCmd[4];
        // Si4455Cmd.GET_MODEM_STATUS.ANT2_RSSI    = radioCmd[5];
        // Si4455Cmd.GET_MODEM_STATUS.AFC_FREQ_OFFSET.U8[MSB]  = radioCmd[6];
        // Si4455Cmd.GET_MODEM_STATUS.AFC_FREQ_OFFSET.U8[LSB]  = radioCmd[7];
    }

    //! Returns the interrupt status of the Chip Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    EZRadioReply::ChipStatus const& readChipStatus(uint8_t pendingChipIntsClearMask = 0xFF)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_CHIP_STATUS,
            pendingChipIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_CHIP_STATUS,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_CHIP_STATUS);
        return m_commandReply.GET_CHIP_STATUS;

        // Si4455Cmd.GET_CHIP_STATUS.CHIP_PEND         = radioCmd[0];
        // Si4455Cmd.GET_CHIP_STATUS.CHIP_STATUS       = radioCmd[1];
        // Si4455Cmd.GET_CHIP_STATUS.CMD_ERR_STATUS    = radioCmd[2];
    }


    EZRadioReply::GpioPinConfig const& configureGpioPins(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3,
                                                            uint8_t nirq, uint8_t sdo,
                                                            uint8_t genConfig)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GPIO_PIN_CFG,
            gpio0,
            gpio1,
            gpio2,
            gpio3,
            nirq,
            sdo,
            genConfig
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GPIO_PIN_CFG,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GPIO_PIN_CFG);
        return m_commandReply.GPIO_PIN_CFG;

        // m_commandReply.GPIO_PIN_CFG.GPIO0        = radioCmd[0];
        // m_commandReply.GPIO_PIN_CFG.GPIO1        = radioCmd[1];
        // m_commandReply.GPIO_PIN_CFG.GPIO2        = radioCmd[2];
        // m_commandReply.GPIO_PIN_CFG.GPIO3        = radioCmd[3];
        // m_commandReply.GPIO_PIN_CFG.NIRQ         = radioCmd[4];
        // m_commandReply.GPIO_PIN_CFG.SDO          = radioCmd[5];
        // m_commandReply.GPIO_PIN_CFG.GEN_CONFIG   = radioCmd[6];
    }

    //! Performs conversions using the Auxiliary ADC and returns the results of those conversions.
    EZRadioReply::AdcReadings const& readAdc(uint8_t adcEnable, uint8_t adcConfig)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_ADC_READING,
            adcEnable,
            adcConfig
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_ADC_READING,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_ADC_READING);
        return m_commandReply.GET_ADC_READING;

        // Si4455Cmd.GET_ADC_READING.GPIO_ADC.U8[MSB]      = radioCmd[0];
        // Si4455Cmd.GET_ADC_READING.GPIO_ADC.U8[LSB]      = radioCmd[1];
        // Si4455Cmd.GET_ADC_READING.BATTERY_ADC.U8[MSB]   = radioCmd[2];
        // Si4455Cmd.GET_ADC_READING.BATTERY_ADC.U8[LSB]   = radioCmd[3];
        // Si4455Cmd.GET_ADC_READING.TEMP_ADC.U8[MSB]      = radioCmd[4];
        // Si4455Cmd.GET_ADC_READING.TEMP_ADC.U8[LSB]      = radioCmd[5];
        // Si4455Cmd.GET_ADC_READING.TEMP_SLOPE            = radioCmd[6];
        // Si4455Cmd.GET_ADC_READING.TEMP_INTERCEPT        = radioCmd[7];
    }


    EZRadioReply::PartInfo const& readPartInformation()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_PART_INFO
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_PART_INFO,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_PART_INFO);
        return m_commandReply.PART_INFO;

        // Si4455Cmd.PART_INFO.CHIPREV         = radioCmd[0];
        // Si4455Cmd.PART_INFO.PART.U8[MSB]    = radioCmd[1];
        // Si4455Cmd.PART_INFO.PART.U8[LSB]    = radioCmd[2];
        // Si4455Cmd.PART_INFO.PBUILD          = radioCmd[3];
        // Si4455Cmd.PART_INFO.ID.U8[MSB]      = radioCmd[4];
        // Si4455Cmd.PART_INFO.ID.U8[LSB]      = radioCmd[5];
        // Si4455Cmd.PART_INFO.CUSTOMER        = radioCmd[6];
        // Si4455Cmd.PART_INFO.ROMID           = radioCmd[7];
        // Si4455Cmd.PART_INFO.BOND            = radioCmd[8];
    }

    EZRadioReply::FuncInfo const& readFunctionRevisionInformation()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FUNC_INFO
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_FUNC_INFO,
                                m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_FUNC_INFO);
        return m_commandReply.FUNC_INFO;

        // Si4455Cmd.FUNC_INFO.REVEXT          = radioCmd[0];
        // Si4455Cmd.FUNC_INFO.REVBRANCH       = radioCmd[1];
        // Si4455Cmd.FUNC_INFO.REVINT          = radioCmd[2];
        // Si4455Cmd.FUNC_INFO.PATCH.U8[MSB]   = radioCmd[3];
        // Si4455Cmd.FUNC_INFO.PATCH.U8[LSB]   = radioCmd[4];
        // Si4455Cmd.FUNC_INFO.FUNC            = radioCmd[5];
        // Si4455Cmd.FUNC_INFO.SVNFLAGS        = radioCmd[6];
        // Si4455Cmd.FUNC_INFO.SVNREV.U8[b3]   = radioCmd[7];
        // Si4455Cmd.FUNC_INFO.SVNREV.U8[b2]   = radioCmd[8];
        // Si4455Cmd.FUNC_INFO.SVNREV.U8[b1]   = radioCmd[9];
        // Si4455Cmd.FUNC_INFO.SVNREV.U8[b0]   = radioCmd[10];
    }


    //! Waits for CTS to be high. Returns false if timeout occured.
    bool waitForClearToSend(unsigned long timeout_ms = 300)
    {
        if (m_hal.waitForClearToSend(SI4455_CMD_ID_READ_CMD_BUFF, timeout_ms)) {
            m_deviceBusy = false;
            return true;
        }
        m_deviceBusy = true;
        return false;
    }


    static Events processPacketHandlerInterruptPending(uint8_t phPend)
    {
        Events it;

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT)
            it |= (Event::PacketTransmitted);

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT)
            it |= Event::PacketReceived;

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_CRC_ERROR_PEND_BIT)
            it |= (Event::CrcError);

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_TX_FIFO_ALMOST_EMPTY_PEND_BIT)
            it |= (Event::TxFifoAlmostEmpty);

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_RX_FIFO_ALMOST_FULL_PEND_BIT)
            it |= (Event::RxFifoAlmostFull);

        return it;
    }

    static Events processModemInterruptPending(uint8_t modemPend)
    {
        Events it;

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_INVALID_SYNC_PEND_BIT)
            it |= (Event::InvalidSync);

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_INVALID_PREAMBLE_PEND_BIT)
            it |= (Event::InvalidPreamble);

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_PREAMBLE_DETECT_PEND_BIT)
            it |= (Event::DetectedPreamble);

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_SYNC_DETECT_PEND_BIT)
            it |= (Event::DetectedSync);

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_RSSI_PEND_BIT)
            it |= (Event::LatchedRssi);

        return it;
    }

    static Events processChipInterruptPending(uint8_t chipPend)
    {
        Events it;

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT)
            it |= (Event::FifoUnderflowOrOverflowError);

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_CMD_ERROR_PEND_BIT)
            it |= (Event::CommandError);

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_STATE_CHANGE_PEND_BIT)
            it |= (Event::StateChange);

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_CHIP_READY_PEND_BIT)
            it |= Event::ChipReady;

        return it;
    }

    Events readInterrupts()
    {
        EZRadioReply::InterruptStatus const& is { readAndClearInterruptStatus() };

        if (failed())
            return Event::DeviceBusy;

        Events ev;
        ev |= processPacketHandlerInterruptPending(is.PH_PEND);
        ev |= processModemInterruptPending(is.MODEM_PEND);
        ev |= processChipInterruptPending(is.CHIP_PEND);

        /*debug("IT: ");
        debug(" PH x");
        debug(is.PH_PEND, HEX);
        debug(" MODEM  x");
        debug(is.MODEM_PEND, HEX);
        debug(" CHIP x");
        debug(is.CHIP_PEND, HEX);

        debug(" EVs b");
        debug(uint8_t(ev.underlying_value()>>0)&0xF, BIN);
        debug(" ");
        debug(uint8_t(ev.underlying_value()>>4)&0xF, BIN);
        debug(" ");
        debug(uint8_t(ev.underlying_value()>>8)&0xF, BIN);
        debug(" ");
        debug(uint8_t(ev.underlying_value()>>12)&0xF, BIN);
        debugln();//*/

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

    //writeDataWithoutClearToSend
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

        m_hal.resumeOrBeginSpiTransaction(SI4455_CMD_ID_READ_CMD_BUFF);
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

} // namespace ZetaRfEZRadio
