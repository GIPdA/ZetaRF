/*!
 * @brief Interface controlling the Si4455 radio chip.
 *
 * License: see LICENSE file
 */

#pragma once

#include "si4455_defs.h"

#include <stdint.h>
#include <stdarg.h>

//#define ZETARF_DEBUG_ON

#if defined(ZETARF_DEBUG_ON)
    #define debug(...)   Serial.print(__VA_ARGS__)
    #define debugln(...) Serial.println(__VA_ARGS__)
#else
    #define debug(...)
    #define debugln(...)
#endif

// Pretty names for reply packets
typedef si4455_reply_FIFO_INFO_map      Si4455_FifoInfo;
typedef si4455_reply_GPIO_PIN_CFG_map   Si4455_GpioPinConfig;
typedef si4455_reply_GET_INT_STATUS_map Si4455_InterruptStatus;
typedef si4455_reply_PART_INFO_map      Si4455_PartInfo;
typedef si4455_reply_GET_PROPERTY_map   Si4455_Properties;
typedef si4455_reply_FUNC_INFO_map      Si4455_FuncInfo;

typedef si4455_reply_FRR_A_READ_map     Si4455_FrrA;
typedef si4455_reply_FRR_B_READ_map     Si4455_FrrB;
typedef si4455_reply_FRR_C_READ_map     Si4455_FrrC;
typedef si4455_reply_FRR_D_READ_map     Si4455_FrrD;

typedef si4455_reply_REQUEST_DEVICE_STATE_map   Si4455_DeviceState;
typedef si4455_reply_READ_CMD_BUFF_map          Si4455_CommandBuffer;
typedef si4455_reply_GET_ADC_READING_map        Si4455_AdcReadings;

typedef si4455_reply_GET_PH_STATUS_map      Si4455_PhStatus;
typedef si4455_reply_GET_MODEM_STATUS_map   Si4455_ModemStatus;
typedef si4455_reply_GET_CHIP_STATUS_map    Si4455_ChipStatus;

typedef si4455_reply_PACKET_INFO_map    Si4455_PacketInfo;


class ZetaRFRadio
{
public:
    enum class RadioState
    {
        Invalid = 0,
        Sleep,
        SpiActive,
        Ready,
        Ready2,
        TxTune,
        RxTune,
        Tx,
        Rx
    };

    enum class Status
    {
        NoStatus = 0,

        // Packet Handler
        DataTransmitted,
        DataAvailable,
        CrcError,
        FifoAlmostEmpty,
        FifoAlmostFull,

        // Modem Interrupt
        InvalidSync,
        InvalidPreamble,
        DetectedPreamble,
        DetectedSync,
        LatchedRssi,

        // Chip Interrupt
        FifoUnderflowOrOverflowError,
        CommandError,

        DeviceBusy
    };

    enum class ReadPacketResult
    {
        InvalidArgument,
        NotEnoughDataInFifo,
        PacketSizeLargerThanBuffer, // Variable length
        InvalidPacketSize, // Variable length
        RequestFailed,
        Success
    };
};

constexpr ZetaRFRadio::Status operator |(ZetaRFRadio::Status left, ZetaRFRadio::Status right) noexcept
{
    return static_cast<ZetaRFRadio::Status>(static_cast<uint32_t>(left) | static_cast<uint32_t>(right));
}
inline ZetaRFRadio::Status operator |=(ZetaRFRadio::Status& left, ZetaRFRadio::Status right) noexcept
{
    left = left | right;
    return left;
}
constexpr ZetaRFRadio::Status operator &(ZetaRFRadio::Status left, ZetaRFRadio::Status right) noexcept
{
  return static_cast<ZetaRFRadio::Status>(static_cast<uint32_t>(left) & static_cast<uint32_t>(right));
}
inline ZetaRFRadio::Status operator &=(ZetaRFRadio::Status& left, ZetaRFRadio::Status right) noexcept
{
    left = left & right;
    return left;
}
constexpr ZetaRFRadio::Status operator ^(ZetaRFRadio::Status left, ZetaRFRadio::Status right) noexcept
{
  return static_cast<ZetaRFRadio::Status>(static_cast<uint32_t>(left) ^ static_cast<uint32_t>(right));
}
constexpr ZetaRFRadio::Status operator ~(ZetaRFRadio::Status e) noexcept
{
    return static_cast<ZetaRFRadio::Status>(~static_cast<uint32_t>(e));
}
constexpr bool is_null(ZetaRFRadio::Status e) noexcept
{
    return static_cast<uint32_t>(e) == 0;
}

/*!
 * ZetaRF Radio Device Interface
 */
template<class Hal>
class ZetaRFRadioImpl : public ZetaRFRadio
{
    Hal hal;

public:
    using RadioState = ZetaRFRadio::RadioState;
    using Status = ZetaRFRadio::Status;
    using ReadPacketResult = ZetaRFRadio::ReadPacketResult;

    //! Current device status
    Status status() const {
        return m_deviceStatus;
    }

    bool statusHasError() const {
        return !is_null(m_deviceStatus & (Status::DeviceBusy | Status::CommandError));
    }
    bool statusNoError() const {
        return !statusHasError();
    }


    //! Begin the library
    bool beginWithConfigurationDataArray(uint8_t const* configArray)
    {
        hal.initialize();

        powerUp();

        int retryCount = 10;
        // Load radio configuration
        while (!initialize(configArray) && (retryCount--)) {
            // Wait and retry
            delay(20);
            powerUp();
        }

        if (retryCount <= 0)
            return false;

        cmd_clearAllPendingInterrupts();

        // Configure FRR
        cmd_setProperties(0x02, // Group ID
                          4,    // 4 registers to set
                          0x00, // Start at index 0 (FRR A)
                          SI4455_PROP_FRR_CTL_A_MODE_FRR_A_MODE_ENUM_CURRENT_STATE,
                          SI4455_PROP_FRR_CTL_B_MODE_FRR_B_MODE_ENUM_INT_PH_PEND,
                          SI4455_PROP_FRR_CTL_C_MODE_FRR_C_MODE_ENUM_INT_MODEM_PEND,
                          SI4455_PROP_FRR_CTL_D_MODE_FRR_D_MODE_ENUM_INT_CHIP_PEND);

        if (lastCommandFailed())
            return false;

        clearStatus();
        updateStatus();
        return statusNoError();
    }


    bool update()
    {
        if (hal.isIrqAsserted()) {
            updateStatus();
            return true;
        }
        return false;
    }


    /*!
     * Send data to @a channel.
     * Notes when using variable length packets:
     *  - First data byte must be the payload length field, and its value must be the data length (ignoring this extra byte).
     *  - e.g. To send 5 bytes of data, @a data buffer must contains: [5, data1, ... , data5] and @a length must be 6.
     * @sa sendVariableLengthPacket
     *
     * @param channel Channel to send data to.
     * @param data Pointer to data to send, first byte should be the payload length.
     * @param length Data length (bytes). Includes payload length byte when using variable length packets.
     * @param timeout_ms Max delay in ms to wait for the device to be ready to send a packet.
     */
    bool sendFixedLengthPacket(uint8_t channel, uint8_t const* data, uint8_t dataSize, unsigned long timeout_ms = 100)
    {
        // TODO: handle retransmit feature
        if (!data || dataSize == 0)
            return false;

        cmd_clearAllPendingInterrupts(); // MAYBE: needed?

        if (statusHasError())
            return false;

        if (!waitReadyToSendPacket(timeout_ms))
            return false;

        // Fill the TX fifo with data
        cmd_writeTxFifo(data, min(m_packetLength, dataSize));

        // Fill remaining with data
        if (m_packetLength > dataSize)
            cmd_writeTxFifoWithZeros(m_packetLength-dataSize);

        // Start sending packet on channel, return to RX after transmit
        cmd_startTx(channel, 0x80, m_packetLength);

        return statusNoError();
    }

    /*!
     * Convenience method to send variable length packets without the need to handle the extra payload field.
     * Do not include any extra payload byte or account for it in @a length, it is handled internally.
     *
     * @param channel Channel to send data to.
     * @param data Pointer to data to send, first byte should not be the payload length.
     * @param length Data length (bytes). Do not include the payload length byte.
     * @param timeout_ms Max delay in ms to wait for the device to be ready to send a packet.
     */
    bool sendVariableLengthPacket(uint8_t channel, uint8_t const* data, uint8_t length, unsigned long timeout_ms = 100)
    {
        if (!data || length == 0)
            return false;

        cmd_clearAllPendingInterrupts(); // MAYBE: needed?

        if (statusHasError())
            return false;

        if (!waitReadyToSendPacket(timeout_ms))
            return false;

        // Write the varible length field
        cmd_writeTxFifo(&length, 1);

        // Fill the TX fifo with data
        cmd_writeTxFifo(data, length);

        // Start sending packet on channel, return to RX after transmit
        cmd_startTx(channel, 0x80, length+1);

        return statusNoError();
    }


    /*!
     * Read packet from Rx FIFO.
     */
    ReadPacketResult readFixedLengthPacket(uint8_t* data, uint8_t byteCount)
    {
        if (!data)
            return ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        Si4455_FifoInfo const& fi = cmd_readFifoInfo();

        if (lastCommandFailed())
            return ReadPacketResult::RequestFailed;

        return readPacket(fi, data, byteCount);
    }

    /*!
     * Read variable length packet from Rx FIFO.
     */
    ReadPacketResult readVariableLengthPacket(uint8_t* data, uint8_t maxByteCount, uint8_t& packetDataLength)
    {
        if (!data)
            return ReadPacketResult::InvalidArgument;

        // Read FIFO info to known how many bytes are pending
        Si4455_FifoInfo fi = cmd_readFifoInfo(); // Make a copy

        if (fi.RX_FIFO_COUNT < 1) {
            debugln("Read VL Packet: Not Enough Data In Fifo");
            return ReadPacketResult::NotEnoughDataInFifo;
        }

        // Read size
        fi.RX_FIFO_COUNT--; // Remove variable length field
        cmd_readRxFifo(&packetDataLength, 1);

        debug("Read VL Packet of size: ");
        debugln(packetDataLength);

        if (packetDataLength <= 1) {
            cmd_resetRxFifo();
            return ReadPacketResult::InvalidPacketSize;
        }

        if (packetDataLength > maxByteCount)
            return ReadPacketResult::PacketSizeLargerThanBuffer;

        return readPacket(fi, data, packetDataLength);
    }


    //! Current internal radio state
    RadioState radioState() {
        return static_cast<RadioState>(cmd_readFrrA().FRR_A_VALUE & 0x0F);
    }

    uint8_t packetLength() const {
        return m_packetLength;
    }
    void setPacketLength(uint8_t newLength) {
        m_packetLength = newLength;
    }

    uint8_t listeningChannel() const {
        return m_listeningChannel;
    }

    uint8_t askCurrentChannel()
    {
        Si4455_DeviceState const& ds {cmd_requestDeviceState()};
        return ds.CURRENT_CHANNEL;
    }


    bool startListeningOnChannel(uint8_t newChannel)
    {
        if (!startListening(newChannel, m_packetLength))
            return false;

        m_listeningChannel = askCurrentChannel();
        return statusNoError() && (m_listeningChannel == newChannel);
    }

    bool startListeningSinglePacketOnChannel(uint8_t newChannel)
    {
        if (!startListeningSinglePacket(newChannel, m_packetLength))
            return false;

        m_listeningChannel = askCurrentChannel();
        return statusNoError() && (m_listeningChannel == newChannel);
    }

    bool restartListening()
    {
        return startListening(m_listeningChannel, m_packetLength);
    }


    //! Checks if the last transmission succeeded.
    bool checkTransmitted()
    {
        return testForStatusAndClear(Status::DataTransmitted);
    }

    //! Checks if an incoming message was received.
    bool checkReceived()
    {
        return testForStatusAndClear(Status::DataAvailable);
    }

    //! Returns true if TX FIFO is almost empty (clears the flag).
    bool isTxFifoAlmostEmpty()
    {
        return testForStatusAndClear(Status::FifoAlmostEmpty);
    }

    //! Returns true if RX FIFO is almost full (clears the flag).
    bool isRxFifoAlmostFull()
    {
        return testForStatusAndClear(Status::FifoAlmostFull);
    }


    /*!
     * Check the current state of the module, returns false if it is not responding correctly.
     * It actually reads the device state and check if the value is correct or not.
     *
     * @return false if the module is in an incorrect state, true otherwise.
     */
    bool isAlive()
    {
        cleanCommandBuffer();
        Si4455_DeviceState const& cs = cmd_requestDeviceState();
        return (cs.CURR_STATE != 0) && (cs.CURR_STATE != 0xFF) && lastCommandSucceeded();// && cs.CURRENT_CHANNEL == m_listeningChannel);
    }

    //! Returns the current RSSI reading from the modem.
    uint8_t readCurrentRssi()
    {
        // Clear RSSI_LATCH_PEND and RSSI_PEND status bits
        return cmd_readModemStatus(0x27).CURR_RSSI;
    }


    //! Check device status and clear the flag
    bool testForStatusAndClear(Status status)
    {
        update();

        if (hasStatus(status)) {
            clearStatus(status);
            return true;
        }
        return false;
    }

    void clear(Status status)
    {
        clearStatus(status);
    }


    //! Hardware reset the chip using the shutdown pin
    void hardwareReset()
    {
        // Put radio in shutdown, wait then release
        hal.putInShutdown();
        delay(10);
        clearStatus();
        hal.releaseFromShutdown();
        delay(10);
    }

    void holdInReset()
    {
        hal.putInShutdown();
        clearStatus();
    }

    void releaseFromReset()
    {
        hal.releaseFromShutdown();
    }



public:
    // #### EZ RADIO COMMANDS ####

    //! Reads data byte(s) from the current position in RX FIFO. Returns false on error.
    //! Doesn't need CTS
    void cmd_readRxFifo(uint8_t* data, uint8_t length)
    {
        readDataWithoutClearToSend(SI4455_CMD_ID_READ_RX_FIFO, data, length);
    }

    //! Writes data byte(s) to the TX FIFO.
    void cmd_writeTxFifo(const uint8_t* data, uint8_t length)
    {
        writeDataWithoutClearToSend(SI4455_CMD_ID_WRITE_TX_FIFO, data, length);
    }
    //! Writes data byte(s) to the TX FIFO.
    void cmd_writeTxFifoWithZeros(uint8_t length)
    {
        writeZerosWithoutClearToSend(SI4455_CMD_ID_WRITE_TX_FIFO, length);
    }


    //! Switches to RX state and starts reception of a packet.
    void cmd_startRx(uint8_t channel, uint8_t condition, uint16_t length, uint8_t nextState1, uint8_t nextState2, uint8_t nextState3)
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
    void cmd_startRx(uint8_t channel, uint8_t condition, uint16_t length)
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

    //! Switches to TX state and starts transmission of a packet.
    void cmd_startTx(uint8_t channel, uint8_t condition, uint16_t length)
    {
        const uint8_t buffer[] = {
            SI4455_CMD_ID_START_TX,
            channel,
            condition,
            (uint8_t)(length >> 8),
            (uint8_t)(length),
            0,
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_START_TX);
    }


    //! Request current device state and channel.
    Si4455_DeviceState const& cmd_requestDeviceState()
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
    void cmd_changeState(uint8_t nextState)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_CHANGE_STATE,
            nextState
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_CHANGE_STATE);
    }



    //! Returns the interrupt status of ALL the possible interrupt events (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the corresponding clear mask argument).
    Si4455_InterruptStatus const& cmd_readInterruptStatus(uint8_t pendingPacketHandlerIntsClearMask, uint8_t pendingModemIntsClearMask, uint8_t pendingChipIntsClearMask)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_INT_STATUS,
            pendingPacketHandlerIntsClearMask,
            pendingModemIntsClearMask,
            pendingChipIntsClearMask
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_INT_STATUS,
                                   m_commandReply.RAW, SI4455_CMD_REPLY_COUNT_GET_INT_STATUS);

        //processPacketHandlerInterruptPending(m_commandReply.GET_INT_STATUS.PH_PEND);
        //processModemInterruptPending(m_commandReply.GET_INT_STATUS.MODEM_PEND);
        //processChipInterruptPending(m_commandReply.GET_INT_STATUS.CHIP_PEND);

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

    Si4455_InterruptStatus const& cmd_readAndClearInterruptStatus()
    {
        return cmd_readInterruptStatus(0,0,0);
    }

    //! Clear all pending interrupts
    void cmd_clearAllPendingInterrupts()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_INT_STATUS,
            0x00,
            0x00,
            0x00
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_GET_INT_STATUS);
    }


    //!  Reads the fast response registers (FRR) starting with FRR_A.
    Si4455_FrrA const& cmd_readFrrA(uint8_t registersToRead = 1)
    {
        readDataWithoutClearToSend(SI4455_CMD_ID_FRR_A_READ,
                                   m_commandReply.RAW,
                                   registersToRead);
        return m_commandReply.FRR_A_READ;

        // Si4455Cmd.FRR_A_READ.FRR_A_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_A_READ.FRR_B_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_A_READ.FRR_C_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_A_READ.FRR_D_VALUE = radioCmd[3];
    }

    //! Reads the fast response registers (FRR) starting with FRR_B.
    Si4455_FrrB const& cmd_readFrrB(uint8_t registersToRead = 1)
    {
        readDataWithoutClearToSend(SI4455_CMD_ID_FRR_B_READ,
                                   m_commandReply.RAW,
                                   registersToRead);
        return m_commandReply.FRR_B_READ;

        // Si4455Cmd.FRR_B_READ.FRR_B_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_B_READ.FRR_C_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_B_READ.FRR_D_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_B_READ.FRR_A_VALUE = radioCmd[3];
    }


    //! Reads the fast response registers (FRR) starting with FRR_C.
    Si4455_FrrC const& cmd_readFrrC(uint8_t registersToRead = 1)
    {
        readDataWithoutClearToSend(SI4455_CMD_ID_FRR_C_READ,
                                   m_commandReply.RAW,
                                   registersToRead);
        return m_commandReply.FRR_C_READ;

        // Si4455Cmd.FRR_C_READ.FRR_C_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_C_READ.FRR_D_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_C_READ.FRR_A_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_C_READ.FRR_B_VALUE = radioCmd[3];
    }

    //! Reads the fast response registers (FRR) starting with FRR_D.
    Si4455_FrrD const& cmd_readFrrD(uint8_t registersToRead = 1)
    {
        readDataWithoutClearToSend(SI4455_CMD_ID_FRR_D_READ,
                                   m_commandReply.RAW,
                                   registersToRead);
        return m_commandReply.FRR_D_READ;

        // Si4455Cmd.FRR_D_READ.FRR_D_VALUE = radioCmd[0];
        // Si4455Cmd.FRR_D_READ.FRR_A_VALUE = radioCmd[1];
        // Si4455Cmd.FRR_D_READ.FRR_B_VALUE = radioCmd[2];
        // Si4455Cmd.FRR_D_READ.FRR_C_VALUE = radioCmd[3];
    }

    //! No Operation command.
    void cmd_noOperation()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_NOP
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_NOP);
    }


    //! Reset the internal FIFOs.
    void cmd_resetRxAndTxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x03
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    //! Reset the internal RX FIFO.
    void cmd_resetRxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x02
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    //! Reset the internal TX FIFO.
    void cmd_resetTxFifo()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_FIFO_INFO,
            0x01
        };

        sendCommand(buffer, SI4455_CMD_ARG_COUNT_FIFO_INFO);
    }

    //! Access the current byte counts in the TX and RX FIFOs, and provide for resetting the FIFOs (see doc).
    Si4455_FifoInfo const& cmd_readFifoInfo(uint8_t fifoResetMask = 0)
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
    Si4455_PacketInfo const& cmd_readPacketInfo()
    {
        return cmd_readPacketInfo(0, 0, 0);
    }

    //! Returns information about the length of the variable field in the last packet received, and (optionally) overrides field length.
    Si4455_PacketInfo const& cmd_readPacketInfo(uint8_t fieldNum, uint16_t length, uint16_t lenDiff)
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
    Si4455_CommandBuffer const& cmd_readCommandBuffer()
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
    Si4455_Properties const& cmd_readProperties(uint8_t group, uint8_t count, uint8_t startProperty)
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_GET_PROPERTY,
            group,
            count,
            startProperty
        };

        sendCommandAndReadResponse(buffer, SI4455_CMD_ARG_COUNT_GET_PROPERTY,
                                   m_commandReply.RAW, count);
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

    //! Sets the value of one or more properties. Max property count is 16.
    void cmd_setProperties(uint8_t group, uint8_t count, uint8_t propertyIndex, uint8_t data, ...)
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
            buffer[cmdIndex] = (uint8_t)(va_arg(argList, int));
            cmdIndex++;
            if (cmdIndex == 16)
                break;
        }
        va_end(argList);

        sendCommand(buffer, cmdIndex);
    }


    //! Returns the interrupt status of the Packet Handler Interrupt Group (both STATUS and PENDING).
    //! Optionally, it may be used to clear latched (PENDING) interrupt events (set bit to 0 in the mask).
    Si4455_PhStatus const& cmd_readPacketHandlerStatus(uint8_t pendingPacketHandlerIntsClearMask = 0xFF)
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
    Si4455_ModemStatus const& cmd_readModemStatus(uint8_t pendingModemIntsClearMask = 0xFF)
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
    Si4455_ChipStatus const& cmd_readChipStatus(uint8_t pendingChipIntsClearMask = 0xFF)
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



    //! Configures the GPIO pins.
    Si4455_GpioPinConfig const& cmd_configureGpioPins(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3,
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
    Si4455_AdcReadings const& cmd_readAdc(uint8_t adcEnable, uint8_t adcConfig)
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


    //! Reports basic information about the device.
    Si4455_PartInfo const& cmd_readPartInformation()
    {
        uint8_t const buffer[] = {
            SI4455_CMD_ID_PART_INFO
        };

        // TODO: check PART value, seems like MSB is null and it shouldn't be.
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

    //! Returns the Function revision information of the device.
    Si4455_FuncInfo const& cmd_readFunctionRevisionInformation()
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


private:
        //! Power up the chip.
    void powerUp()
    {
        debugln("Powering Up...");

        // Hardware reset the chip
        hardwareReset();

        // Wait until reset timeout or Reset IT signal
        //for (unsigned int wDelay = 0; wDelay < RadioConfiguration.Radio_Delay_Cnt_After_Reset; wDelay++);
        delay(100);
    }

    //! Load all properties and commands with a list of NULL terminated commands.
    //! Call @reset before.
    bool initialize(uint8_t const* configArray)
    {
        debugln("Initializing...");

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
                        debugln("Init failed: Command bytes exceeds 128 bytes");
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
                    debugln("Init failed: Command bytes exceeds max");
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
                debugln("Init failed: Command timeout");
                return false;
            }

            // Check response byte for EZCONFIG_CHECK command
            if (radioCmd[0] == SI4455_CMD_ID_EZCONFIG_CHECK) {
                if (response != SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_VALID) {
                    // EZConfig failed, either SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_BAD_CHECKSUM or SI4455_CMD_EZCONFIG_CHECK_REP_RESULT_ENUM_INVALID_STATE
                    debugln("Init failed: EZConfig Check error");
                    return false;
                }
            }

            if (hal.isIrqAsserted()) {
                // Get and clear all interrupts. An error has occured...
                Si4455_InterruptStatus const& it = cmd_readAndClearInterruptStatus();
                if (it.CHIP_PEND & SI4455_CMD_GET_CHIP_STATUS_REP_CMD_ERROR_PEND_MASK) {
                    // Command error
                    debugln("Init failed: Command Error");
                    return false;
                }
            }
        }

        debugln("Init succeeded.");
        return true;
    }

    //! Writes data byte(s) to the EZConfig array (array generated from EZConfig tool).
    bool writeEZConfigArray(uint8_t const* ezConfigArray, uint8_t count)
    {
        return writeData(SI4455_CMD_ID_WRITE_TX_FIFO, ezConfigArray, count);
    }


    bool waitReadyToSendPacket(unsigned long timeout_ms = 100)
    {
        // Wait for the device to be ready to send a packet
        unsigned long const t { millis() };
        RadioState s {RadioState::Invalid};
        do {
            s = radioState();

            if ((millis()-t) > timeout_ms)
                return false;

        } while ((s == RadioState::Tx) || (s == RadioState::TxTune)); // Goes out of Tx/TxTune when packet is sent

        return statusNoError();
    }

    //! Read a packet from RX FIFO, with size checks
    ReadPacketResult readPacket(Si4455_FifoInfo const& fifoInfo, uint8_t* data, uint8_t byteCount)
    {
        bool const dataRemaining { (fifoInfo.RX_FIFO_COUNT > byteCount) };

        debug("Read Packet: ");
        debug(byteCount); debug('/'); debugln(fifoInfo.RX_FIFO_COUNT);

        if (byteCount > fifoInfo.RX_FIFO_COUNT) {
            //cmd_resetRxFifo();
            debugln("Read Packet: Not Enough Data In Fifo");
            return ReadPacketResult::NotEnoughDataInFifo;
        }

        clearStatus(Status::DataAvailable);

        // Read FIFO
        cmd_readRxFifo(data, byteCount);

        if (dataRemaining)
            raiseStatus(Status::DataAvailable);

        updateStatus();

        return statusNoError() ? ReadPacketResult::Success : ReadPacketResult::RequestFailed;
    }


    //! Ask to start listening on given channel with given packet length.
    //! Auto-returns to RX mode after receiving a valid packet.
    //! For variable length packet configuration, set packetLegth to zero.
    bool startListening(uint8_t channel, uint8_t packetLength)
    {
        debug("Listening on channel ");
        debug(channel);
        debug(" with packet size ");
        debugln(packetLength);

        clearStatus();
        cmd_clearAllPendingInterrupts();

        // Start Receiving packet on channel, START immediately, Packet n bytes long
        cmd_startRx(channel, 0, packetLength,
                    SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_RX,
                    SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_RX,
                    SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX);

        return lastCommandSucceeded();
    }

    //! Ask to start listening on given channel with given packet length.
    //! Goes to Ready mode after receiving a valid packet. Call startListening again to listen for new packets.
    //! For variable length packet configuration, set packetLegth to zero.
    bool startListeningSinglePacket(uint8_t channel, uint8_t packetLength)
    {
        debug("Listening single packet on channel ");
        debug(channel);
        debug(" with packet size ");
        debugln(packetLength);

        clearStatus();
        cmd_clearAllPendingInterrupts();

        // Start Receiving packet on channel, START immediately, Packet n bytes long
        cmd_startRx(channel, 0, packetLength,
                    SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_RX,
                    SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_READY,
                    SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX);

        return lastCommandSucceeded();
    }


    void readAndProcessPendingInterrupts()
    {
        Si4455_InterruptStatus const& is {cmd_readInterruptStatus(0,0,0)}; // Read and clear all pending interrupts

        processPacketHandlerInterruptPending(is.PH_PEND);
        processModemInterruptPending(is.MODEM_PEND);
        processChipInterruptPending(is.CHIP_PEND);
    }

    //! Read pending interrupts via FRR
    void updateStatus()
    {
        // FRR B: PH_PEND
        // FRR C: MODEM_PEND
        // FRR D: CHIP_PEND
        Si4455_FrrB const& frrb = cmd_readFrrB(3);

        if (lastCommandFailed())
            return;

        //clearStatus();

        bool clearIT {false};
        clearIT |= processPacketHandlerInterruptPending(frrb.FRR_B_VALUE);
        clearIT |= processModemInterruptPending(frrb.FRR_C_VALUE);
        clearIT |= processChipInterruptPending(frrb.FRR_D_VALUE);

//#ifdef ZETARF_DEBUG_ON
//        cmd_clearAllPendingInterrupts();
//#else
        if (clearIT)
            cmd_clearAllPendingInterrupts();
//#endif

        // TODO: check how to implement properly this auto-recovery and if it is really needed
        //if (m_deviceStatus & (CommandError|CrcError)) // auto recovery
          //  startListening();
    }

    //! Process Packet Handler interrupts
    //! @return true to clear interrupts, false otherwise
    bool processPacketHandlerInterruptPending(uint8_t phPend)
    {
        bool clearIT {false};

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::DataTransmitted);
            debugln("Packet IT: Data Transmitted");
        }
        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT) {
            clearIT = true;
            // @todo Add circular buffer?
            raiseStatus(Status::DataAvailable);
            debugln("Packet IT: Data Available");
        }

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_CRC_ERROR_PEND_BIT) {
            clearIT = true;
            cmd_resetRxFifo(); // MAYBE: leave that choice to the user?
            raiseStatus(Status::CrcError);
            debugln("Packet IT: CRC Error");
        }

        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_TX_FIFO_ALMOST_EMPTY_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::FifoAlmostEmpty);
            debugln("Packet IT: TX FIFO Almost Empty");
        }
        if (phPend & SI4455_CMD_GET_INT_STATUS_REP_RX_FIFO_ALMOST_FULL_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::FifoAlmostFull);
            debugln("Packet IT: RX FIFO Almost Full");
        }

        return clearIT;
    }

    //! Process Modem interrupts
    //! @return true to clear interrupts, false otherwise
    bool processModemInterruptPending(uint8_t modemPend)
    {
        bool clearIT {false};

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_INVALID_SYNC_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::InvalidSync);
            //debugln("Modem IT: Invalid Sync");
        }
        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_INVALID_PREAMBLE_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::InvalidPreamble);
            //debugln("Modem IT: Invalid Preamble");
        }

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_PREAMBLE_DETECT_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::DetectedPreamble);
            //debugln("Modem IT: Detected Preamble");
        }
        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_SYNC_DETECT_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::DetectedSync);
            //debugln("Modem IT: Detected Sync");
        }

        if (modemPend & SI4455_CMD_GET_INT_STATUS_REP_RSSI_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::LatchedRssi);
            debugln("Modem IT: RSSI Latched");
        }

        return clearIT;
    }

    //! Process Chip interrupts
    //! @return true to clear interrupts, false otherwise
    bool processChipInterruptPending(uint8_t chipPend)
    {
        bool clearIT {false};

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT) {
            clearIT = true;
            cmd_resetRxFifo(); // MAYBE: leave that choice to the user?
            raiseStatus(Status::FifoUnderflowOrOverflowError);
            debugln("Chip IT: FIFO Underflow/Overflow");
        }

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_CMD_ERROR_PEND_BIT) {
            clearIT = true;
            raiseStatus(Status::CommandError);
            debugln("Chip IT: Command Error");
        }

        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_STATE_CHANGE_PEND_BIT) {
            //clearIT = true;
            //debugln("Chip IT: State Change");
        }
        if (chipPend & SI4455_CMD_GET_INT_STATUS_REP_CHIP_READY_PEND_BIT) {
            //clearIT = true;
            debugln("Chip IT: Chip Ready");
        }

        return clearIT;
    }


    // #### INTERNAL COMM HANDLING ####

    bool lastCommandSucceeded() const {
        return !lastCommandFailed();
    }
    bool lastCommandFailed() const {
        return hasStatus(Status::DeviceBusy);
    }


    /*!
     * Sends a command to the radio chip.
     *
     * @param data  Pointer to the command to send.
     * @param count Number of bytes in the command to send to the radio device.
     *
     * @return true if data sent, false otherwise.
     */
    bool sendCommand(uint8_t const* data, uint8_t count)
    {
        if (!waitForClearToSend())
            return false;

        hal.restartOrBeginSpiTransaction();
        hal.spiWriteData(data, count);
        hal.endSpiTransaction();
        return true;
    }

    /*!
     * Sends a command to the radio chip and gets a response.
     *
     * @param commandData       Pointer to the command data.
     * @param commandByteCount  Number of bytes in the command to send to the radio device.
     * @param responseData      Pointer to where to put the response data.
     * @param responseByteCount Number of bytes in the response to fetch.
     *
     * @return true if send and read succeeded, false otherwise.
     */
    bool sendCommandAndReadResponse(uint8_t const* commandData, uint8_t commandByteCount, uint8_t* responseData, uint8_t responseByteCount)
    {
        if (!sendCommand(commandData, commandByteCount))
            return false;

        // Wait until radio IC is ready with the data
        if (!waitForClearToSend())
            return false;

        hal.resumeOrBeginSpiTransaction();
        hal.spiReadData(responseData, responseByteCount);
        hal.endSpiTransaction();
        return true;
    }


    /*!
     * Gets a command response from the radio chip without waiting for CTS.
     *
     * @param command     Command ID.
     * @param data        Pointer to where to put the data.
     * @param count       Number of bytes to get from the radio chip.
     *
     * @return true if read succeeded, false otherwise.
     */
    bool readDataWithoutClearToSend(uint8_t command, uint8_t* data, uint8_t count)
    {
        hal.resumeOrBeginSpiTransaction();
        hal.spiWriteByte(command);
        hal.spiReadData(data, count);
        hal.endSpiTransaction();
        return true;
    }

    //! Waits for CTS and gets a command response from the radio chip.
    bool readData(uint8_t command, uint8_t* data, uint8_t count)
    {
        if (!waitForClearToSend())
            return false;

        return readDataWithoutClearToSend(command, data, count);
    }

    /*!
     * Sends a command to the radio chip without waiting for CTS.
     *
     * @param command     Command ID.
     * @param data        Pointer to the data to write. Must be valid.
     * @param count       Number of bytes to write to the radio chip.
     *
     * @return true if write succeeded, false otherwise.
     */
    bool writeDataWithoutClearToSend(uint8_t command, uint8_t const* data, uint8_t count)
    {
        hal.restartOrBeginSpiTransaction();
        hal.spiWriteByte(command);
        hal.spiWriteData(data, count);
        hal.endSpiTransaction();
        return true;
    }
    bool writeZerosWithoutClearToSend(uint8_t command, uint8_t count)
    {
        hal.restartOrBeginSpiTransaction();
        hal.spiWriteByte(command);
        while (count--)
            hal.spiWriteByte(0);
        hal.endSpiTransaction();
        return true;
    }

    //! Waits for CTS and sends a command to the radio chip.
    bool writeData(uint8_t command, uint8_t const* data, uint8_t count)
    {
        if (!waitForClearToSend())
            return false;

        return writeDataWithoutClearToSend(command, data, count);
    }


    //! Waits for CTS to be high. Returns false if timeout occured.
    //! Check CTS via SPI command, holds SPI transaction if clear to send.
    //template <typename Hal_ = Hal, typename std::enable_if<!Hal_::HasHardwareClearToSend>>
    template<typename Hal_ = Hal,
         typename = std::enable_if_t<
            !Hal_::HasHardwareClearToSend
            && std::is_same<Hal_, Hal>::value>>
    bool waitForClearToSend(uint16_t timeout_ms = 300)
    {
        unsigned long const t { millis() };

        uint8_t ctsVal {0};
        do {
            hal.restartOrBeginSpiTransaction();
            hal.spiWriteByte(SI4455_CMD_ID_READ_CMD_BUFF);
            ctsVal = hal.spiReadByte();

            if ((millis()-t) > timeout_ms) {
                hal.endSpiTransaction();
                deviceBusy();
                return false;
            }

        } while (ctsVal != 0xFF); // Clear to send when 0xFF

        // Holds SPI transaction
        deviceReady();
        return true;
    }

    //! Waits for CTS to be high. Returns false if timeout occured.
    //! Uses Hardware CTS I/O.
    //template <typename T = typename std::enable_if<Hal::HasHardwareClearToSend>::type>
    template<typename Hal_ = Hal,
         typename = std::enable_if_t<
            Hal_::HasHardwareClearToSend
            && std::is_same<Hal_, Hal>::value>>
    bool waitForClearToSend(uint16_t timeout_ms = 300, uint8_t _ = 0)
    {
        unsigned long const t { millis() };
        while (!hal.isClearToSend()) {
            // Wait with timeout...
            if ((millis()-t) > timeout_ms) {
                deviceBusy();
                return false;
            }

            delayMicroseconds(100);
        }
        deviceReady();
        return true;
    }


private:
    void raiseStatus(Status v) {
        m_deviceStatus |= v;
    }
    void clearStatus(Status v) {
        m_deviceStatus &= ~v;
    }
    void clearStatus() {
        m_deviceStatus = Status::NoStatus;
    }
    void setStatus(Status v, bool enable) {
        if (enable)
            raiseStatus(v);
        else
            clearStatus(v);
    }
    bool hasStatus(Status status) const {
        return !is_null(m_deviceStatus & status);
    }

    void deviceBusy()
    {
        raiseStatus(Status::DeviceBusy);
    }
    void deviceReady()
    {
        clearStatus(Status::DeviceBusy);
    }

    // Write zeros in command buffer
    void cleanCommandBuffer() {
        memset(m_commandReply.RAW, 0, 16);
    }


    si4455_cmd_reply_union m_commandReply;

    Status m_deviceStatus {Status::NoStatus};

    uint8_t m_listeningChannel {0};
    //uint8_t m_transmittingChannel {0};
    uint8_t m_packetLength {0};
};
