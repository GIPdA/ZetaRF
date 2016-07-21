/*! @file ZetaRF.h
 *
 * @brief This file contains the public functions controlling the Si4455 radio chip.
 */

#ifndef ZETARF_H_
#define ZETARF_H_

#include "radio_config.h"
#include "si4455_defs.h"


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


class ZetaRF
{
public:
    enum CommandResult
    {
        Success,
        NoPatch,
        CtsTimeout,
        PatchFail,
        CommandError
    };


    ZetaRF(int csPin, int shutdownPin, int irqPin);

    void begin();


    void sendData(uint8_t channel, uint8_t *data, uint8_t count);
    void sendPacket(uint8_t channel, uint8_t *data);
    
    void startReceiver(uint8_t channel);
    void startReceiver(uint8_t channel, uint8_t byteCount);

    bool checkTransmitted();
    bool checkReceived();
    
private:
    typedef si4455_cmd_reply_union CommandReply;

    void powerUp();


    void reset();
    CommandResult initialize(const uint8_t* configArray);

    void writeEZConfigArray(const uint8_t* ezConfigArray, uint8_t count);
    uint8_t checkEZConfig(uint16_t checksum);

    void startTx(uint8_t channel, uint8_t condition, uint16_t count);
    void startRx(uint8_t channel, uint8_t condition, uint16_t count, uint8_t nextState1, uint8_t nextState2, uint8_t nextState3);

    Si4455_InterruptStatus& readInterruptStatus(uint8_t clearPendingPH, uint8_t clearPendingModem, uint8_t clearPendingChip);


    Si4455_GpioPinConfig& configureGpioPins(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3, uint8_t nirq, uint8_t sdo, uint8_t genConfig);
    void setProperties(uint8_t group, uint8_t count, uint8_t property, ...);
    void changeState(uint8_t nextState1);


    // Extended driver support
    void nopCommand();

    void writeTxFifo(uint8_t* data, uint8_t count);
    void readRxFifo(uint8_t* data, uint8_t count);

    Si4455_FifoInfo& readFifoInfo(uint8_t fifo = 0);
    Si4455_PartInfo& readPartInfo();
    Si4455_Properties& readProperties(uint8_t group, uint8_t count, uint8_t startProperty);
    Si4455_FuncInfo& readFuncInfo();

    Si4455_FrrA& readFrrA(uint8_t count);
    Si4455_FrrB& readFrrB(uint8_t count);
    Si4455_FrrC& readFrrC(uint8_t count);
    Si4455_FrrD& readFrrD(uint8_t count);

    Si4455_DeviceState& requestDeviceState();
    Si4455_CommandBuffer& readCommandBuffer();
    Si4455_AdcReadings& readADC(uint8_t adcEnable, uint8_t adcConfig);
    Si4455_PhStatus& readPhStatus(uint8_t clearPendingPH);
    Si4455_ModemStatus& readModemStatus(uint8_t clearPendingModem);
    Si4455_ChipStatus& readChipStatus(uint8_t clearPendingChip);




    uint8_t getResponse(uint8_t* data, uint8_t count);
    void sendCommand(const uint8_t* data, uint8_t count);
    void readData(uint8_t command, uint8_t* data, uint8_t count, bool pollCtsFlag);
    void writeData(uint8_t command, const uint8_t* data, uint8_t count, bool pollCtsFlag);
    uint8_t pollCts();
    void clearCts();
    uint8_t sendCommandAndGetResponse(const uint8_t* commandData, uint8_t commandByteCount, uint8_t* responseData, uint8_t responseByteCount);


    void assertShutdown();
    void deassertShutdown();
    void clearCS();
    void setCS();
    bool irqLevel();

    uint8_t spiReadWriteByte(uint8_t value);
    void spiWriteByte(uint8_t value);
    uint8_t spiReadByte();


    void spiReadWriteData(uint8_t* data, uint8_t count);
    void spiWriteData(const uint8_t* data, uint8_t count);
    void spiReadData(uint8_t* data, uint8_t count);


    const int m_csPin;
    const int m_sdnPin;
    const int m_irqPin;
    bool m_ctsWentHigh;

    si4455_cmd_reply_union m_commandReply;

    const uint8_t m_channelNumber;
    const uint8_t m_packetLength;


    const uint8_t *m_radioConfigurationDataArray;
    //static const RadioConfiguration RadioConfigurationData;// = RADIO_CONFIGURATION_DATA(RadioConfigurationDataArray);
};

#endif /* ZETARF_H_ */
