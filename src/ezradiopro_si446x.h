/*
 * EZRadio Commands
 */

#pragma once

#include "ezradiopro_si446x_defs.h"
#include "zetarf_radio.hpp"
#include "flags/flags.hpp"


namespace ZetaRFEZRadioPro {

template<class Hal>
class EZRadioProSi446x
{	
public:
	using Event = ZetaRF::Event;
	using Events = ZetaRF::Events;
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

	uint8_t _cast(StartTxOption_Update update) {
		return static_cast<uint8_t>(update);
	}
	uint8_t _cast(StartTxOption_ReTransmit retransmit) {
		return static_cast<uint8_t>(retransmit);
	}
	uint8_t _cast(StartTxOption_Start start) {
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

			uint8_t response {0};
			if (!sendCommandAndReadResponse(radioCmd, cmdBytesCount, &response, 1)) {
				// Timeout occured
				//debugln("Init failed: Command timeout");
				return false;
			}

			if (m_hal.isIrqAsserted()) {
				// Get and clear all interrupts. An error has occured...
				_cleanCommandBuffer();
				EZRadioReply::InterruptStatus const& it = readAndClearInterruptStatus();
				if (it.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CHIP_PEND_CMD_ERROR_PEND_MASK) {
					// Command error
					//debugln("Init failed: Command Error");
					return false;
				}
			}
		}
		return true;
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
	void startRx(uint8_t channel, uint8_t condition, uint16_t length, uint8_t nextState1, uint8_t nextState2, uint8_t nextState3)
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

	void startRx(uint8_t channel, uint8_t condition, uint16_t length)
	{
		// No changes to next states, avoid sending the extra 3 bytes
		uint8_t const buffer[] = {
			SI446X_CMD_ID_START_RX,
			channel,
			condition,
			(uint8_t)(length >> 8),
			(uint8_t)(length)
		};

		sendCommand(buffer, SI446X_CMD_ARG_COUNT_START_RX-3);
	}

	void startRxWithPreviousParameters()
	{
		// Use the last settings
		uint8_t const buffer[] = {
			SI446X_CMD_ID_START_RX
		};

		sendCommand(buffer, 1);
	}

	//! Switches to TX state and starts transmission of a packet.
	void startTx(uint8_t channel, uint8_t txCompleteState, uint16_t length,
				 StartTxOption_ReTransmit retransmit = StartTxOption_ReTransmit::SendDataFromTxFifo,
				 uint8_t delayBetweenRetransmitPackets_us = 0,
				 uint8_t retransmitRepeatCount = 0,
				 StartTxOption_Update update = StartTxOption_Update::UseTxParametersNow,
				 StartTxOption_Start start = StartTxOption_Start::Immediate)
	{
		const uint8_t buffer[] = {
			SI446X_CMD_ID_START_TX,
			channel,
			uint8_t(((txCompleteState&0x0F) << SI446X_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_LSB)
					| (_cast(update) << SI446X_CMD_START_TX_ARG_CONDITION_UPDATE_LSB)
			        | (_cast(retransmit) << SI446X_CMD_START_TX_ARG_CONDITION_RETRANSMIT_LSB)
					| (_cast(start) << SI446X_CMD_START_TX_ARG_CONDITION_START_ENUM_IMMEDIATE)
					),
			uint8_t((length&0x1FFF) >> 8),
			uint8_t(length),
			delayBetweenRetransmitPackets_us, // Delay (in usec) between packet retransmissions
			retransmitRepeatCount  // The number of times to repeat the packet.
		};

		sendCommand(buffer, SI446X_CMD_ARG_COUNT_START_TX);
	}

	//! Start TX for packet retransmission
	void startTxRepeat(uint8_t channel, uint8_t txCompleteState, uint16_t length,
				 StartTxOption_ReTransmit retransmit,
				 uint8_t delayBetweenPacket_us,
				 uint8_t repeatCount)
	{
		const uint8_t buffer[] = {
			SI446X_CMD_ID_START_TX,
			channel,
			uint8_t(((txCompleteState&0x0F) << SI446X_CMD_START_TX_ARG_CONDITION_TXCOMPLETE_STATE_LSB)
					| (SI446X_CMD_START_TX_ARG_CONDITION_UPDATE_ENUM_USE << SI446X_CMD_START_TX_ARG_CONDITION_UPDATE_LSB)
			        | (_cast(retransmit) << SI446X_CMD_START_TX_ARG_CONDITION_RETRANSMIT_LSB)
					| (SI446X_CMD_START_TX_ARG_CONDITION_START_ENUM_IMMEDIATE << SI446X_CMD_START_TX_ARG_CONDITION_START_ENUM_IMMEDIATE)
					),
			uint8_t((length&0x1FFF) >> 8),
			uint8_t(length),
			delayBetweenPacket_us, // Delay (in usec) between packet retransmissions
			repeatCount  // The number of times to repeat the packet.
		};

		sendCommand(buffer, SI446X_CMD_ARG_COUNT_START_TX);
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
			SI446X_CMD_ID_GET_CHIP_STATUS
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
			it |= (Event::Rssi);

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

		m_hal.resumeOrBeginSpiTransaction();
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

} // namespace ZetaRFEZRadioPro
