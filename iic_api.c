/*This file is inteded for Xilinx IIC on generic platform 
 * original Please read the READ me before proceeding further  
 *
 * */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <csp.h>
#include <map_system_memory.h>
#include <csp_sysregs.h>

/******************************************************************************
*
* Send the specified buffer to the device that has been previously addressed
* on the IIC bus. This function assumes that the 7 bit address has been sent.
*
* @param	BaseAddress contains the base address of the IIC Device.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent.
* @param	Option: XIIC_STOP = end with STOP condition, XIIC_REPEATED_START
*		= don't end with STOP condition.
*
* @return	The number of bytes remaining to be sent.
*
* @note		This function does not take advantage of the transmit Fifo
*		because it is designed for minimal code space and complexity.
*
******************************************************************************/
static unsigned DynSendData( uint32_t  BaseAddress, uint8_t  *BufferPtr,
			    uint8_t  ByteCount, uint8_t  Option)
{
	uint32_t IntrStatus;
	printf("Dyan Send Data\n");

	while (ByteCount > 0) {
		printf("ByteCount %d\n", ByteCount);	
		/*
		 * Wait for the transmit to be empty before sending any more
		 * data by polling the interrupt status register.
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);
			if (IntrStatus & (XIIC_INTR_TX_ERROR_MASK |
					  XIIC_INTR_ARB_LOST_MASK |
					  XIIC_INTR_BNB_MASK)) {
				/*
				 * Error condition (NACK or ARB Lost or BNB
				 * Error Has occurred. Clear the Control
				 * register to send a STOP condition on the Bus
				 * and return the number of bytes still to
				 * transmit.
				 */
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
						0x03);
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
						0x01);

				return ByteCount;
			}

			/*
			 * Check for the transmit Fifo to become Empty.
			 */
			if (IntrStatus & XIIC_INTR_TX_EMPTY_MASK) {
				break;
			}
		}

		/*
		 * Send data to Tx Fifo. If a stop condition is specified and
		 * the last byte is being sent, then set the dynamic stop bit.
		 */
		if ((ByteCount == 1) && (Option == XIIC_STOP)) {
			/*
			 * The MSMS will be cleared automatically upon setting
			 *  dynamic stop.
			 */
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
					XIIC_TX_DYN_STOP_MASK | *BufferPtr++);
		} else {
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
					*BufferPtr++);
		}

		/*
		 * Update the byte count to reflect the byte sent.
		 */
		ByteCount--;
	}

	if (Option == XIIC_STOP) {
		/*
		 * If the Option is to release the bus after transmission of
		 * data, Wait for the bus to transition to not busy before
		 * returning, the IIC device cannot be disabled until this
		 * occurs.
		 */
		while (1) {
			if (XIic_ReadIisr(BaseAddress) & XIIC_INTR_BNB_MASK) {
				break;
			}
		}
	}

	return ByteCount;
}


/*****************************************************************************/
/**
* Send data as a master on the IIC bus. This function sends the data using
* polled I/O and blocks until the data has been sent. It only supports 7 bit
* addressing. The user is responsible for ensuring the bus is not busy if
* multiple masters are present on the bus.
*
* @param	BaseAddress contains the base address of the IIC Device.
* @param	Address contains the 7 bit IIC address of the device to send the
*		specified data to.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent.
* @param	Option: XIIC_STOP = end with STOP condition,
*		XIIC_REPEATED_START = don't end with STOP condition.
*
* @return	The number of bytes sent.
*
* @note		None.
*
******************************************************************************/
unsigned XIic_DynSend( uint32_t BaseAddress, uint16_t  Address, uint8_t  *BufferPtr,
			uint8_t ByteCount, uint8_t Option)
{
	unsigned RemainingByteCount;
	uint32_t StatusRegister;
	printf("Dyn Send is initiated\n");

	/*
	 * Clear the latched interrupt status so that it will be updated with
	 * the new state when it changes, this must be done after the address
	 * is put in the FIFO
	 */
	XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK |
			XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK);

	/*
	 * Put the address into the Fifo to be sent and indicate that the
	 * operation to be performed on the bus is a write operation. Upon
	 * writing the address, a start condition is initiated. MSMS is
	 * automatically set to master when the address is written to the Fifo.
	 * If MSMS was already set, then a re-start is sent prior to the
	 * address.
	 */
	if(!(Address & XIIC_TX_DYN_STOP_MASK)) {

		XIic_DynSend7BitAddress(BaseAddress, Address,
				XIIC_WRITE_OPERATION);
	} else {
		XIic_DynSendStartStopAddress(BaseAddress, Address,
					XIIC_WRITE_OPERATION);
	}

	/*
	 * Wait for the bus to go busy.
	 */
	StatusRegister = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);

	printf ("Waited for bus to go busy \n");

	while (( StatusRegister & XIIC_SR_BUS_BUSY_MASK) !=
			XIIC_SR_BUS_BUSY_MASK) {
		StatusRegister = XIic_ReadReg(BaseAddress,
				XIIC_SR_REG_OFFSET);
	}

	/*
	 * Clear the latched interrupt status for the bus not busy bit which
	 * must be done while the bus is busy.
	 */
	XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);

	/*
	 * Send the specified data to the device on the IIC bus specified by the
	 * the address.
	 */
	RemainingByteCount = DynSendData(BaseAddress, BufferPtr, ByteCount,
					 Option);

	/*
	 * The send is complete return the number of bytes that was sent.
	 */
	printf("Dyn Send is Done\n");
	return ByteCount - RemainingByteCount;
}


/*****************************************************************************/
/**
* Receive the specified data from the device that has been previously addressed
* on the IIC bus. This function assumes the following:
* - The Rx Fifo occupancy depth has been set to its max.
* - Upon entry, the Rx Fifo is empty.
* - The 7 bit address has been sent.
* - The dynamic stop and number of bytes to receive has been written to Tx
*   Fifo.
*
* @param	BaseAddress contains the base address of the IIC Device.
* @param	BufferPtr points to the buffer to hold the data that is
*		received.
* @param	ByteCount is the number of bytes to be received. The range of
*		this value is greater than 0 and not higher than 255.
*
* @return	The number of bytes remaining to be received.
*
* @note		This function contains loops that could cause the function not
*		to return if the hardware is not working.
*
******************************************************************************/
static unsigned DynRecvData(unsigned int  BaseAddress, unsigned int  *BufferPtr, unsigned int  ByteCount)
{
	unsigned int  StatusReg;
	unsigned int  IntrStatus;
	unsigned int  IntrStatusMask;

	while (ByteCount > 0) {

		/*
		 * Setup the mask to use for checking errors because when
		 * receiving one byte OR the last byte of a multibyte message
		 * an error naturally occurs when the no ack is done to tell
		 * the slave the last byte.
		 */
		if (ByteCount == 1) {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_BNB_MASK;
		} else {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK |
				XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_BNB_MASK;
		}

		/*
		 * Wait for a byte to show up in the Rx Fifo.
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);
			StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);

			if ((StatusReg & XIIC_SR_RX_FIFO_EMPTY_MASK) !=
				XIIC_SR_RX_FIFO_EMPTY_MASK) {
				break;
			}
			/*
			 * Check the transmit error after the receive full
			 * because when sending only one byte transmit error
			 * will occur because of the no ack to indicate the end
			 * of the data.
			 */
			if (IntrStatus & IntrStatusMask) {
				return ByteCount;
			}
		}

		/*
		 * Read in byte from the Rx Fifo. If the Fifo reached the
		 * programmed occupancy depth as programmed in the Rx occupancy
		 * reg, this read access will un throttle the bus such that
		 * the next byte is read from the IIC bus.
		 */
		*BufferPtr++ = XIic_ReadReg(BaseAddress,  XIIC_DRR_REG_OFFSET);
		ByteCount--;
	}

	return ByteCount;
}



/*****************************************************************************/
/**
* Receive data as a master on the IIC bus. This function receives the data
* using polled I/O and blocks until the data has been received. It only
* supports 7 bit addressing. The user is responsible for ensuring the bus is
* not busy if multiple masters are present on the bus.
*
* @param	BaseAddress contains the base address of the IIC Device.
* @param	Address contains the 7 bit IIC Device address of the device to
*		send the specified data to.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent. This value can't be
*		greater than 255 and needs to be greater than 0.
*
* @return	The number of bytes received.
*
* @note		Upon entry to this function, the IIC interface needs to be
*		already enabled in the CR register.
*
******************************************************************************/
unsigned XIic_DynRecv(unsigned int  BaseAddress, unsigned int  Address, unsigned int  *BufferPtr, unsigned int  ByteCount)
{
	unsigned RemainingByteCount;
	unsigned int  StatusRegister;

	/*
	 * Clear the latched interrupt status so that it will be updated with
	 * the new state when it changes.
	 */
	XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK |
			XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK);

	/*
	 * Send the 7 bit slave address for a read operation and set the state
	 * to indicate the address has been sent. Upon writing the address, a
	 * start condition is initiated. MSMS is automatically set to master
	 * when the address is written to the Fifo. If MSMS was already set,
	 * then a re-start is sent prior to the address.
	 */
	XIic_DynSend7BitAddress(BaseAddress, Address, XIIC_READ_OPERATION);

	/*
	 * Wait for the bus to go busy.
	 */
	StatusRegister = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);

	while (( StatusRegister & XIIC_SR_BUS_BUSY_MASK)
			!= XIIC_SR_BUS_BUSY_MASK) {
		StatusRegister = XIic_ReadReg(BaseAddress,
				XIIC_SR_REG_OFFSET);
	}

	/*
	 * Clear the latched interrupt status for the bus not busy bit which
	 * must be done while the bus is busy.
	 */
	XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);

	/*
	 * Write to the Tx Fifo the dynamic stop control bit with the number of
	 * bytes that are to be read over the IIC interface from the presently
	 * addressed device.
	 */
	XIic_DynSendStop(BaseAddress, ByteCount);

	/*
	 * Receive the data from the IIC bus.
	 */
	RemainingByteCount = DynRecvData(BaseAddress, BufferPtr, ByteCount);

	/*
	 * The receive is complete. Return the number of bytes that were
	 * received.
	 */
	return ByteCount - RemainingByteCount;
}



/******************************************************************************
*
* Initialize the IIC core for Dynamic Functionality.
*
* @param	BaseAddress contains the base address of the IIC Device.
*
* @return	XST_SUCCESS if Successful else XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int XIic_DynInit(uint32_t BaseAddress)
{
	uint32_t Status;

	printf("Dyna Init \n");

	/*
	 * Reset IIC Core.
	 */
	XIic_WriteReg(BaseAddress, XIIC_RESETR_OFFSET, XIIC_RESET_MASK);

	/*
	 * Set receive Fifo depth to maximum (zero based).
	 */
	XIic_WriteReg(BaseAddress,  XIIC_RFD_REG_OFFSET,
			IIC_RX_FIFO_DEPTH - 1);

	/*
	 * Reset Tx Fifo.
	 */
	XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
			XIIC_CR_TX_FIFO_RESET_MASK);

	/*
	 * Enable IIC Device, remove Tx Fifo reset & disable general call.
	 */
	XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET,
			XIIC_CR_ENABLE_DEVICE_MASK);

	/*
	 * Read status register and verify IIC Device is in initial state. Only
	 * the Tx Fifo and Rx Fifo empty bits should be set.
	 */
	Status = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);

	printf("Dyna Init exit \n");
	if(Status == (XIIC_SR_RX_FIFO_EMPTY_MASK |
		XIIC_SR_TX_FIFO_EMPTY_MASK)) {
		return XST_SUCCESS;
	}

	return XST_FAILURE;
}

/*IIC read write single data*/

/******************************************************************************
*
* Receive the specified data from the device that has been previously addressed
* on the IIC bus.  This function assumes that the 7 bit address has been sent
* and it should wait for the transmit of the address to complete.
*
* @param	BaseAddress contains the base address of the IIC device.
* @param	BufferPtr points to the buffer to hold the data that is
*		received.
* @param	ByteCount is the number of bytes to be received.
* @param	Option indicates whether to hold or free the bus after reception
*		of data, XIIC_STOP = end with STOP condition,
*		XIIC_REPEATED_START = don't end with STOP condition.
*
* @return	The number of bytes remaining to be received.
*
* @note
*
* This function does not take advantage of the receive FIFO because it is
* designed for minimal code space and complexity.  It contains loops that
* that could cause the function not to return if the hardware is not working.
*
* This function assumes that the calling function will disable the IIC device
* after this function returns.
*
******************************************************************************/
static unsigned RecvData(uint32_t BaseAddress, uint8_t *BufferPtr,
			 unsigned ByteCount, uint8_t Option)
{
	uint32_t CntlReg;
	uint32_t IntrStatusMask;
	uint32_t IntrStatus;

	/* Attempt to receive the specified number of bytes on the IIC bus */

	while (ByteCount > 0) {
		/* Setup the mask to use for checking errors because when
		 * receiving one byte OR the last byte of a multibyte message an
		 * error naturally occurs when the no ack is done to tell the
		 * slave the last byte
		 */
		if (ByteCount == 1) {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_BNB_MASK;
		} else {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK |
				XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_BNB_MASK;
		}

		/* Wait for the previous transmit and the 1st receive to
		 * complete by checking the interrupt status register of the
		 * IPIF
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);
			if (IntrStatus & XIIC_INTR_RX_FULL_MASK) {
				break;
			}
			/* Check the transmit error after the receive full
			 * because when sending only one byte transmit error
			 * will occur because of the no ack to indicate the end
			 * of the data
			 */
			if (IntrStatus & IntrStatusMask) {
				return ByteCount;
			}
		}

		CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);

		/* Special conditions exist for the last two bytes so check for
		 * them. Note that the control register must be setup for these
		 * conditions before the data byte which was already received is
		 * read from the receive FIFO (while the bus is throttled
		 */
		if (ByteCount == 1) {
			if (Option == XIIC_STOP) {

				/* If the Option is to release the bus after the
				 * last data byte, it has already been read and
				 * no ack has been done, so clear MSMS while
				 * leaving the device enabled so it can get off
				 * the IIC bus appropriately with a stop
				 */
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
					 XIIC_CR_ENABLE_DEVICE_MASK);
			}
		}

		/* Before the last byte is received, set NOACK to tell the slave
		 * IIC device that it is the end, this must be done before
		 * reading the byte from the FIFO
		 */
		if (ByteCount == 2) {
			/* Write control reg with NO ACK allowing last byte to
			 * have the No ack set to indicate to slave last byte
			 * read
			 */
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 CntlReg | XIIC_CR_NO_ACK_MASK);
		}

		/* Read in data from the FIFO and unthrottle the bus such that
		 * the next byte is read from the IIC bus
		 */
		*BufferPtr++ = (uint8_t) XIic_ReadReg(BaseAddress,
						  XIIC_DRR_REG_OFFSET);

		if ((ByteCount == 1) && (Option == XIIC_REPEATED_START)) {

			/* RSTA bit should be set only when the FIFO is
			 * completely Empty.
			 */
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 XIIC_CR_ENABLE_DEVICE_MASK | XIIC_CR_MSMS_MASK
				 | XIIC_CR_REPEATED_START_MASK);

		}

		/* Clear the latched interrupt status so that it will be updated
		 * with the new state when it changes, this must be done after
		 * the receive register is read
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_RX_FULL_MASK |
				XIIC_INTR_TX_ERROR_MASK |
				XIIC_INTR_ARB_LOST_MASK);
		ByteCount--;
	}

	if (Option == XIIC_STOP) {

		/* If the Option is to release the bus after Reception of data,
		 * wait for the bus to transition to not busy before returning,
		 * the IIC device cannot be disabled until this occurs. It
		 * should transition as the MSMS bit of the control register was
		 * cleared before the last byte was read from the FIFO
		 */
		while (1) {
			if (XIic_ReadIisr(BaseAddress) & XIIC_INTR_BNB_MASK) {
				break;
			}
		}
	}

	return ByteCount;
}
/****************************************************************************/
/**
* Receive data as a master on the IIC bus.  This function receives the data
* using polled I/O and blocks until the data has been received. It only
* supports 7 bit addressing mode of operation. The user is responsible for
* ensuring the bus is not busy if multiple masters are present on the bus.
*
* @param	BaseAddress contains the base address of the IIC device.
* @param	Address contains the 7 bit IIC address of the device to send the
*		specified data to.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent.
* @param	Option indicates whether to hold or free the bus after reception
*		of data, XIIC_STOP = end with STOP condition,
*		XIIC_REPEATED_START = don't end with STOP condition.
*
* @return	The number of bytes received.
*
* @note		None.
*
******************************************************************************/
unsigned XIic_Recv(uint32_t BaseAddress, uint8_t Address,
			uint8_t *BufferPtr, unsigned ByteCount, uint8_t Option)
{
	uint32_t CntlReg;
	unsigned RemainingByteCount;
	volatile uint32_t StatusReg;

	/* Tx error is enabled incase the address (7 or 10) has no device to
	 * answer with Ack. When only one byte of data, must set NO ACK before
	 * address goes out therefore Tx error must not be enabled as it will go
	 * off immediately and the Rx full interrupt will be checked.  If full,
	 * then the one byte was received and the Tx error will be disabled
	 * without sending an error callback msg
	 */
	XIic_ClearIisr(BaseAddress,
			XIIC_INTR_RX_FULL_MASK | XIIC_INTR_TX_ERROR_MASK |
			XIIC_INTR_ARB_LOST_MASK);

	/* Set receive FIFO occupancy depth for 1 byte (zero based) */
	XIic_WriteReg(BaseAddress,  XIIC_RFD_REG_OFFSET, 0);


	/* Check to see if already Master on the Bus.
	 * If Repeated Start bit is not set send Start bit by setting MSMS bit
	 * else Send the address
	 */
	CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((CntlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/* 7 bit slave address, send the address for a read operation
		 * and set the state to indicate the address has been sent
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_READ_OPERATION);


		/* MSMS gets set after putting data in FIFO. Start the master
		 * receive operation by setting CR Bits MSMS to Master, if the
		 * buffer is only one byte, then it should not be acknowledged
		 * to indicate the end of data
		 */
		CntlReg = XIIC_CR_MSMS_MASK | XIIC_CR_ENABLE_DEVICE_MASK;
		if (ByteCount == 1) {
			CntlReg |= XIIC_CR_NO_ACK_MASK;
		}

		/* Write out the control register to start receiving data and
		 * call the function to receive each byte into the buffer
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, CntlReg);

		/* Clear the latched interrupt status for the bus not busy bit
		 * which must be done while the bus is busy
		 */
		StatusReg = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);

		while ((StatusReg & XIIC_SR_BUS_BUSY_MASK) == 0) {
			StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);
		}

		XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);
	} else {
	        /* Before writing 7bit slave address the Direction of Tx bit
		 * must be disabled
		 */
		CntlReg &= ~XIIC_CR_DIR_IS_TX_MASK;
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, CntlReg);
		/* Already owns the Bus indicating that its a Repeated Start
		 * call. 7 bit slave address, send the address for a read
		 * operation and set the state to indicate the address has been
		 * sent
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_READ_OPERATION);
	}
	/* Try to receive the data from the IIC bus */

	RemainingByteCount = RecvData(BaseAddress, BufferPtr,
				      ByteCount, Option);

	CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((CntlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/* The receive is complete, disable the IIC device if the Option
		 * is to release the Bus after Reception of data and return the
		 * number of bytes that was received
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, 0);
	}

	/* Return the number of bytes that was received */
	return ByteCount - RemainingByteCount;
}


/******************************************************************************
*
* Send the specified buffer to the device that has been previously addressed
* on the IIC bus.  This function assumes that the 7 bit address has been sent
* and it should wait for the transmit of the address to complete.
*
* @param	BaseAddress contains the base address of the IIC device.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent.
* @param	Option indicates whether to hold or free the bus after
*		transmitting the data.
*
* @return	The number of bytes remaining to be sent.
*
* @note
*
* This function does not take advantage of the transmit FIFO because it is
* designed for minimal code space and complexity.  It contains loops that
* that could cause the function not to return if the hardware is not working.
*
******************************************************************************/
static unsigned SendData(uint32_t BaseAddress, uint8_t *BufferPtr,
			 unsigned ByteCount, uint8_t Option)
{
	uint32_t IntrStatus;

	/*
	 * Send the specified number of bytes in the specified buffer by polling
	 * the device registers and blocking until complete
	 */
	while (ByteCount > 0) {
		/*
		 * Wait for the transmit to be empty before sending any more
		 * data by polling the interrupt status register
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);

			if (IntrStatus & (XIIC_INTR_TX_ERROR_MASK |
					  XIIC_INTR_ARB_LOST_MASK |
					  XIIC_INTR_BNB_MASK)) {
				return ByteCount;
			}

			if (IntrStatus & XIIC_INTR_TX_EMPTY_MASK) {
				break;
			}
		}
		/* If there is more than one byte to send then put the
		 * next byte to send into the transmit FIFO
		 */
		if (ByteCount > 1) {
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
				 *BufferPtr++);
		}
		else {
			if (Option == XIIC_STOP) {
				/*
				 * If the Option is to release the bus after
				 * the last data byte, Set the stop Option
				 * before sending the last byte of data so
				 * that the stop Option will be generated
				 * immediately following the data. This is
				 * done by clearing the MSMS bit in the
				 * control register.
				 */
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
					 XIIC_CR_ENABLE_DEVICE_MASK |
					 XIIC_CR_DIR_IS_TX_MASK);
			}

			/*
			 * Put the last byte to send in the transmit FIFO
			 */
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
				 *BufferPtr++);

			if (Option == XIIC_REPEATED_START) {
				XIic_ClearIisr(BaseAddress,
						XIIC_INTR_TX_EMPTY_MASK);
				/*
				 * Wait for the transmit to be empty before
				 * setting RSTA bit.
				 */
				while (1) {
					IntrStatus =
						XIic_ReadIisr(BaseAddress);
					if (IntrStatus &
						XIIC_INTR_TX_EMPTY_MASK) {
						/*
						 * RSTA bit should be set only
						 * when the FIFO is completely
						 * Empty.
						 */
						XIic_WriteReg(BaseAddress,
							 XIIC_CR_REG_OFFSET,
						   XIIC_CR_REPEATED_START_MASK |
						   XIIC_CR_ENABLE_DEVICE_MASK |
						   XIIC_CR_DIR_IS_TX_MASK |
						   XIIC_CR_MSMS_MASK);
						break;
					}
				}
			}
		}

		/*
		 * Clear the latched interrupt status register and this must be
		 * done after the transmit FIFO has been written to or it won't
		 * clear
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK);

		/*
		 * Update the byte count to reflect the byte sent and clear
		 * the latched interrupt status so it will be updated for the
		 * new state
		 */
		ByteCount--;
	}

	if (Option == XIIC_STOP) {
		/*
		 * If the Option is to release the bus after transmission of
		 * data, Wait for the bus to transition to not busy before
		 * returning, the IIC device cannot be disabled until this
		 * occurs. Note that this is different from a receive operation
		 * because the stop Option causes the bus to go not busy.
		 */
		while (1) {
			if (XIic_ReadIisr(BaseAddress) &
				XIIC_INTR_BNB_MASK) {
				break;
			}
		}
	}

	return ByteCount;
}

/****************************************************************************/
/**
* Send data as a master on the IIC bus.  This function sends the data
* using polled I/O and blocks until the data has been sent. It only supports
* 7 bit addressing mode of operation.  The user is responsible for ensuring
* the bus is not busy if multiple masters are present on the bus.
*
* @param	BaseAddress contains the base address of the IIC device.
* @param	Address contains the 7 bit IIC address of the device to send the
*		specified data to.
* @param	BufferPtr points to the data to be sent.
* @param	ByteCount is the number of bytes to be sent.
* @param	Option indicates whether to hold or free the bus after
* 		transmitting the data.
*
* @return	The number of bytes sent.
*
* @note		None.
*
******************************************************************************/
unsigned XIic_Send(uint32_t BaseAddress, uint8_t Address,
		   uint8_t *BufferPtr, unsigned ByteCount, uint8_t Option)
{
	unsigned RemainingByteCount;
	uint32_t ControlReg;
	volatile uint32_t StatusReg;

	/* Check to see if already Master on the Bus.
	 * If Repeated Start bit is not set send Start bit by setting
	 * MSMS bit else Send the address.
	 */
	ControlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((ControlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/*
		 * Put the address into the FIFO to be sent and indicate
		 * that the operation to be performed on the bus is a
		 * write operation
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_WRITE_OPERATION);
		/* Clear the latched interrupt status so that it will
		 * be updated with the new state when it changes, this
		 * must be done after the address is put in the FIFO
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK |
				XIIC_INTR_TX_ERROR_MASK |
				XIIC_INTR_ARB_LOST_MASK);

		/*
		 * MSMS must be set after putting data into transmit FIFO,
		 * indicate the direction is transmit, this device is master
		 * and enable the IIC device
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
			 XIIC_CR_MSMS_MASK | XIIC_CR_DIR_IS_TX_MASK |
			 XIIC_CR_ENABLE_DEVICE_MASK);

		/*
		 * Clear the latched interrupt
		 * status for the bus not busy bit which must be done while
		 * the bus is busy
		 */
		StatusReg = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);
		while ((StatusReg & XIIC_SR_BUS_BUSY_MASK) == 0) {
			StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);
		}

		XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);
	}
	else {
		/*
		 * Already owns the Bus indicating that its a Repeated Start
		 * call. 7 bit slave address, send the address for a write
		 * operation and set the state to indicate the address has
		 * been sent.
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_WRITE_OPERATION);
	}

	/* Send the specified data to the device on the IIC bus specified by the
	 * the address
	 */
	RemainingByteCount = SendData(BaseAddress, BufferPtr,
					ByteCount, Option);

	ControlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((ControlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/*
		 * The Transmission is completed, disable the IIC device if
		 * the Option is to release the Bus after transmission of data
		 * and return the number of bytes that was received. Only wait
		 * if master, if addressed as slave just reset to release
		 * the bus.
		 */
		if ((ControlReg & XIIC_CR_MSMS_MASK) != 0) {
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 (ControlReg & ~XIIC_CR_MSMS_MASK));
			StatusReg = XIic_ReadReg(BaseAddress,
					XIIC_SR_REG_OFFSET);
			while ((StatusReg & XIIC_SR_BUS_BUSY_MASK) != 0) {
				StatusReg = XIic_ReadReg(BaseAddress,
						XIIC_SR_REG_OFFSET);
			}
		}

		if ((XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET) &
		    XIIC_SR_ADDR_AS_SLAVE_MASK) != 0) {
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, 0);
		}
	}

	return ByteCount - RemainingByteCount;
}



#define IIC_BASE_ADDRESS 	0xFD070000
#define IIC_SLAVE_ADDRESS 	0x70
#define APB2JTAG_BASE  0xFD400000
#define APB2GPIO_BASE  0xFD030000
#define AXI_SW_BASE    0xFD080000
#define AXI_LED_BASE   (AXI_SW_BASE + 0x8)
#define XPAR_PCIE_MGMT_APB_BASEADDR  0xFB000000

#define PCIE_CORE_CONFIG_SPACE_BASE  (XPAR_PCIE_MGMT_APB_BASEADDR + 0x000000)
#define PCIE_CORE_LOCAL_MGMT_BASE    (XPAR_PCIE_MGMT_APB_BASEADDR + 0x100000)
#define PCIE_CORE_AXI_CONFIG_BASE    (XPAR_PCIE_MGMT_APB_BASEADDR + 0x400000)

int udelay(unsigned int val){
        volatile unsigned int i;
        for (i = 0; i < val*10; i++ );
}

void i2c_write(unsigned char REG_ADDR, unsigned int data)
{
	printf("i2c write %u\n", REG_ADDR);
    unsigned int rval=0;
        Xil_Out32((IIC_BASE_ADDRESS + XIIC_RFD_REG_OFFSET), 0x0F);
    Xil_Out32((IIC_BASE_ADDRESS + XIIC_CR_REG_OFFSET), 0x02);
        Xil_Out32((IIC_BASE_ADDRESS + XIIC_CR_REG_OFFSET), 0x01);
    Xil_Out32((IIC_BASE_ADDRESS + XIIC_DTR_REG_OFFSET), 0x1E0);
    Xil_Out32((IIC_BASE_ADDRESS + XIIC_DTR_REG_OFFSET), REG_ADDR);
    Xil_Out32((IIC_BASE_ADDRESS + XIIC_DTR_REG_OFFSET), 0x200 | data);
    while((rval & 0x80) != 0x80)  // wait until tx fifo goes empty
        {
                rval = Xil_In32(IIC_BASE_ADDRESS + XIIC_SR_REG_OFFSET);
                //kill_cyc(2);
                udelay(0x200);
        }
}

void reset_assert_uphy (void)
{
	//tc rst =0 apb/tap rst =0 phyreset =0 pipe rst = 0
	Xil_Out32 ((APB2GPIO_BASE + (0x30<<2)), 0x0);   // TOP_CHIP_RST_B
	Xil_Out32 ((APB2GPIO_BASE + (0x34<<2)), 0x0);   // UPHY_APB_PRESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	Xil_Out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x40<<2)), 0x0);   // UPHY_PIPE_RESET_N
}

void reset_assert_pcie (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x55<<2)), 0x0);   // SOFT_PIPE_RESET_N
}

void reset_deassert_pcie (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
}

void link_training_disable (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x43<<2)), 0x0);   // LINK_TRAINING_ENABLE
}

void link_training_enable (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
}


void uphy_bringup(void)
{
	unsigned int read_val=0;
	
	// When CRTL in Endpoint
	
#ifdef DEBUG_PRINT_ENABLE
	PRINT("UPHY Init Start\n\r");
#endif
	
	//tc rst =1 apb/tap rst =1 phyreset =1 pipe rst = 1
	Xil_Out32 ((APB2GPIO_BASE + (0x30<<2)), 0x1);   // TOP_CHIP_RST_B
	Xil_Out32 ((APB2GPIO_BASE + (0x34<<2)), 0x1);   // UPHY_APB_PRESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	Xil_Out32 ((APB2GPIO_BASE + (0x38<<2)), 0x1);   // UPHY_PHY_RESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x40<<2)), 0x1);   // UPHY_PIPE_RESET_N
	
	udelay (0x200);
	
	//tc rst =0 apb/tap rst =0 phyreset =0 pipe rst = 0
	Xil_Out32 ((APB2GPIO_BASE + (0x30<<2)), 0x0);   // TOP_CHIP_RST_B
	Xil_Out32 ((APB2GPIO_BASE + (0x34<<2)), 0x0);   // UPHY_APB_PRESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	Xil_Out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	Xil_Out32 ((APB2GPIO_BASE + (0x40<<2)), 0x0);   // UPHY_PIPE_RESET_N
	udelay (0x200);
	
	Xil_Out32 ((APB2GPIO_BASE + (0x31<<2)), 0x0);   // TOP_CHIPMODE for PCIe
	udelay (0x500);
	Xil_Out32 ((APB2GPIO_BASE + (0x32<<2)), 0x2);   // TOP_SEL_TAP 1x for TC
	udelay (0x500);
	
	//pipe rst = 1
	Xil_Out32 ((APB2GPIO_BASE + (0x40<<2)), 0x1);   // UPHY_PIPE_RESET_N
	udelay (0x500);
	
	Xil_Out32 ((APB2GPIO_BASE + (0x3B<<2)), 0x1);   // 32bit_sel
	Xil_Out32 ((APB2GPIO_BASE + (0x1C<<2)), 0x1);   // PIPE_L01_TX_ELEC_IDLE
	Xil_Out32 ((APB2GPIO_BASE + (0x29<<2)), 0x1);   // PIPE_L00_TX_ELEC_IDLE
	Xil_Out32 ((APB2GPIO_BASE + (0x2F<<2)), 0x2);   // Powerdown
	Xil_Out32 ((APB2GPIO_BASE + (0x3A<<2)), 0x1);   // UPHY_PHY_TX_CMN_MODE_EN
	Xil_Out32 ((APB2GPIO_BASE + (0x39<<2)), 0x1);   // UPHY_PHY_RX_ELEC_IDLE_DET_EN
	
	Xil_Out32 ((APB2GPIO_BASE + (0x20<<2)), 0x2);   // full rt clock 125mhz
	
	//apb rst =1
	Xil_Out32 ((APB2GPIO_BASE + (0x34<<2)), 0x1);   // UPHY_APB_PRESET_N
	udelay (0x50);
	
	//tc rst =1
	Xil_Out32 ((APB2GPIO_BASE + (0x30<<2)), 0x1);   // TOP_CHIP_RST_B
	udelay (0x50);
	
	//tap rst = 1
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	Xil_Out32 ((APB2JTAG_BASE + (0x0024<<4)), 0x3);   //uphy_t28_comp__TC_UPHY_CTRL_REG_15
	udelay (0x500);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x0024<<4)));
	//xil_printf(" uphy_t28_comp__TC_UPHY_CTRL_REG_15 = 0x%X \r\n", read_val);
	
	//tap rst = 0
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x0);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	//phy reset =0
	Xil_Out32 ((APB2GPIO_BASE + (0x38<<2)), 0x0);   // UPHY_PHY_RESET_N
	udelay (0x500);
	
	Xil_Out32 ((APB2GPIO_BASE + (0x32<<2)), 0x0);   // TOP_SEL_TAP 0x for IP
	udelay (0x500);
	
	//tap rst = 1
	Xil_Out32 ((APB2GPIO_BASE + (0x4C<<2)), 0x1);   // UPHY_TAP_TRST_N
	udelay (0x50);
	
	//phy reset =1
	Xil_Out32 ((APB2GPIO_BASE + (0x38<<2)), 0x1);   // UPHY_PHY_RESET_N
	udelay (0x500);
	
	
	Xil_Out32 ((APB2JTAG_BASE + (0xC800<<4)), 0x3800);   //PHY_PMA_CMN_CTRL1  differential clock selection
	udelay (0x500);
	
	//Xil_Out32 ((APB2JTAG_BASE + (0x0022<<4)), 0x0040);   //CMN_SSM_BIAS_TMR only for Gen2
	//udelay (0x10000);
	
	
	read_val = 0;
	while (!(read_val & 0x0001) )
	{
	read_val = Xil_In32 ((APB2JTAG_BASE + (0xC800<<4)));  //wait for PHY_PMA_CMN_CTRL1[0] === 1
	//xil_printf(" PHY_PMA_CMN_CTRL1 = 0x%X \r\n", read_val);
	}
	udelay (0x50);
	
	Xil_Out32 ((APB2JTAG_BASE + (0xcc10<<4)), 0x0020);   //for lane 0    PHY_PMA_ISO_XCVR_CTRL
	udelay (0x50);
	Xil_Out32 ((APB2JTAG_BASE + (0xcc50<<4)), 0x0020);   //for lane 1    PHY_PMA_ISO_XCVR_CTRL
	udelay (0x50);
	
	Xil_Out32 ((APB2JTAG_BASE + (0x8004<<4)), 0x1010);   //RX_PSC_A4
	udelay (0x50);
	Xil_Out32 ((APB2JTAG_BASE + (0x8080<<4)), 0x2AB3);   //RX_CDRLF_CNFG
	udelay (0x50);
	Xil_Out32 ((APB2JTAG_BASE + (0x819D<<4)), 0x8014);   //RX_REE_PEAK_COVRD - 0x8014
	udelay (0x50);
	Xil_Out32 ((APB2JTAG_BASE + (0x81BB<<4)), 0x4080);   //RX_REE_CTRL_DATA_MASK
	udelay (0x50);
	
#ifdef DEBUG_PRINT_ENABLE
	//Reading all the register
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x8004<<4)));
	xil_printf(" RX_PSC_A4 = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x8080<<4)));
	xil_printf(" RX_CDRLF_CNFG = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x819D<<4)));
	xil_printf(" RX_REE_PEAK_COVRD = 0x%X \r\n", read_val);
	
	read_val = Xil_In32 ((APB2JTAG_BASE + (0x81BB<<4)));
	xil_printf(" RX_REE_CTRL_DATA_MASK = 0x%X \r\n", read_val);
	
	PRINT("\nUPHY Init Done\n\r");
#endif
	
	//Xil_Out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
	//Xil_Out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
	
	//Xil_Out32 ((APB2GPIO_BASE + (0x1C<<2)), 0x0);   // PIPE_L01_TX_ELEC_IDLE
	//Xil_Out32 ((APB2GPIO_BASE + (0x29<<2)), 0x0);   // PIPE_L00_TX_ELEC_IDLE
	
	//When ctrl in RC
	//Xil_Out32 ((APB2GPIO_BASE + (0x56<<2)), 0x1);   // perst_o
	//Xil_Out32 ((APB2GPIO_BASE + (0x57<<2)), 0x1);   // RP_EP_MODE_SEL
	
	//Xil_Out32 ((APB2GPIO_BASE + (0x43<<2)), 0x1);   // LINK_TRAINING_ENABLE
	//Xil_Out32 ((APB2GPIO_BASE + (0x55<<2)), 0x1);   // SOFT_PIPE_RESET_N
	
	udelay (0x50);
	

}

void pcie_mode_root_complex (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x57<<2)), 0x1);
}

void pcie_mode_end_point (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x57<<2)), 0x0);
}

void perstn_assert (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x56<<2)), 0x0);
}

void perstn_deassert (void)
{
	Xil_Out32 ((APB2GPIO_BASE + (0x56<<2)), 0x1);
}

void wait_perstn_deassert (void)
{
	unsigned int read_val=0;
	
	while (!(read_val & 0x0001) )
	{
		read_val = Xil_In32 ((APB2GPIO_BASE + (0x0A<<2)));  //wait for perst_i === 1
	}
}

void wait_perstn_assert (void)
{
	unsigned int read_val=0;
	
	while ((read_val & 0x0001) )
	{
		read_val = Xil_In32 ((APB2GPIO_BASE + (0x0A<<2)));  //wait for perst_i === 1
	}
}

void wait_link_training_done (void)
{
	unsigned int read_val=0;
	
	while (!(read_val & 0x0001) )
	{
		read_val = Xil_In32 (PCIE_CORE_LOCAL_MGMT_BASE + 0x00);
		udelay (0x50);
	}
	//Xil_Out32 ((AXI_LED_BASE), 0x05);
}

//IIC Defines
#define IIC_GIE               0x001C
#define IIC_ISR               0x0020
#define IIC_IER               0x0028
#define IIC_RE                0x0040
#define IIC_CR                0x0100
#define IIC_SR                0x0104
#define IIC_TX_FIFO           0x0108
#define IIC_RX_FIFO           0x010C
#define IIC_SL_ADR            0x0110
#define IIC_TX_FIFO_OCY       0x0114
#define IIC_RX_FIFO_OCY       0x0118
#define IIC_SL_ADR_TEN        0x011C
#define IIC_RX_FIFO_PIRQ      0x0120
#define IIC_GPO               0x0124
#define IIC_BASE              0xFD070000    // map it to the actual IIC base address in the design


void kill_cyc(unsigned int val) {
  unsigned int mc;
  for (mc = 0; mc < ((val * 3000) + 1); mc++)
  {
         // __asm__ __volatile__ ("nop");
  }
}

unsigned int iic_read(unsigned char REG_ADDR)
{
    unsigned int rval=0x40;
    Xil_Out32((IIC_BASE + IIC_RX_FIFO_PIRQ), 0x0F);
    Xil_Out32((IIC_BASE + IIC_CR), 0x02);
    Xil_Out32((IIC_BASE + IIC_CR), 0x01);
    Xil_Out32((IIC_BASE + IIC_TX_FIFO), 0x1E0);
    Xil_Out32((IIC_BASE + IIC_TX_FIFO), REG_ADDR);
    Xil_Out32((IIC_BASE + IIC_TX_FIFO), 0x1E1);
    Xil_Out32((IIC_BASE + IIC_TX_FIFO), 0x201);
    //while((rval & 0x40) == 0x40)  // wait unitl rx fifo gets some data
        {
                rval = Xil_In32(IIC_BASE + IIC_SR);
                kill_cyc(2);
        }

    rval = Xil_In32(IIC_BASE + IIC_RX_FIFO);
    return rval;
}
	
struct iic_data{
	uint8_t addr;
	uint8_t data;
};	 			
		
struct iic_data si_iic_data[] =  {
	{0xE6,0x1F},
	{0xF1,0x85},
	{0xFF,0x00},
	
	{0x0,0x00},
	{0x1,0x00},
	{0x2,0x00},
	{0x3,0x00},
	{0x4,0x00},
	{0x5,0x00},
	{0x6,0x04},
	{0x7,0x00},
	{0x8,0x70},
	{0x9,0x0F},
	{0xA,0x00},
	{0xB,0x00},
	{0xC,0x00},
	{0xD,0x00},
	{0xE,0x00},
	{0xF,0x00},
	{0x10,0x00},
	{0x11,0x00},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x00},
	{0x15,0x00},
	{0x16,0x00},
	{0x17,0x00},
	{0x18,0x00},
	{0x19,0x00},
	{0x1A,0x00},
	{0x1B,0x70},
	{0x1C,0x03},
	{0x1D,0x62},
	{0x1E,0xA2},
	{0x1F,0x02},
	{0x20,0x02},
	{0x21,0x02},
	{0x22,0x02},
	{0x23,0xAA},
	{0x24,0x07},
	{0x25,0x07},
	{0x26,0x07},
	{0x27,0x07},
	{0x28,0xE7},
	{0x29,0x1C},
	{0x2A,0x27},
	{0x2B,0x00},
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0x00},
	{0x2F,0x14},
	{0x30,0x35},
	{0x31,0x00},
	{0x32,0x03},
	{0x33,0x07},
	{0x34,0x10},
	{0x35,0x00},
	{0x36,0x0B},
	{0x37,0x00},
	{0x38,0x00},
	{0x39,0x00},
	{0x3A,0x00},
	{0x3B,0x01},
	{0x3C,0x00},
	{0x3D,0x00},
	{0x3E,0x00},
	{0x3F,0x10},
	{0x40,0x00},
	{0x41,0x0B},
	{0x42,0x00},
	{0x43,0x00},
	{0x44,0x00},
	{0x45,0x00},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0x00},
	{0x49,0x00},
	{0x4A,0x10},
	{0x4B,0x00},
	{0x4C,0x0B},
	{0x4D,0x00},
	{0x4E,0x00},
	{0x4F,0x00},
	{0x50,0x00},
	{0x51,0x01},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x00},
	{0x55,0x10},
	{0x56,0x00},
	{0x57,0x0B},
	{0x58,0x00},
	{0x59,0x00},
	{0x5A,0x00},
	{0x5B,0x00},
	{0x5C,0x01},
	{0x5D,0x00},
	{0x5E,0x00},
	{0x5F,0x00},
	{0x60,0x10},
	{0x61,0x00},
	{0x62,0x32},
	{0x63,0x00},
	{0x64,0x00},
	{0x65,0x00},
	{0x66,0x00},
	{0x67,0x01},
	{0x68,0x00},
	{0x69,0x00},
	{0x6A,0x80},
	{0x6B,0x00},
	{0x6C,0x00},
	{0x6D,0x00},
	{0x6E,0x40},
	{0x6F,0x00},
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x40},
	{0x73,0x00},
	{0x74,0x80},
	{0x75,0x00},
	{0x76,0x40},
	{0x77,0x00},
	{0x78,0x00},
	{0x79,0x00},
	{0x7A,0x40},
	{0x7B,0x00},
	{0x7C,0x00},
	{0x7D,0x00},
	{0x7E,0x00},
	{0x7F,0x00},
	{0x80,0x00},
	{0x81,0x00},
	{0x82,0x00},
	{0x83,0x00},
	{0x84,0x00},
	{0x85,0x00},
	{0x86,0x00},
	{0x87,0x00},
	{0x88,0x00},
	{0x89,0x00},
	{0x8A,0x00},
	{0x8B,0x00},
	{0x8C,0x00},
	{0x8D,0x00},
	{0x8E,0x00},
	{0x8F,0x00},
	{0x90,0x00},
	{0x91,0x00},
	{0x92,0xFF},
	{0x93,0x00},
	{0x94,0x00},
	{0x95,0x00},
	{0x96,0x00},
	{0x97,0x00},
	{0x98,0x00},
	{0x99,0x00},
	{0x9A,0x00},
	{0x9B,0x00},
	{0x9C,0x00},
	{0x9D,0x00},
	{0x9E,0x00},
	{0x9F,0x00},
	{0xA0,0x00},
	{0xA1,0x00},
	{0xA2,0x00},
	{0xA3,0x00},
	{0xA4,0x00},
	{0xA5,0x00},
	{0xA6,0x00},
	{0xA7,0x00},
	{0xA8,0x00},
	{0xA9,0x00},
	{0xAA,0x00},
	{0xAB,0x00},
	{0xAC,0x00},
	{0xAD,0x00},
	{0xAE,0x00},
	{0xAF,0x00},
	{0xB0,0x00},
	{0xB1,0x00},
	{0xB2,0x00},
	{0xB3,0x00},
	{0xB4,0x00},
	{0xB5,0x00},
	{0xB6,0x00},
	{0xB7,0x00},
	{0xB8,0x00},
	{0xB9,0x00},
	{0xBA,0x00},
	{0xBB,0x00},
	{0xBC,0x00},
	{0xBD,0x00},
	{0xBE,0x00},
	{0xBF,0x00},
	{0xC0,0x00},
	{0xC1,0x00},
	{0xC2,0x00},
	{0xC3,0x00},
	{0xC4,0x00},
	{0xC5,0x00},
	{0xC6,0x00},
	{0xC7,0x00},
	{0xC8,0x00},
	{0xC9,0x00},
	{0xCA,0x00},
	{0xCB,0x00},
	{0xCC,0x00},
	{0xCD,0x00},
	{0xCE,0x00},
	{0xCF,0x00},
	{0xD0,0x00},
	{0xD1,0x00},
	{0xD2,0x00},
	{0xD3,0x00},
	{0xD4,0x00},
	{0xD5,0x00},
	{0xD6,0x00},
	{0xD7,0x00},
	{0xD8,0x00},
	{0xD9,0x00},
	{0xDA,0x00},
	{0xDB,0x00},
	{0xDC,0x00},
	{0xDD,0x0D},
	{0xDE,0x00},
	{0xDF,0x00},
	{0xE0,0xF4},
	{0xE1,0xF0},
	{0xE2,0x00},
	{0xE3,0x00},
	{0xE4,0x00},
	{0xE5,0x00},
	{0xE6,0x00},
	{0xE7,0x00},
	{0xE8,0x00},
	{0xE9,0x00},
	{0xEA,0x00},
	{0xEB,0x00},
	{0xEC,0x00},
	{0xED,0x00},
	{0xEE,0x14},
	{0xEF,0x00},
	{0xF0,0x00},
	{0xF2,0x00},
	{0xF3,0xF0},
	{0xF4,0x00},
	{0xF5,0x00},
	{0xF7,0x00},
	{0xF8,0x00},
	{0xF9,0xA8},
	{0xFA,0x00},
	{0xFB,0x84},
	{0xFC,0x00},
	{0xFD,0x00},
	{0xFE,0x00},
	{0xFF,0xFF},
	
	{0x0,0x00},
	{0x1,0x00},
	{0x2,0x00},
	{0x3,0x00},
	{0x4,0x00},
	{0x5,0x00},
	{0x6,0x00},
	{0x7,0x00},
	{0x8,0x00},
	{0x9,0x00},
	{0xA,0x00},
	{0xB,0x00},
	{0xC,0x00},
	{0xD,0x00},
	{0xE,0x00},
	{0xF,0x00},
	{0x10,0x00},
	{0x11,0x01},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x90},
	{0x15,0x31},
	{0x16,0x00},
	{0x17,0x00},
	{0x18,0x01},
	{0x19,0x00},
	{0x1A,0x00},
	{0x1B,0x00},
	{0x1C,0x00},
	{0x1D,0x00},
	{0x1E,0x00},
	{0x1F,0x00},
	{0x20,0x00},
	{0x21,0x01},
	{0x22,0x00},
	{0x23,0x00},
	{0x24,0x90},
	{0x25,0x31},
	{0x26,0x00},
	{0x27,0x00},
	{0x28,0x01},
	{0x29,0x00},
	{0x2A,0x00},
	{0x2B,0x00},
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0x00},
	{0x2F,0x00},
	{0x30,0x00},
	{0x31,0x01},
	{0x32,0x00},
	{0x33,0x00},
	{0x34,0x90},
	{0x35,0x31},
	{0x36,0x00},
	{0x37,0x00},
	{0x38,0x01},
	{0x39,0x00},
	{0x3A,0x00},
	{0x3B,0x00},
	{0x3C,0x00},
	{0x3D,0x00},
	{0x3E,0x00},
	{0x3F,0x00},
	{0x40,0x00},
	{0x41,0x01},
	{0x42,0x00},
	{0x43,0x00},
	{0x44,0x90},
	{0x45,0x31},
	{0x46,0x00},
	{0x47,0x00},
	{0x48,0x01},
	{0x49,0x00},
	{0x4A,0x00},
	{0x4B,0x00},
	{0x4C,0x00},
	{0x4D,0x00},
	{0x4E,0x00},
	{0x4F,0x00},
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x90},
	{0x55,0x31},
	{0x56,0x00},
	{0x57,0x00},
	{0x58,0x01},
	{0x59,0x00},
	{0x5A,0x00},
	{0x5B,0x00},
	{0x5C,0x00},
	{0x5D,0x00},
	{0x5E,0x00}
};

int main (){
	
	int result, val, i;			
	uint32_t StatusReg;

	printf("Enter Main\n\r");
	pcie_mode_root_complex();
	perstn_assert();
	
	printf("start si5338_bring_up\n\r");
	printf("XLNX IIC test Init \n");				

	result = XIic_DynInit(IIC_BASE_ADDRESS);

	if (result != XST_SUCCESS) {
					printf ("ERROR in dynamic init 0x%x \n", result);
					return 0;

	}

	/*
	 * Make sure all the Fifo's are cleared and Bus is Not busy.
	 */
	printf("Status clear taking time\n");
	while (((StatusReg = XIic_ReadReg(IIC_BASE_ADDRESS,
				XIIC_SR_REG_OFFSET)) &
				(XIIC_SR_RX_FIFO_EMPTY_MASK |
				XIIC_SR_TX_FIFO_EMPTY_MASK |
				XIIC_SR_BUS_BUSY_MASK)) !=
				(XIIC_SR_RX_FIFO_EMPTY_MASK |
				XIIC_SR_TX_FIFO_EMPTY_MASK)) {

	}

	size_t size;
	printf("Total Size of %d\n", size = sizeof(si_iic_data));
	printf("Size of each %d\n", sizeof(si_iic_data[0]));
	
	for (i = 0; i < size/2; i++){
                result =  XIic_Send(IIC_BASE_ADDRESS, IIC_SLAVE_ADDRESS,
                               &si_iic_data[i], sizeof(si_iic_data[i]), XIIC_STOP);
		if(result >= 1){
			printf("Err at Addr %d Data %d count %d res %d\n", si_iic_data[i].addr, si_iic_data[i].data, i, result);
		}	
	}
	uphy_bringup();
	printf("done\n\r");
	
	perstn_deassert();
	
	link_training_enable();
	
	reset_deassert_pcie ();
	
	wait_link_training_done();
	printf("link_training_done\n\r");
	udelay(0x100);
	return 0;
}
