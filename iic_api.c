/*This file is inteded for Xilinx IIC on generic platform 
 * original Please read the READ me before proceeding further  
 *
 * */

#include <linux/kernel.h>
#include <linux/printk.h>

#include <platform/hardware.h>
#include <platform/lcd.h>

#include <platform/iic_api.h>
#include <platform/si5338_uphy.h>

#define printf printk

#ifdef IIC_DYNAMIC_FUNC
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
#endif


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

