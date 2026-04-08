/******************************************************************************
* Copyright (C) 2010 - 2021 Xilinx, Inc.  All rights reserved.
* Copyright (C) 2022 - 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/****************************************************************************/
/**
* @file     xuartps_low_echo_example.c
*
* This file contains a design example using the hardware interface.
*
* First, certain character sequence is output onto the terminal. Then any
* characters typed in are echoed back, for letters, cases are switched.
* An 'ESC' character terminates the execution of the example.
*
* This example requires an external SchmartModule to be connected to the
* appropriate pins for the device through a daughter board. The test uses
* the default settings of the device:
*	. baud rate of 9600
*	. 8 bits data
* 	. 1 stop bit
* 	. no parity
*
* @note
* The test hangs if communication channel from the user terminal to the device
* is broken.
*
* MODIFICATION HISTORY:
* <pre>
* Ver   Who    Date     Changes
* ----- ------ -------- -----------------------------------------------------
* 1.00a drg/jz 01/13/10 First release
* 3.4   ms     01/23/17 Added xil_printf statement in main function to
*                       ensure that "Successfully ran" and "Failed" strings
*                       are available in all examples. This is a fix for
*                       CR-965028.
* </pre>
****************************************************************************/

/***************************** Include Files *******************************/

#include "xparameters.h"
#include "xstatus.h"
#include "xil_types.h"
#include "xil_assert.h"
#include "xuartps_hw.h"
#include "xil_printf.h"

/************************** Constant Definitions ***************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define UART_BASEADDR		XPAR_XUARTPS_0_BASEADDR
#define UART_CLOCK_HZ		XPAR_XUARTPS_0_CLOCK_HZ
/*
 * The following constant controls the length of the buffers to be sent
 * and received with the device. This constant must be 32 bytes or less so the
 * entire buffer will fit into the TX and RX FIFOs of the device.
 */
#define TEST_BUFFER_SIZE	16

#define CHAR_ESC		0x1b	/* 'ESC' character is used as terminator */

#define A_COLS 8
#define A_ROWS 64
/**************************** Type Definitions *****************************/

/***************** Macros (Inline Functions) Definitions *******************/

/************************** Function Prototypes ****************************/

int UartPsEchoExample(u32 UartBaseAddress);

/************************** Variable Definitions ***************************/

/*
 * The following buffers are used in this example to send and receive data
 * with the UART.
 */
u8 SendBuffer[TEST_BUFFER_SIZE];	/* Buffer for Transmitting Data */


/***************************************************************************/
/**
*
* Main function to call the Uart Echo example.
*
*
* @return	XST_SUCCESS if successful, XST_FAILURE if unsuccessful
*
* @note		None
*
****************************************************************************/
int main(void)
{
	int Status;

	/*
	 * Run the Uart Echo example , specify the Base Address that is
	 * generated in xparameters.h
	 */
	Status = UartPsEchoExample(UART_BASEADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("Uartps low echo Example Failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran Uartps low echo Example\r\n");
	return XST_SUCCESS;
}


/**************************************************************************/
/**
*
* This function does a minimal test on the UART device using the hardware
* interface.
*
* @param	UartBaseAddress is the base address of the device
*
* @return	XST_SUCCESS if successful, XST_FAILURE if unsuccessful
*
* @note		None.
*
**************************************************************************/
int UartPsEchoExample(u32 UartBaseAddress)
{
	int Index;
	u32 Running;
	u8 RecvChar;
	u8 RecArray[] = {0,0,0,0};
	int counter =0;
	u32 CntrlRegister;
	u8 A_Matrix[A_ROWS*A_COLS] = {0};
    u8 B_Matrix[A_COLS] = {0};
    u8 result[A_ROWS] = {0};

	CntrlRegister = XUartPs_ReadReg(UartBaseAddress, XUARTPS_CR_OFFSET);

	/* Enable TX and RX for the device */
	XUartPs_WriteReg(UartBaseAddress, XUARTPS_CR_OFFSET,
			  ((CntrlRegister & ~XUARTPS_CR_EN_DIS_MASK) |
			   XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN));
	/*
	 * Initialize the send buffer bytes with a pattern to send and the
	 * the receive buffer bytes to zero
	 */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		SendBuffer[Index] = Index + '0';
	}

	/* Send the entire transmit buffer. */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		/* Wait until there is space in TX FIFO */
		 while (XUartPs_IsTransmitFull(UartBaseAddress));

		/* Write the byte into the TX FIFO */
		XUartPs_WriteReg(UartBaseAddress, XUARTPS_FIFO_OFFSET,
				  SendBuffer[Index]);
	}

	Running = TRUE;
	while (Running) {
		 /* Wait until there is data */
		while (!XUartPs_IsReceiveData(UartBaseAddress));
		RecvChar = XUartPs_ReadReg(UartBaseAddress,
							XUARTPS_FIFO_OFFSET);
		// if (CHAR_ESC == RecvChar) {
		// 	Running = FALSE;
		// }
		if(counter<A_COLS*A_ROWS){
			A_Matrix[counter] = RecvChar;
			counter+=1;
		} else if(counter<A_COLS*(A_ROWS+1)){
			B_Matrix[counter-A_ROWS*A_COLS] = RecvChar;
			counter+=1;
		}
		if(counter>=A_COLS*(A_ROWS+1)){
			counter = 0;
			
			// for(int out_i =0;out_i<A_ROWS*A_COLS;out_i++){
			// 	XUartPs_WriteReg(UartBaseAddress,  XUARTPS_FIFO_OFFSET,
			// 		A_Matrix[out_i]*2);
			// }
			
			for(int r= 0; r < A_ROWS; r++) {
				int sum = 0;
				for(int c=0; c < A_COLS; c++) {
					sum += A_Matrix[r * A_COLS + c] * B_Matrix[c];
				
					// Storing the result back in result array for demonstration
				}
				result[r] = sum/256;
			}
			for(int out_i =0;out_i<A_ROWS;out_i++){
				XUartPs_WriteReg(UartBaseAddress,  XUARTPS_FIFO_OFFSET,
					result[out_i]);
			}
			
		}

		// /* Change the capitalization */
		// if (('a' <= RecvChar) && ('z' >= RecvChar)) {
		// 	/* Convert the Capital letter to a small. */
		// 	RecvChar = RecvChar - 'a' + 'A';
		// }
		// else if (('A' <= RecvChar) && ('Z' >= RecvChar)) {
		// 	/* Convert the small letter to a Capital. */
		// 	RecvChar = RecvChar - 'A' + 'a';
		// }
		

		/* Echo the character back */
		
	}

	return XST_SUCCESS;
}


// int matrix_multiply( int A_Matrix[], int B_Matrix[],int result[]) {

//     for(int r= 0; r < A_ROWS; r++) {
//         int sum = 0;
//         for(int c=0; c < A_COLS; c++) {
//             sum += A_Matrix[r * A_COLS + c] * B_Matrix[c];
        
//             // Storing the result back in result array for demonstration
//         }
//         result[r] = sum;
//     }
//     return 0;
// }