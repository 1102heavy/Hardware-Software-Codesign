#include "xparameters.h"
#include "xil_exception.h"
#include "xstreamer.h"
#include "xil_cache.h"
#include "xllfifo.h"
#include "xstatus.h"
#include "xil_types.h"
#include "xil_assert.h"
#include "xuartps_hw.h"
#include "xil_printf.h"
#include <stdint.h>
#include "xtmrctr.h"

#include "xaxidma.h"
#include "xdebug.h"

#ifndef DDR_BASE_ADDR
#warning CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H, \
DEFAULT SET TO 0x01000000
#define MEM_BASE_ADDR		0x01000000
#else
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x1000000)
#endif

// Transmit and receive buffer allocated sufficiently away from the start of DDR, hopefully not overlapping with the program's other memory segments.
// It is better to hard code transmit and receive buffers to avoid it being in the same cache line as other variables, and for better alignment.
#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000) 
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)
#define RX_BUFFER_HIGH		(MEM_BASE_ADDR + 0x004FFFFF)

/***************** Macros *********************/
#define NUMBER_OF_INPUT_WORDS 520  // length of an input vector
#define NUMBER_OF_OUTPUT_WORDS 64  // length of an input vector
#define NUMBER_OF_TEST_VECTORS 1  // number of such test vectors (cases)

#define DMA_DEV_ID        XPAR_XAXIDMA_0_BASEADDR


#define TMR_NUM 0            // use Timer 0 inside the AXI Timer IP

#ifdef XPAR_UARTNS550_0_BASEADDR
#include "xuartns550_l.h"       /* to use uartns550 */
#endif

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

#ifndef SDT
#define FIFO_DEV_ID	   	XPAR_AXI_FIFO_0_DEVICE_ID
#endif

#define WORD_SIZE 4			/* Size of words in bytes */

#define MAX_PACKET_LEN 4

#define NO_OF_PACKETS 130

#define A_COLS 8
#define A_ROWS 64
#define B_COLS 1
#define B_ROWS 8


#define TOTAL_MATRIX_A_ELEMENTS ((A_COLS * A_ROWS))
#define TOTAL_MATRIX_B_ELEMENTS ((B_COLS * B_ROWS))
#define TOTAL_MATRIX_ELEMENTS  ((TOTAL_MATRIX_B_ELEMENTS + TOTAL_MATRIX_A_ELEMENTS))

#define MAX_DATA_BUFFER_SIZE NO_OF_PACKETS*MAX_PACKET_LEN



//------------------------------------------------------------------------------------

#define UART_BASEADDR		XPAR_XUARTPS_0_BASEADDR
#define UART_CLOCK_HZ		XPAR_XUARTPS_0_CLOCK_HZ

#define TEST_BUFFER_SIZE	16

#define CHAR_ESC		0x1b	/* 'ESC' character is used as terminator */



#undef DEBUG

uint8_t idx = 0;
uint8_t test1 = 0;
uint8_t test2 = 0;

//-----------------------------------------------------------------
//------------------------Timer setup-----------------------------
//-----------------------------------------------------------------
#ifndef SDT
#define TMRCTR_DEVICE_ID	XPAR_TMRCTR_0_DEVICE_ID
#else
#define XTMRCTR_BASEADDRESS	XPAR_XTMRCTR_0_BASEADDR
#endif


/*
 * This example only uses the 1st of the 2 timer counters contained in a
 * single timer counter hardware device
 */
#define TIMER_COUNTER_0	 0


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
#ifndef SDT
int TmrCtrPolledExample(u16 DeviceId, u8 TmrCtrNumber);
#else
int TmrCtrPolledExample(UINTPTR BaseAddr, u8 TmrCtrNumber);
#endif
/************************** Variable Definitions *****************************/

XTmrCtr TimerCounter; /* The instance of the Tmrctr Device */

XTmrCtr *TmrCtrInstancePtr = &TimerCounter;
XAxiDma AxiDma;	// Device instance
XAxiDma *InstancePtr = &AxiDma; // Device pointer

// Commenting out the lines below as we are using hardcoded buffers, and not compiler/linker allocated.
//int test_input_memory [NUMBER_OF_TEST_VECTORS*NUMBER_OF_INPUT_WORDS] = {0x01, 0x02, 0x03, 0x04, 0x02, 0x03, 0x04, 0x05}; // 4 inputs * 2
//int result_memory [NUMBER_OF_TEST_VECTORS*NUMBER_OF_OUTPUT_WORDS]; // same size as test_result_expected_memory
int test_result_expected_memory [NUMBER_OF_TEST_VECTORS*NUMBER_OF_OUTPUT_WORDS];// 4 outputs *2

//-----------------------------------------------------------------
//---------------------Timer setup ends ----------------------------------------
//-----------------------------------------------------------------


/************************** Function Prototypes ******************************/
#ifdef XPAR_UARTNS550_0_BASEADDR
static void Uart550_Setup(void);
#endif

#ifndef SDT
int XLlFifoPollingExample(XLlFifo *InstancePtr, u16 DeviceId, u32 UART_BASEADDR);
#else
int XLlFifoPollingExample(XLlFifo *InstancePtr, UINTPTR BaseAddress);
#endif

int TxSend(XLlFifo *InstancePtr, u32 *SourceAddr);
int RxReceive(XLlFifo *InstancePtr, u32 *DestinationAddr);

//------------------------------------------------------------------------------------

int UartPsEchoExample(u32 UartBaseAddress);

uint8_t uart_getc(u32 base);
int uart_read_u8_csv(u32 base, u8 *out, u32 *buffer);

void uart_put_u32_dec(u32 base, u32 v);
void uart_puts(u32 base, const char *s);
void uart_putc(u32 base, u8 c);



u8 SendBuffer[TEST_BUFFER_SIZE];	/* Buffer for Transmitting Data */

/************************** Variable Definitions *****************************/
/*
 * Device instance definitions
 */
XLlFifo FifoInstance;

u32 SourceBuffer[TOTAL_MATRIX_ELEMENTS] = {0};
u32 DestinationBuffer[MAX_DATA_BUFFER_SIZE * WORD_SIZE];

/*****************************************************************************/
/**
*
* Main function
*
* This function is the main entry of the Axi FIFO Polling test.
*
* @param	None
*
* @return
*		- XST_SUCCESS if tests pass
* 		- XST_FAILURE if fails.
*
* @note		None
*
******************************************************************************/
int main()
{
	xil_printf("beginning of programme\n\r");
	int Status1;
	int Status2;

	
#ifndef SDT
    xil_printf("Timer initialising1..,\n\r");
	Status2 = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	xil_printf("Timer initialisiation successful\n\r");
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TIMER_COUNTER_0,
			XTC_AUTO_RELOAD_OPTION);
    XTmrCtr_Start(TmrCtrInstancePtr,  TIMER_COUNTER_0);	
	
	Status1 = XLlFifoPollingExample(&FifoInstance, FIFO_DEV_ID, UART_BASEADDR);
#else
    xil_printf("Timer initialising1.1\n\r");
	Status2 = XTmrCtr_Initialize(TmrCtrInstancePtr, XTMRCTR_BASEADDRESS);
	xil_printf("Timer initialisiation successful\n\r");
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TIMER_COUNTER_0,
			XTC_AUTO_RELOAD_OPTION);
	XTmrCtr_Start(TmrCtrInstancePtr,  TIMER_COUNTER_0);	
		
	Status1 = XLlFifoPollingExample(&FifoInstance, XPAR_XLLFIFO_0_BASEADDR);
#endif
	if (Status1 != XST_SUCCESS | Status2 != XST_SUCCESS) {
		xil_printf("Axi Streaming FIFO Polling Example Test Failed\n\r");
		xil_printf("--- Exiting main() ---\n\r");
		return XST_FAILURE;
	}



	xil_printf("Successfully ran Axi Streaming FIFO Polling Example\n\r");
	xil_printf("--- Exiting main() ---\n\r");

		

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function demonstrates the usage AXI FIFO
* It does the following:
*       - Set up the output terminal if UART16550 is in the hardware build
*       - Initialize the Axi FIFO Device.
*	- Transmit the data
*	- Receive the data from fifo
*	- Compare the data
*	- Return the result
*
* @param	InstancePtr is a pointer to the instance of the
*		XLlFifo component.
* @param	DeviceId is Device ID of the Axi Fifo Device instance,
*		typically XPAR_<AXsI_FIFO_instance>_DEVICE_ID value from
*		xparameters.h.
*
* @return
*		-XST_SUCCESS to indicate success
*		-XST_FAILURE to indicate failure
*
******************************************************************************/
#ifndef SDT
int XLlFifoPollingExample(XLlFifo *InstancePtr, u16 DeviceId, u32 UART_BASEADDR)
#else
int XLlFifoPollingExample(XLlFifo *InstancePtr, UINTPTR BaseAddress)
#endif
{
	XLlFifo_Config *Config;
	int Status;
	int i;
	int Error;
	Status = XST_SUCCESS;

	/* Initial setup for Uart16550 */
#ifdef XPAR_UARTNS550_0_BASEADDR

	Uart550_Setup();

#endif

	/* Enable TX and RX for the device */
	u32 cr = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_CR_OFFSET);
	XUartPs_WriteReg(UART_BASEADDR, XUARTPS_CR_OFFSET,
		(cr & ~XUARTPS_CR_EN_DIS_MASK) | XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN);

	uart_puts(UART_BASEADDR, "BOOT\n\r");
		
	/* Initialize the Device Configuration Interface driver */
#ifndef SDT
	Config = XLlFfio_LookupConfig(DeviceId);
#else
	Config = XLlFfio_LookupConfig(BaseAddress);
#endif
	if (!Config) {
#ifndef SDT
		xil_printf("No config found for %d\r\n", DeviceId);
#endif
		return XST_FAILURE;
	}

	/*
	 * This is where the virtual address would be used, this example
	 * uses physical address.
	 */
	Status = XLlFifo_CfgInitialize(InstancePtr, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed\n\r");
		return Status;
	}

	/* Check for the Reset value */
	Status = XLlFifo_Status(InstancePtr);
	XLlFifo_IntClear(InstancePtr,0xffffffff);
	Status = XLlFifo_Status(InstancePtr);
	if(Status != 0x0) {
		xil_printf("\n ERROR : Reset value of ISR0 : 0x%x\t"
			    "Expected : 0x0\n\r",
			    XLlFifo_Status(InstancePtr));
		return XST_FAILURE;
	}

    /*1.Wait in the while loop for matrix data*/
	/*2.Check if matrix data is valid*/
    /*3.Gather Matrix data of A and B in the source buffer*/
	u32 Running;
	u8 A_Matrix[A_ROWS*A_COLS] = {0};
	u8 B_Matrix[A_COLS] = {0};
	u8 result[A_ROWS] = {0};

	int word_cnt, test_case_cnt = 0;
	int success;
	Status = XST_SUCCESS;

	int* test_input_memory = (int*)TX_BUFFER_BASE;
	int* result_memory = (int*)RX_BUFFER_BASE;


	/************************** Initializations *****************************/

	 XAxiDma_Config *CfgPtr;

	/* Initialize the XAxiDma device.*/
	CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
	if (!CfgPtr) {
	  xil_printf("No config found for %d\r\n", DMA_DEV_ID);
	  return XST_FAILURE;
	}

	Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
	if (Status != XST_SUCCESS) {
	  xil_printf("Initialization failed %d\r\n", Status);
	  return XST_FAILURE;
	}

	if(XAxiDma_HasSg(&AxiDma)){
	  xil_printf("Device configured as SG mode \r\n");
	  return XST_FAILURE;
	}

	/* Disable interrupts, we use polling mode */
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
	// Xil_DCacheDisable(); // uncomment this as a last resort, which will avoid all cache related issues, but at the expense of performance.

	Running = 1;
    
	while(Running)
	{
		//Wait for first data to come in
		//while(!XUartPs_IsReceiveData(UART_BASEADDR));
	
		//Receive the data from fifo buffer
		//RecvChar = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
		
		for (int i = 0; i < A_ROWS * A_COLS; i++) {
			uart_read_u8_csv(UART_BASEADDR, &A_Matrix[i], &SourceBuffer[i]);
		}

	    for (int i = 0; i < A_COLS; i++) {
		    uart_read_u8_csv(UART_BASEADDR, &B_Matrix[i], &SourceBuffer[i + TOTAL_MATRIX_A_ELEMENTS]);
        }
		for (word_cnt=0 ; word_cnt < NUMBER_OF_INPUT_WORDS ; word_cnt++){
			test_input_memory[word_cnt] = SourceBuffer[word_cnt];
			// test_input_memory[word_cnt] = 100;
		}
		

		xil_printf(" Transmitting Data for test case\r\n");

		/* Flush the SrcBuffer and DestBuffer before the DMA transfer, in case the Data Cache is enabled */
		Xil_DCacheFlushRange((u32)(result_memory), 4*NUMBER_OF_INPUT_WORDS);
		Xil_DCacheFlushRange((u32)(test_input_memory), 4*NUMBER_OF_INPUT_WORDS);
		
		u32 t0 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);

		Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) (result_memory), 4*NUMBER_OF_OUTPUT_WORDS, XAXIDMA_DEVICE_TO_DMA);//rx

		if (Status != XST_SUCCESS) {
		  return XST_FAILURE;
		}
		
		Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) (test_input_memory), 4*NUMBER_OF_INPUT_WORDS, XAXIDMA_DMA_TO_DEVICE);//tx

		if (Status != XST_SUCCESS) {
		  return XST_FAILURE;
		}
		while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {
			//wait for transfer to complete
		}

		/* Transmission Complete */

		/************************** Receive the Data Stream *****************************/

		//xil_printf(" Receiving data for test case %d ... \r\n", test_case_cnt);

		while (XAxiDma_Busy(&AxiDma,XAXIDMA_DEVICE_TO_DMA)) {
			//wait for transfer to complete
		}
		u32 t1 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);
		u32 cycles_dma = t1 - t0;   // unsigned wraparound-safe
		
		uart_puts(UART_BASEADDR, "CYC_DMA=");
		uart_put_u32_dec(UART_BASEADDR, cycles_dma);
		uart_puts(UART_BASEADDR, "\n");
		uart_puts(UART_BASEADDR, "\r");
		uart_puts(UART_BASEADDR, "\n");
		uart_puts(UART_BASEADDR, "\r");

		/* Invalidate the DestBuffer before receiving the data, in case the Data Cache is enabled */
		Xil_DCacheInvalidateRange((u32)(result_memory), 4*NUMBER_OF_OUTPUT_WORDS);

	
		
		u32 t3 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);

		/* Transmit the Data Stream */
		Status = TxSend(InstancePtr, SourceBuffer);
		if (Status != XST_SUCCESS){
			xil_printf("Transmission of Data failed\n\r");
			return XST_FAILURE;
		}

		/* Receive the Data Stream */
		Status = RxReceive(InstancePtr, DestinationBuffer);
		if (Status != XST_SUCCESS){
			xil_printf("Receiving data failed");
			return XST_FAILURE;
		}
		u32 t4 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);
		u32 cycles_fifo = t4 - t3;   // unsigned wraparound-safe
		
		uart_puts(UART_BASEADDR, "CYC_FIFO=");
		uart_put_u32_dec(UART_BASEADDR, cycles_fifo);
		uart_puts(UART_BASEADDR, "\n");
		uart_puts(UART_BASEADDR, "\r");
		uart_puts(UART_BASEADDR, "\n");
		uart_puts(UART_BASEADDR, "\r");
		
		Error = 0;

			
		/*if Compare is successfull meaning the data is the same, print them out on the serial terminal*/
		if (!Error)
		{
			/*Matrix Calculation*/
			u32 t0 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);
			for (int r = 0; r < A_ROWS; r++)
			{
				int sum = 0;
				// test1 = A_Matrix[0];
				for (int c = 0; c < A_COLS; c++)
				{
					test1 = A_Matrix[r * A_COLS + c];
					test2 = B_Matrix[c];
					sum += A_Matrix[r * A_COLS + c] * B_Matrix[c];
				}
				
				result[r] = sum / 256;
			
			}
			u32 t1 = XTmrCtr_GetValue(TmrCtrInstancePtr, TIMER_COUNTER_0);
			u32 cycles_matrix_multiply = t1 - t0;
			uart_puts(UART_BASEADDR, "CYC_MATRIX_MULTIPLY_Software=");
			uart_put_u32_dec(UART_BASEADDR, cycles_matrix_multiply);
			uart_puts(UART_BASEADDR, "\n");
			uart_puts(UART_BASEADDR, "\r");
			uart_puts(UART_BASEADDR, "\n");
			uart_puts(UART_BASEADDR, "\r");

			uart_puts(UART_BASEADDR, "CO_PROCESSOR_RES\n\r");
			for (int r = 0; r < A_ROWS; r++) 
			{
				uart_put_u32_dec(UART_BASEADDR, *(result_memory + r));
				uart_puts(UART_BASEADDR, "\n");
				uart_puts(UART_BASEADDR, "\r");
				uart_puts(UART_BASEADDR, "\n");
				uart_puts(UART_BASEADDR, "\r");
			}

			uart_puts(UART_BASEADDR, "RES_Softwre\n\r");
			for (int r = 0; r < A_ROWS; r++) 
			{
				uart_put_u32_dec(UART_BASEADDR, (u32)result[r]);
				uart_puts(UART_BASEADDR, "\n");
				uart_puts(UART_BASEADDR, "\r");
				uart_puts(UART_BASEADDR, "\n");
				uart_puts(UART_BASEADDR, "\r");
			}
			
		}
			

			if (Error != 0){
				return XST_FAILURE;
			}

			
    }

				
	

	return Status;
}

/*****************************************************************************/
/**
*
* TxSend routine, It will send the requested amount of data at the
* specified addr.
*
* @param	InstancePtr is a pointer to the instance of the
*		XLlFifo component.
*
* @param	SourceAddr is the address where the FIFO stars writing
*
* @return
*		-XST_SUCCESS to indicate success
*		-XST_FAILURE to indicate failure
*
* @note		None
*
******************************************************************************/
int TxSend(XLlFifo *InstancePtr, u32  *SourceAddr)
{

	int i;
	int j;

	// /* Fill the transmit buffer with incremental pattern */
	// for (i=0;i<MAX_DATA_BUFFER_SIZE;i++)
	// 	*(SourceAddr + i) = i;

	for(i=0 ; i < NO_OF_PACKETS ; i++){

		/* Writing into the FIFO Transmit Port Buffer */
		for (j=0 ; j < MAX_PACKET_LEN ; j++){
			if( XLlFifo_iTxVacancy(InstancePtr) ){
				XLlFifo_TxPutWord(InstancePtr,
					*(SourceAddr+(i*MAX_PACKET_LEN)+j));
			}
		}

	}

	/* Start Transmission by writing transmission length into the TLR */
	XLlFifo_iTxSetLen(InstancePtr, (MAX_DATA_BUFFER_SIZE * WORD_SIZE));

	/* Check for Transmission completion */
	while( !(XLlFifo_IsTxDone(InstancePtr)) ){

	}

	/* Transmission Complete */
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* RxReceive routine.It will receive the data from the FIFO.
*
* @param	InstancePtr is a pointer to the instance of the
*		XLlFifo instance.
*
* @param	DestinationAddr is the address where to copy the received data.
*
* @return
*		-XST_SUCCESS to indicate success
*		-XST_FAILURE to indicate failure
*
* @note		None
*
******************************************************************************/
int RxReceive (XLlFifo *InstancePtr, u32* DestinationAddr)
{
    u32 total = 0;

    // Wait until we receive A_ROWS words
    while (total < A_ROWS) {

        // Wait until some data is available
        while (!XLlFifo_iRxOccupancy(InstancePtr)) { }

        // Drain all available frames
        while (XLlFifo_iRxOccupancy(InstancePtr)) {
            u32 len = XLlFifo_iRxGetLen(InstancePtr) / WORD_SIZE;

            for (u32 i = 0; i < len; i++) {
                u32 w = XLlFifo_RxGetWord(InstancePtr);
                if (total < A_ROWS) {
                    DestinationAddr[total++] = w;
                }
            }
        }
    }

    return XST_SUCCESS;
}
#ifdef XPAR_UARTNS550_0_BASEADDR
/*****************************************************************************/
/*
*
* Uart16550 setup routine, need to set baudrate to 9600 and data bits to 8
*
* @param	None
*
* @return	None
*
* @note		None
*
******************************************************************************/
static void Uart550_Setup(void)
{

	XUartNs550_SetBaud(XPAR_UARTNS550_0_BASEADDR,
			XPAR_XUARTNS550_CLOCK_HZ, 9600);

	XUartNs550_SetLineControlReg(XPAR_UARTNS550_0_BASEADDR,
			XUN_LCR_8_DATA_BITS);
}
#endif



uint8_t uart_getc(u32 base)
{
    while (!XUartPs_IsReceiveData(base)) { }
    return (u8)(XUartPs_ReadReg(base, XUARTPS_FIFO_OFFSET) & 0xFF);
}

int uart_read_u8_csv(u32 base, u8 *out, u32 *buffer)
{
    int num = 0;
    int got_digit = 0;
    u8 c;

    while (1) {
        c = uart_getc(base);

        if (c >= '0' && c <= '9') {
            num = num * 10 + (c - '0');
            got_digit = 1;
        } else if (c == ',' || c == '\n' || c == '\r' || c == ' ' || c == '\t') {
            if (got_digit) {
                if (num < 0) num = 0;
                if (num > 255) num = 255;
                *out = (u8)num;
				*buffer = (u8)num;
                return 1;
            }
            // else: ignore repeated delimiters
        } else {
            // ignore any other characters
        }
    }
}

void uart_putc(u32 base, u8 c) {
    while (XUartPs_IsTransmitFull(base)) {}
    XUartPs_WriteReg(base, XUARTPS_FIFO_OFFSET, c);
}

void uart_puts(u32 base, const char *s) {
    while (*s != '\0' ) uart_putc(base, (u8)*s++);
}

void uart_put_u32_dec(u32 base, u32 v)
{
    // Buffer to hold digits. u32 max is 10 digits (4294967295).
    char digits[10];
    int count = 0;

    // Special case: v = 0 should print "0"
    if (v == 0) {
        uart_putc(base, '0');
        return;
    }

    // Step 1: extract digits from right to left.
    // Each loop gets the least significant digit.
    while (v > 0) {
        u32 digit = v % 10;          // remainder gives last digit
        digits[count] = '0' + digit; // convert digit 0..9 to ASCII '0'..'9'
        count++;

        v = v / 10;                  // drop the last digit
    }

    // At this point digits[] holds the number reversed.
    // Example: v=507 => digits = {'7','0','5'} and count=3.

    // Step 2: output digits in reverse to correct the order.
    for (int i = count - 1; i >= 0; i--) {
        uart_putc(base, digits[i]);
    }
}






//Timer Function
#ifndef SDT
int TmrCtrPolledExample(u16 DeviceId, u8 TmrCtrNumber)
#else
int TmrCtrPolledExample(UINTPTR BaseAddr, u8 TmrCtrNumber)
#endif
{
	//int Status;
	//u32 Value1;
	//u32 Value2;
	//XTmrCtr *TmrCtrInstancePtr = &TimerCounter;

	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
// #ifndef SDT
// 	xil_printf("Timer initialising1..,\n\r");
// 	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
// #else
//     xil_printf("Timer initialising1.1\n\r");
// 	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, BaseAddr);
// #endif
// 	if (Status != XST_SUCCESS) {
// 		xil_printf("Timer initialisation failed\n\r");
// 		return XST_FAILURE;
// 	}

	// /*
	//  * Perform a self-test to ensure that the hardware was built
	//  * correctly, use the 1st timer in the device (0)
	//  */
	// Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TmrCtrNumber);
	// if (Status != XST_SUCCESS) {
	// 	return XST_FAILURE;
	// }

    //xil_printf("Timer initialisiation successful\n\r");
	/*
	 * Enable the Autoreload mode of the timer counters.
	 */
	//XTmrCtr_SetOptions(TmrCtrInstancePtr, TmrCtrNumber,
			   //XTC_AUTO_RELOAD_OPTION);

	// /*
	//  * Get a snapshot of the timer counter value before it's started
	//  * to compare against later
	//  */
	// Value1 = XTmrCtr_GetValue(TmrCtrInstancePtr, TmrCtrNumber);

	/*
	 * Start the timer counter such that it's incrementing by default
	 */
	//XTmrCtr_Start(TmrCtrInstancePtr, TmrCtrNumber);

	// /*
	//  * Read the value of the timer counter and wait for it to change,
	//  * since it's incrementing it should change, if the hardware is not
	//  * working for some reason, this loop could be infinite such that the
	//  * function does not return
	//  */
	// while (1) {
	// 	Value2 = XTmrCtr_GetValue(TmrCtrInstancePtr, TmrCtrNumber);
	// 	if (Value1 != Value2) {
	// 		break;
	// 	}
	// }

	// /*
	//  * Disable the Autoreload mode of the timer counters.
	//  */
	// XTmrCtr_SetOptions(TmrCtrInstancePtr, TmrCtrNumber, 0);

	return XST_SUCCESS;
}
