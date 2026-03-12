/*
----------------------------------------------------------------------------------
--	(c) Rajesh C Panicker, NUS,
--  Description : AXI Stream Coprocessor (HLS), implementing the sum of 4 numbers
--	License terms :
--	You are free to use this code as long as you
--		(i) DO NOT post a modified version of this on any public repository;
--		(ii) use it only for educational purposes;
--		(iii) accept the responsibility to ensure that your implementation does not violate any intellectual property of any entity.
--		(iv) accept that the program is provided "as is" without warranty of any kind or assurance regarding its suitability for any particular purpose;
--		(v) send an email to rajesh.panicker@ieee.org briefly mentioning its use (except when used for the course EE4218/CEG5203 at the National University of Singapore);
--		(vi) retain this notice in this file or any files derived from this.
----------------------------------------------------------------------------------
*/

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"


#define A_COLS 8
#define A_ROWS 64
#define B_COLS 1
#define B_ROWS 8
#define TOTAL_MATRIX_A_ELEMENTS ((A_COLS * A_ROWS))
#define TOTAL_MATRIX_B_ELEMENTS ((B_COLS * B_ROWS))
#define TOTAL_MATRIX_ELEMENTS  ((TOTAL_MATRIX_B_ELEMENTS + TOTAL_MATRIX_A_ELEMENTS))

#define NUMBER_OF_INPUT_WORDS TOTAL_MATRIX_ELEMENTS  // length of an input vector
#define NUMBER_OF_OUTPUT_WORDS A_ROWS   // length of an input vector
// ACLK, ARESETN, TREADY, TDATA, TVALID are essential signals for AXIS. New version of AXI DMA seems to expect TSTRB and/or TKEEP as well.

typedef ap_axis<32,0,0,0> AXIS;  //data, user, id, dest
void myip_v1_0_HLS_optimised(hls::stream<AXIS>& S_AXIS, hls::stream<AXIS>& M_AXIS){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis port=S_AXIS
#pragma HLS INTERFACE axis port=M_AXIS
#pragma HLS pipeline off


	ap_uint<8> A_Matrix[A_ROWS*A_COLS] = {0};
	ap_uint<8> B_Matrix[A_COLS] = {0};
	ap_uint<8> result[A_ROWS] = {0};

	#pragma HLS array_partition variable=A_Matrix cyclic factor=8
	#pragma HLS array_partition variable=B_Matrix cyclic factor=8

	int word_cnt;
	ap_uint<8> sum = 0; // using arbitrary precision
	//int sum = 0;		 // using 32 bit precision
	AXIS read_input, write_output;

		myip_v1_0_HLS_optimised_for1:for(word_cnt = 0; word_cnt < NUMBER_OF_INPUT_WORDS; word_cnt++){
        #pragma HLS UNROLL
			read_input = S_AXIS.read();
			// read_input is the element (data + other signals) received by our ip through S_AXIS in one clock cycle (which contains one word).
			// read() extracts it from the stream. Overloaded operator >> can also be used.
			// sum += read_input.data; //extracting that word
			// We are not making using of S_AXIS_TLAST in this example.
			// S_AXIS_TLAST is required only when we are receiving an unknown number of words.
			if(word_cnt< TOTAL_MATRIX_A_ELEMENTS){
				A_Matrix[word_cnt]=read_input.data;

			}else if (word_cnt< TOTAL_MATRIX_ELEMENTS){
				B_Matrix[word_cnt-TOTAL_MATRIX_A_ELEMENTS]=read_input.data;
			}
		}

					for (int r = 0; r < A_ROWS; r++)
			{
				int sum = 0;
				// test1 = A_Matrix[0];
				for (int c = 0; c < A_COLS; c++)
				{
					sum += A_Matrix[r * A_COLS + c] * B_Matrix[c];
				}
				
				result[r] = sum / 256;
			
			}

		myip_v1_0_HLS_optimised_for2:for(word_cnt = 0; word_cnt < NUMBER_OF_OUTPUT_WORDS; word_cnt++){
			//write_output.data = sum.to_int() + word_cnt;	// using arbitrary precision internally but int for interfacing
			write_output.data = result[word_cnt];	// using 32 bit precision or arbitrary precision all the way
			// write_output is the element sent by our ip through M_AXIS in one clock cycle.
			write_output.last = 0;
			write_output.keep = 0xFU;
			write_output.strb = 0xFU;
			if(word_cnt==NUMBER_OF_OUTPUT_WORDS-1)
			{
				write_output.last = 1;
				// M_AXIS_TLAST is required to be asserted for the last word.
				// Else, the AXI Stream FIFO / AXI DMA will not know if all the words have been received from the co-processor.
			}
			M_AXIS.write(write_output);
			// write() inserts it into the stream. Overloaded operator << can also be used.
		}
}
