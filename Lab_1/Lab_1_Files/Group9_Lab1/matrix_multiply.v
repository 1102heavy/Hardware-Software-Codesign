`timescale 1ns / 1ps

/* 
----------------------------------------------------------------------------------
--	(c) Rajesh C Panicker, NUS
--  Description : Template for the Matrix Multiply unit for the AXI Stream Coprocessor
--	License terms :
--	You are free to use this code as long as you
--		(i) DO NOT post a modified version of this on any public repository;
--		(ii) use it only for educational purposes;
--		(iii) accept the responsibility to ensure that your implementation does not violate any intellectual property of any entity.
--		(iv) accept that the program is provided "as is" without warranty of any kind or assurance regarding its suitability for any particular purpose;
--		(v) send an email to rajesh.panicker@ieee.org briefly mentioning its use (except when used for the course EE4218 at the National University of Singapore);
--		(vi) retain this notice in this file or any files derived from this.
----------------------------------------------------------------------------------
*/

// those outputs which are assigned in an always block of matrix_multiply shoud be changes to reg (such as output reg Done).

module matrix_multiply
	#(	parameter width = 8, 			// width is the number of bits per location
		parameter A_depth_bits = 3, 	// depth is the number of locations (2^number of address bits)
		parameter B_depth_bits = 2, 
		parameter RES_depth_bits = 1
	) 
	(
		input clk,										
		input Start,									// myip_v1_0 -> matrix_multiply_0.
		output reg Done = 0,									// matrix_multiply_0 -> myip_v1_0. Possibly reg.
		
		output reg A_read_en = 0,  								// matrix_multiply_0 -> A_RAM. Possibly reg.
		output reg [A_depth_bits-1:0] A_read_address = 0, 		// matrix_multiply_0 -> A_RAM. Possibly reg.
		input [width-1:0] A_read_data_out,				// A_RAM -> matrix_multiply_0.
		
		output reg B_read_en = 0, 								// matrix_multiply_0 -> B_RAM. Possibly reg.
		output reg [B_depth_bits-1:0] B_read_address = 0, 		// matrix_multiply_0 -> B_RAM. Possibly reg.
		input [width-1:0] B_read_data_out,				// B_RAM -> matrix_multiply_0.
		
		output reg RES_write_en = 0, 							// matrix_multiply_0 -> RES_RAM. Possibly reg.
		output reg [RES_depth_bits-1:0] RES_write_address = 0, 	// matrix_multiply_0 -> RES_RAM. Possibly reg.
		output reg [width-1:0] RES_write_data_in = 0			// matrix_multiply_0 -> RES_RAM. Possibly reg.
	);
	
	// implement the logic to read A_RAM, read B_RAM, do the multiplication and write the results to RES_RAM
	// Note: A_RAM and B_RAM are to be read synchronously. Read the wiki for more details.

    //Define the number of elements and deduce the rows and columns
    localparam integer A_ELEMS   = (1 << A_depth_bits);
    localparam integer B_ELEMS   = (1 << B_depth_bits);
    localparam integer RES_ELEMS = (1 << RES_depth_bits);
    
    localparam integer N = 1;              // fixed: B is Kx1
    localparam integer K = B_ELEMS / N;    // = B_ELEMS
    localparam integer R = RES_ELEMS / N;  // = RES_ELEMS
    
    localparam integer A_ROWS = R;
    localparam integer A_COLS = K;         // “rowsize/stride” for row-major  
    
    //Define counters and their sizes
    localparam ROWSIZE = (1 << B_depth_bits); //Logic: Column number for B matrix is 1. This is the ROWSIZE OF B
    reg [$clog2(A_COLS):0] k = 0; //Row of B = Column of A
    reg [$clog2(A_ROWS):0] r = 0;
    reg [15:0 ]accumulator=0;
    reg [15:0 ]product=0;
    
	// Define the states of state machine (one hot encoding)
	localparam Idle  = 5'b10000;
	localparam Read_Inputs = 5'b01000;
	localparam Compute = 5'b00100;
//	localparam Sum = 6'b000100;
	localparam Write_Outputs  = 5'b00010;
	localparam DONE = 5'b00001;
	
    reg [4:0] next_state = 5'b10000;
    reg [4:0] current_state = 5'b10000;
    reg acc_reset = 0;
    reg count_en = 0;
    
    
    always @(posedge clk) begin
  
        current_state<= next_state;
         
        
        
        if(acc_reset == 1)begin
            accumulator <= 0;
            k<=0; 
            if(r==A_ROWS-1) begin
                r<=0;
            end
            else begin    
                r<=r+1;
            end
        end
        else if(count_en==1) begin
            accumulator<=accumulator+product;
            k<=k+1;

        end
        
       
    end
    
    always @(*) begin    // output   
        
        A_read_address = 0;
        B_read_address = 0;
        A_read_en = 0;
        B_read_en = 0;
        count_en =0;
        
        product = 0;
        
        RES_write_address = 0;
        RES_write_en = 0;
        RES_write_data_in = 0;
        acc_reset =0;
        
        Done = 0; 
        next_state = Idle;
        case (current_state)        
            Idle: begin  

                
                A_read_address = 0;
                B_read_address = 0;
                A_read_en = 0;
                B_read_en = 0;
                count_en =0;
                
                product = 0;
                
                RES_write_address = 0;
                RES_write_en = 0;
                RES_write_data_in = 0;
                acc_reset =0;
                
                Done = 0;
                
                if (!Start) begin
                    next_state = Idle;
                end
                else begin
                    next_state = Read_Inputs;
                end     
            end
            
            Read_Inputs: begin
               
            
//               Reset RES parameters

                
                A_read_address = A_COLS * r + k;
                B_read_address = k;
                A_read_en = 1;
                B_read_en = 1;
                count_en =0;
                
                product = 0;
                
                RES_write_address = 0;
                RES_write_en = 0;
                RES_write_data_in = 0;
                acc_reset =0;
                
                Done = 0;
                if(k==A_COLS) begin
                    next_state= Write_Outputs;
                end
                else begin
                
                    next_state = Compute;
                end 

          
//                next_state = Compute;
            end
            
            Compute: begin
                //Address is being sent
                
                A_read_address = A_COLS * r + k;
                B_read_address = k;
                A_read_en = 1;
                B_read_en = 1;
                count_en =1;
                
                product = (A_read_data_out * B_read_data_out);
                
                RES_write_address = 0;
                RES_write_en = 0;
                RES_write_data_in = 0;
                acc_reset =0;
                
                Done = 0;
                
                next_state = Read_Inputs;
                
//                next_state = Sum;

            end
            
//            Sum: begin
               
                
//                A_read_address = A_COLS * r + k;
//                B_read_address = k;
//                A_read_en = 1;
//                B_read_en = 1;
//                count_en =0;
                
//                product = 0;
                
//                RES_write_address = 0;
//                RES_write_en = 0;
//                RES_write_data_in = 0;
//                acc_reset =0;
                
//                Done = 0;

//                //Check if one row of calculation is done
//                if(k==A_COLS) begin
//                    next_state= Write_Outputs;
//                end
//                else begin
                
//                    next_state = Read_Inputs;
//                end     
//            end
            
            Write_Outputs: begin
                
                A_read_address = A_COLS * r + k;
                B_read_address = k;
                A_read_en = 1;
                B_read_en = 1;
                count_en =0;
                
                product = 0;
                RES_write_address = r;
                RES_write_en = 1;
                RES_write_data_in = accumulator[15:8];
                acc_reset =1;
                
                Done = 0;
                
                
                if(r==A_ROWS-1) begin
                   next_state = DONE;
                end 
                else begin
                    next_state = Read_Inputs;
                end
            
       
            end
            
            DONE: begin
                
                A_read_address = 0;
                B_read_address = 0;
                A_read_en = 0;
                B_read_en = 0;
                count_en =0;
                
                product = 0;
                
                RES_write_address = 0;
                RES_write_en = 0;
                RES_write_data_in = 0;
                acc_reset =0;
                
                Done = 1;
                next_state = Idle;
            end
            
            default: next_state = Idle;
            
            
        endcase

    
    end




endmodule


