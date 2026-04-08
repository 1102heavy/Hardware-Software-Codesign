// ─────────────────────────────────────────────────────────────────────────
// fft_twiddle_rom_n8.v
//
// 4-entry twiddle ROM for an 8-point FFT (N/2 = 4 entries).
//
// W_8^k = cos(2*pi*k/8) - j*sin(2*pi*k/8),  k = 0..3, Q1.15
//
//   k=0: cos(0)       - j*sin(0)        =  1.0000 - j*0.0000
//        w_real = 0x7FFF (+32767), w_imag = 0x0000 (0)
//
//   k=1: cos(pi/4)    - j*sin(pi/4)     = +0.7071 - j*0.7071
//        w_real = 0x5A82 (+23170), w_imag = 0xA57E (-23170)
//
//   k=2: cos(pi/2)    - j*sin(pi/2)     =  0.0000 - j*1.0000
//        w_real = 0x0000 (0),      w_imag = 0x8000 (-32768)
//
//   k=3: cos(3*pi/4)  - j*sin(3*pi/4)  = -0.7071 - j*0.7071
//        w_real = 0xA57E (-23170), w_imag = 0xA57E (-23170)
//
// Output is combinational (async) — same pipeline reason as fft_data_ram:
// rom_addr is registered by the FSM in S_READ; the twiddle value must be
// available combinationally for S_LATCH to capture one cycle later.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_twiddle_rom_n8 #(
    parameter DEPTH = 4,     // N/2
    parameter ABITS = 2      // log2(DEPTH)
)(
    input  wire              clk,   // kept for port compatibility; not used
    input  wire [ABITS-1:0]  addr,
    output wire signed [15:0] w_real,
    output wire signed [15:0] w_imag
);

    (* rom_style = "distributed" *) reg [31:0] rom [0:DEPTH-1];

    initial begin
        rom[0] = {16'h7FFF, 16'h0000};  // W_8^0 =  1.0000 + j*0.0000
        rom[1] = {16'h5A82, 16'hA57E};  // W_8^1 = +0.7071 - j*0.7071
        rom[2] = {16'h0000, 16'h8000};  // W_8^2 =  0.0000 - j*1.0000
        rom[3] = {16'hA57E, 16'hA57E};  // W_8^3 = -0.7071 - j*0.7071
    end

    assign w_real = $signed(rom[addr][31:16]);
    assign w_imag = $signed(rom[addr][15:0]);

endmodule
