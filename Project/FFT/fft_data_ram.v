// ─────────────────────────────────────────────────────────────────────────
// fft_data_ram.v
//
// True dual-port RAM — synchronous write, asynchronous (combinational) read.
// Width: 32 bits (16b real + 16b imag per word)
// Depth: 1024 entries (addresses 0 to 1023)
//
// Asynchronous read is required by the FFT controller pipeline:
//   S_READ  sets ram_addr (registered → effective next cycle)
//   S_LATCH reads ram_dout — which must reflect mem[addr_A] that was
//   presented in S_READ.  With sync read there is a one-cycle gap
//   (dout would still show the address from BEFORE S_READ).
//   Async read closes this gap: once addr settles after S_READ's posedge,
//   dout combinationally shows mem[addr_A] in time for S_LATCH.
//
// Write latency: data is in RAM on the rising edge when we=1.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_data_ram #(
    parameter DWIDTH = 32,    // 16b real + 16b imag
    parameter DEPTH  = 1024,
    parameter ABITS  = 10
)(
    input  wire              clk,

    // ── Port A ─────────────────────────────────────────────────────────
    input  wire [ABITS-1:0]  addr_a,
    input  wire [DWIDTH-1:0] din_a,
    input  wire              we_a,
    output wire [DWIDTH-1:0] dout_a,

    // ── Port B ─────────────────────────────────────────────────────────
    input  wire [ABITS-1:0]  addr_b,
    input  wire [DWIDTH-1:0] din_b,
    input  wire              we_b,
    output wire [DWIDTH-1:0] dout_b
);

    integer init_i;
    reg [DWIDTH-1:0] mem [0:DEPTH-1];

    // Initialise every entry to 0 so simulation never reads X.
    initial begin
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1)
            mem[init_i] = {DWIDTH{1'b0}};
    end

    // Port A: synchronous write, combinational read
    always @(posedge clk) begin
        if (we_a) mem[addr_a] <= din_a;
    end
    assign dout_a = mem[addr_a];

    // Port B: synchronous write, combinational read
    always @(posedge clk) begin
        if (we_b) mem[addr_b] <= din_b;
    end
    assign dout_b = mem[addr_b];

endmodule