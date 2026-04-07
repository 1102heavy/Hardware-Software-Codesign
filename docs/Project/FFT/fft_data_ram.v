// ─────────────────────────────────────────────────────────────────────────
// fft_data_ram.v
//
// True dual-port block RAM for the FFT working set.
//
// Why BRAM is useful here:
//   - moves the 1024×32 working memory out of LUTRAM/distributed RAM
//   - provides two independent write ports so both butterfly outputs can be
//     committed in the same clock cycle
//   - usually improves LUT usage and often helps timing closure
//
// Read behaviour:
//   - synchronous read on both ports
//   - the controller is retimed so addresses are presented before the edge
//     that should capture the requested word
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_data_ram #(
    parameter DWIDTH = 32,
    parameter DEPTH  = 1024,
    parameter ABITS  = 10
)(
    input  wire              clk,

    // ── Port A - read/write ────────────────────────────────────────────
    input  wire [ABITS-1:0]  addr_a,
    input  wire [DWIDTH-1:0] din_a,
    input  wire              we_a,
    output reg  [DWIDTH-1:0] dout_a,

    // ── Port B - read/write ────────────────────────────────────────────
    input  wire [ABITS-1:0]  addr_b,
    input  wire [DWIDTH-1:0] din_b,
    input  wire              we_b,
    output reg  [DWIDTH-1:0] dout_b
);

    integer init_i;
    (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

    initial begin
        dout_a = {DWIDTH{1'b0}};
        dout_b = {DWIDTH{1'b0}};
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1)
            mem[init_i] = {DWIDTH{1'b0}};
    end

    // Port A: synchronous write, synchronous read
    always @(posedge clk) begin
        if (we_a)
            mem[addr_a] <= din_a;
        dout_a <= mem[addr_a];
    end

    // Port B: synchronous write, synchronous read
    always @(posedge clk) begin
        if (we_b)
            mem[addr_b] <= din_b;
        dout_b <= mem[addr_b];
    end

endmodule
