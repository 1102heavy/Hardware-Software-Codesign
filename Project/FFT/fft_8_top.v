// ─────────────────────────────────────────────────────────────────────────
// fft_8_top.v
//
// Top-level wrapper — parameterized radix-2 DIT FFT.
// Currently configured for N=1024, LOG2N=10.
// Uses fft_twiddle_rom (512-entry ROM).
//
// All computation submodules (fft_controller, fft_butterfly, fft_data_ram)
// are fully parameterized.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_8_top #(
    parameter N      = 1024,
    parameter LOG2N  = 10,
    parameter DWIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,

    // ── AXI4-Stream Slave — input samples ────────────────────────────
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire [2*DWIDTH-1:0]   s_axis_tdata,
    input  wire                  s_axis_tlast,

    // ── AXI4-Stream Master — output spectrum ─────────────────────────
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [2*DWIDTH-1:0]   m_axis_tdata,
    output wire                  m_axis_tlast,

    // ── Status ───────────────────────────────────────────────────────
    output wire                  done
);

    // ── Internal wires ────────────────────────────────────────────────
    wire [LOG2N-1:0]    ram_addr_a, ram_addr_b;
    wire [2*DWIDTH-1:0] ram_din_a,  ram_din_b;
    wire                ram_we_a,   ram_we_b;
    wire [2*DWIDTH-1:0] ram_dout_a, ram_dout_b;

    wire [LOG2N-2:0]         rom_addr;
    wire signed [DWIDTH-1:0] rom_w_real, rom_w_imag;

    wire signed [DWIDTH-1:0] bfly_a_r, bfly_a_i;
    wire signed [DWIDTH-1:0] bfly_b_r, bfly_b_i;
    wire signed [DWIDTH-1:0] bfly_w_r, bfly_w_i;
    wire signed [DWIDTH:0]   bfly_x_r, bfly_x_i;
    wire signed [DWIDTH:0]   bfly_y_r, bfly_y_i;

    // ── Submodules ────────────────────────────────────────────────────

    fft_controller #(
        .N(N), .LOG2N(LOG2N), .DWIDTH(DWIDTH)
    ) u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tlast   (s_axis_tlast),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tlast   (m_axis_tlast),
        .ram_addr_a     (ram_addr_a),  .ram_addr_b (ram_addr_b),
        .ram_din_a      (ram_din_a),   .ram_din_b  (ram_din_b),
        .ram_we_a       (ram_we_a),    .ram_we_b   (ram_we_b),
        .ram_dout_a     (ram_dout_a),  .ram_dout_b (ram_dout_b),
        .rom_addr       (rom_addr),
        .rom_w_real     (rom_w_real),  .rom_w_imag (rom_w_imag),
        .bfly_a_r       (bfly_a_r),    .bfly_a_i   (bfly_a_i),
        .bfly_b_r       (bfly_b_r),    .bfly_b_i   (bfly_b_i),
        .bfly_w_r       (bfly_w_r),    .bfly_w_i   (bfly_w_i),
        .bfly_x_r       (bfly_x_r),    .bfly_x_i   (bfly_x_i),
        .bfly_y_r       (bfly_y_r),    .bfly_y_i   (bfly_y_i),
        .done           (done)
    );

    fft_butterfly #(
        .DWIDTH(DWIDTH)
    ) u_bfly (
        .a_r(bfly_a_r), .a_i(bfly_a_i),
        .b_r(bfly_b_r), .b_i(bfly_b_i),
        .w_r(bfly_w_r), .w_i(bfly_w_i),
        .x_r(bfly_x_r), .x_i(bfly_x_i),
        .y_r(bfly_y_r), .y_i(bfly_y_i)
    );

    fft_data_ram #(
        .DWIDTH(2*DWIDTH), .DEPTH(N), .ABITS(LOG2N)
    ) u_ram (
        .clk    (clk),
        .addr_a (ram_addr_a), .din_a (ram_din_a), .we_a (ram_we_a), .dout_a (ram_dout_a),
        .addr_b (ram_addr_b), .din_b (ram_din_b), .we_b (ram_we_b), .dout_b (ram_dout_b)
    );

    // 512-entry twiddle ROM for N=1024
    fft_twiddle_rom #(
        .DEPTH(N/2), .ABITS(LOG2N-1)
    ) u_rom (
        .clk    (clk),
        .addr   (rom_addr),
        .w_real (rom_w_real),
        .w_imag (rom_w_imag)
    );

endmodule
