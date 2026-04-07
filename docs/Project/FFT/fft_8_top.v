// ─────────────────────────────────────────────────────────────────────────
// fft_8_top.v
//
// Compatibility wrapper that preserves the old module name while delegating
// to fft_top_8.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_8_top #(
    parameter N      = 8,
    parameter LOG2N  = 3,
    parameter DWIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,

    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire [2*DWIDTH-1:0]   s_axis_tdata,
    input  wire                  s_axis_tlast,

    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [2*DWIDTH-1:0]   m_axis_tdata,
    output wire                  m_axis_tlast,

    output wire                  done
);

    fft_top_8 #(
        .N(N), .LOG2N(LOG2N), .DWIDTH(DWIDTH)
    ) u_fft_top_8 (
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
        .done           (done)
    );

endmodule
