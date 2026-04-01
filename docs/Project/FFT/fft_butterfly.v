// ─────────────────────────────────────────────────────────────────────────
// fft_butterfly.v
//
// Combinational radix-2 DIT butterfly.
//
// Inputs  (all registered by caller before asserting):
//   a_r, a_i  : top input A = a_r + j*a_i        (Q1.15, 16 bits)
//   b_r, b_i  : bottom input B = b_r + j*b_i      (Q1.15, 16 bits)
//   w_r, w_i  : twiddle W = w_r + j*w_i           (Q1.15, 16 bits)
//
// Outputs (valid combinationally, must be registered by caller):
//   x_r, x_i  : A' = A + W*B                      (Q1.15 + 1 overflow bit, 17 bits)
//   y_r, y_i  : B' = A - W*B                      (Q1.15 + 1 overflow bit, 17 bits)
//
// The extra MSB is the overflow guard bit. The caller discards bit 0
// (right-shift by 1) when writing back, performing the per-stage scaling.
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_butterfly #(
    parameter DWIDTH = 16   // data word width (Q1.15)
)(
    input  wire signed [DWIDTH-1:0] a_r, a_i,
    input  wire signed [DWIDTH-1:0] b_r, b_i,
    input  wire signed [DWIDTH-1:0] w_r, w_i,

    output wire signed [DWIDTH:0]   x_r, x_i,   // A' = A + W*B  (one guard bit)
    output wire signed [DWIDTH:0]   y_r, y_i    // B' = A - W*B
);

    // ── Step 1: four 16×16 multiplications → 32-bit products (Q2.30) ──
    wire signed [2*DWIDTH-1:0] m_rr = b_r * w_r;   // Re*Re
    wire signed [2*DWIDTH-1:0] m_ii = b_i * w_i;   // Im*Im
    wire signed [2*DWIDTH-1:0] m_ri = b_r * w_i;   // Re*Im (cross)
    wire signed [2*DWIDTH-1:0] m_ir = b_i * w_r;   // Im*Re (cross)

    // ── Step 2: combine into real and imaginary parts of W*B (Q2.30) ──
    wire signed [2*DWIDTH-1:0] prod_r = m_rr - m_ii;   // Re(W*B) = Wr*Br - Wi*Bi
    wire signed [2*DWIDTH-1:0] prod_i = m_ri + m_ir;   // Im(W*B) = Wr*Bi + Wi*Br

    // ── Step 3: truncate Q2.30 → Q1.15 by discarding the 15 LSBs ──────
    // Arithmetic right-shift by 15: take bits [30:15]
    wire signed [DWIDTH-1:0] t_r = prod_r[2*DWIDTH-2 : DWIDTH-1];
    wire signed [DWIDTH-1:0] t_i = prod_i[2*DWIDTH-2 : DWIDTH-1];

    // ── Step 4: butterfly add/subtract ──────────────────────────────────
    assign x_r = {a_r[DWIDTH-1], a_r} + {t_r[DWIDTH-1], t_r};   // sign-extend then add
    assign x_i = {a_i[DWIDTH-1], a_i} + {t_i[DWIDTH-1], t_i};
    assign y_r = {a_r[DWIDTH-1], a_r} - {t_r[DWIDTH-1], t_r};
    assign y_i = {a_i[DWIDTH-1], a_i} - {t_i[DWIDTH-1], t_i};

endmodule