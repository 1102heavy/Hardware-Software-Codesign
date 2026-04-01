// ─────────────────────────────────────────────────────────────────────────
// fft_controller.v  (rev 3 – AXI4-Stream interface)
//
// Moore FSM controller for a 1024-point radix-2 DIT FFT.
// Replaces the simple data_valid/din/dout handshake with proper
// AXI4-Stream slave (input) and master (output) interfaces.
//
// AXI-Stream conventions used:
//   s_axis_tdata = { din_real[15:0], din_imag[15:0] }  (upper = real)
//   m_axis_tdata = { dout_real[15:0], dout_imag[15:0] }
//   s_axis_tlast : asserted with the Nth (last) input sample
//   m_axis_tlast : asserted with the Nth (last) output sample
//   s_axis_tready: de-asserted outside S_LOAD (back-pressure to master)
//   m_axis_tready: back-pressure input from downstream consumer
//
// Bugs fixed vs. rev 1 (carried forward from rev 2):
//   Bug 1 – 11-bit group_size prevents 10-bit wrap at stage 9
//   Bug 2 – S_LATCH state ensures butterfly inputs are stable before capture
//   Bug 3 – S_PRE_OUTPUT prefetches address 0 to remove output latency
//
// Parameters:
//   N      = 1024
//   LOG2N  = 10
//   DWIDTH = 16  (Q1.15)
// ─────────────────────────────────────────────────────────────────────────
`timescale 1ns/1ps

module fft_controller #(
    parameter N      = 1024,
    parameter LOG2N  = 10,
    parameter DWIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,   // active-low reset (connect to peripheral_aresetn)

    // ── AXI4-Stream Slave (input samples) ─────────────────────────────
    input  wire                       s_axis_tvalid,
    output reg                        s_axis_tready,
    input  wire [2*DWIDTH-1:0]        s_axis_tdata,   // {real[15:0], imag[15:0]}
    input  wire                       s_axis_tlast,   // high on sample N-1

    // ── AXI4-Stream Master (output spectrum) ──────────────────────────
    output reg                        m_axis_tvalid,
    input  wire                       m_axis_tready,
    output reg  [2*DWIDTH-1:0]        m_axis_tdata,   // {real[15:0], imag[15:0]}
    output reg                        m_axis_tlast,   // high on bin N-1

    // ── Data RAM interface ─────────────────────────────────────────────
    output reg  [LOG2N-1:0]           ram_addr_a,
    output reg  [LOG2N-1:0]           ram_addr_b,
    output reg  [2*DWIDTH-1:0]        ram_din_a,
    output reg  [2*DWIDTH-1:0]        ram_din_b,
    output reg                        ram_we_a,
    output reg                        ram_we_b,
    input  wire [2*DWIDTH-1:0]        ram_dout_a,
    input  wire [2*DWIDTH-1:0]        ram_dout_b,

    // ── Twiddle ROM interface ──────────────────────────────────────────
    output reg  [LOG2N-2:0]           rom_addr,
    input  wire signed [DWIDTH-1:0]   rom_w_real,
    input  wire signed [DWIDTH-1:0]   rom_w_imag,

    // ── Butterfly unit interface ───────────────────────────────────────
    output reg  signed [DWIDTH-1:0]   bfly_a_r, bfly_a_i,
    output reg  signed [DWIDTH-1:0]   bfly_b_r, bfly_b_i,
    output reg  signed [DWIDTH-1:0]   bfly_w_r, bfly_w_i,
    input  wire signed [DWIDTH:0]     bfly_x_r, bfly_x_i,
    input  wire signed [DWIDTH:0]     bfly_y_r, bfly_y_i,

    // ── Status ────────────────────────────────────────────────────────
    output reg                        done
);

    // ── State encoding ────────────────────────────────────────────────
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_LOAD       = 4'd1,   // accept input via AXI-S slave
        S_READ       = 4'd2,   // present RAM/ROM addresses
        S_LATCH      = 4'd3,   // latch RAM/ROM outputs into bfly inputs
        S_BFLY       = 4'd4,   // capture combinational bfly outputs
        S_WRITE      = 4'd5,   // write butterfly results back to RAM
        S_ADVANCE    = 4'd6,   // advance counters to next butterfly
        S_PRE_OUTPUT = 4'd7,   // prefetch address 0 (fix output latency)
        S_OUTPUT     = 4'd8,   // stream out via AXI-S master (with back-pressure)
        S_DONE       = 4'd9;

    reg [3:0] state, next_state;

    // ── Control counters ───────────────────────────────────────────────
    reg [LOG2N-1:0] sample_cnt;
    reg [3:0]        stage_reg;
    reg [LOG2N-1:0] group_start;
    reg [LOG2N-1:0] pair_idx;

    // ── Address generation (combinational) ────────────────────────────
    wire [LOG2N-1:0] span         = {{(LOG2N-1){1'b0}}, 1'b1} << stage_reg;
    wire [LOG2N:0]   group_size   = {1'b0, span} << 1;          // 11-bit
    wire [LOG2N-1:0] addr_A       = group_start + pair_idx;
    wire [LOG2N-1:0] addr_B       = addr_A + span;
    wire [LOG2N-2:0] tw_addr_comb = pair_idx[LOG2N-2:0] << ((LOG2N-1) - stage_reg);

    // ── Bit-reverse function ───────────────────────────────────────────
    function automatic [LOG2N-1:0] bit_rev;
        input [LOG2N-1:0] in_val;
        integer i;
        begin
            for (i = 0; i < LOG2N; i = i + 1)
                bit_rev[i] = in_val[LOG2N-1-i];
        end
    endfunction

    // ── Registered butterfly results ──────────────────────────────────
    reg signed [DWIDTH:0] reg_x_r, reg_x_i;
    reg signed [DWIDTH:0] reg_y_r, reg_y_i;

    // ══════════════════════════════════════════════════════════════════
    // STATE REGISTER
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // ══════════════════════════════════════════════════════════════════
    // NEXT-STATE LOGIC (combinational)
    // ══════════════════════════════════════════════════════════════════
    always @(*) begin
        next_state = state;

        case (state)

            // Auto-start when first sample arrives
            S_IDLE: if (s_axis_tvalid) next_state = S_LOAD;

            // Accept until tlast (last of N samples)
            S_LOAD: if (s_axis_tvalid && s_axis_tready && s_axis_tlast)
                        next_state = S_READ;

            S_READ:    next_state = S_LATCH;
            S_LATCH:   next_state = S_BFLY;
            S_BFLY:    next_state = S_WRITE;
            S_WRITE:   next_state = S_ADVANCE;

            S_ADVANCE: begin
                if (pair_idx + 1 < span)
                    next_state = S_READ;
                else if (({1'b0, group_start} + group_size) < N)
                    next_state = S_READ;
                else if (stage_reg < LOG2N - 1)
                    next_state = S_READ;
                else
                    next_state = S_PRE_OUTPUT;
            end

            S_PRE_OUTPUT: next_state = S_OUTPUT;

            // Stay in S_OUTPUT until last sample handshake completes
            S_OUTPUT: if (sample_cnt == N-1 && m_axis_tready)
                          next_state = S_DONE;

            S_DONE:    next_state = S_IDLE;
            default:   next_state = S_IDLE;

        endcase
    end

    // ══════════════════════════════════════════════════════════════════
    // OUTPUT LOGIC (Moore, registered)
    // ══════════════════════════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axis_tready <= 0;
            m_axis_tvalid <= 0;  m_axis_tdata  <= 0;  m_axis_tlast  <= 0;
            ram_addr_a    <= 0;  ram_addr_b    <= 0;
            ram_din_a     <= 0;  ram_din_b     <= 0;
            ram_we_a      <= 0;  ram_we_b      <= 0;
            rom_addr      <= 0;
            bfly_a_r      <= 0;  bfly_a_i      <= 0;
            bfly_b_r      <= 0;  bfly_b_i      <= 0;
            bfly_w_r      <= 0;  bfly_w_i      <= 0;
            reg_x_r       <= 0;  reg_x_i       <= 0;
            reg_y_r       <= 0;  reg_y_i       <= 0;
            done          <= 0;
            sample_cnt    <= 0;
            stage_reg     <= 0;
            group_start   <= 0;
            pair_idx      <= 0;
        end else begin
            // ── Strobes cleared every cycle (set only in the state that needs them) ──
            ram_we_a      <= 0;
            ram_we_b      <= 0;
            s_axis_tready <= 0;
            m_axis_tvalid <= 0;
            done          <= 0;

            case (state)

                // ──────────────────────────────────────────────────────
                S_IDLE: begin
                    // Nothing — waiting for s_axis_tvalid
                end

                // ──────────────────────────────────────────────────────
                // Assert tready; accept samples on tvalid & tready.
                // Use tdata = {real, imag}; write to bit-reversed address.
                // Counters reset on tlast ready for FFT computation.
                S_LOAD: begin
                    s_axis_tready <= 1'b1;

                    if (s_axis_tvalid && s_axis_tready) begin
                        ram_addr_a <= bit_rev(sample_cnt);
                        ram_din_a  <= s_axis_tdata;
                        ram_we_a   <= 1'b1;

                        if (s_axis_tlast) begin
                            sample_cnt  <= 0;
                            stage_reg   <= 0;
                            group_start <= 0;
                            pair_idx    <= 0;
                        end else begin
                            sample_cnt <= sample_cnt + 1;
                        end
                    end
                end

                // ──────────────────────────────────────────────────────
                // Present addresses; data ready one cycle later (S_LATCH).
                S_READ: begin
                    ram_addr_a <= addr_A;
                    ram_addr_b <= addr_B;
                    rom_addr   <= tw_addr_comb;
                end

                // ──────────────────────────────────────────────────────
                // Latch RAM/ROM outputs into butterfly input registers.
                S_LATCH: begin
                    bfly_a_r <= $signed(ram_dout_a[2*DWIDTH-1 : DWIDTH]);
                    bfly_a_i <= $signed(ram_dout_a[DWIDTH-1   : 0]);
                    bfly_b_r <= $signed(ram_dout_b[2*DWIDTH-1 : DWIDTH]);
                    bfly_b_i <= $signed(ram_dout_b[DWIDTH-1   : 0]);
                    bfly_w_r <= rom_w_real;
                    bfly_w_i <= rom_w_imag;
                end

                // ──────────────────────────────────────────────────────
                // Butterfly inputs are stable; capture outputs.
                S_BFLY: begin
                    reg_x_r <= bfly_x_r;
                    reg_x_i <= bfly_x_i;
                    reg_y_r <= bfly_y_r;
                    reg_y_i <= bfly_y_i;
                end

                // ──────────────────────────────────────────────────────
                // Write results (÷2 scaled) back to both RAM ports.
                S_WRITE: begin
                    ram_addr_a <= addr_A;
                    ram_din_a  <= {reg_x_r[DWIDTH:1], reg_x_i[DWIDTH:1]};
                    ram_we_a   <= 1'b1;

                    ram_addr_b <= addr_B;
                    ram_din_b  <= {reg_y_r[DWIDTH:1], reg_y_i[DWIDTH:1]};
                    ram_we_b   <= 1'b1;
                end

                // ──────────────────────────────────────────────────────
                // Advance: pair_idx → group_start → stage (priority order).
                S_ADVANCE: begin
                    if (pair_idx + 1 < span) begin
                        pair_idx <= pair_idx + 1;
                    end else if (({1'b0, group_start} + group_size) < N) begin
                        pair_idx    <= 0;
                        group_start <= {1'b0, group_start} + group_size;
                    end else if (stage_reg < LOG2N - 1) begin
                        stage_reg   <= stage_reg + 1;
                        pair_idx    <= 0;
                        group_start <= 0;
                    end
                end

                // ──────────────────────────────────────────────────────
                // Prefetch address 0 so ram_dout_a is valid on S_OUTPUT entry.
                S_PRE_OUTPUT: begin
                    ram_addr_a <= 0;
                    sample_cnt <= 0;
                end

                // ──────────────────────────────────────────────────────
                // Stream FFT output with full AXI-S back-pressure support.
                // ram_dout_a always holds data for the current sample_cnt
                // because the address was presented one cycle earlier.
                // Stall (hold everything) when m_axis_tready is low.
                S_OUTPUT: begin
                    m_axis_tvalid <= 1'b1;
                    m_axis_tdata  <= ram_dout_a;               // stable during stall
                    m_axis_tlast  <= (sample_cnt == N - 1);

                    if (m_axis_tready) begin
                        // Downstream accepted current sample; advance to next
                        if (sample_cnt < N - 1) begin
                            ram_addr_a <= sample_cnt + 1;      // prefetch next
                            sample_cnt <= sample_cnt + 1;
                        end
                        // sample_cnt == N-1: next_state logic moves to S_DONE
                    end
                end

                // ──────────────────────────────────────────────────────
                S_DONE: begin
                    done          <= 1'b1;
                    m_axis_tvalid <= 1'b0;
                    // Reset sample_cnt here so the next S_LOAD starts at 0.
                    // S_OUTPUT leaves sample_cnt at N-1; without this reset,
                    // a second FFT run stores x[0] at bit_rev(N-1) instead
                    // of bit_rev(0), shifting the impulse by N-1 samples.
                    sample_cnt    <= 0;
                end

                default: ;

            endcase
        end
    end

endmodule
